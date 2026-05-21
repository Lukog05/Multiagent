"""
LNS2 (Large Neighborhood Search 2) anytime post-processor.

For MAPF (no-box) plans
-----------------------
Iterative destroy-and-repair using space-time constrained A*:
  1. Pick a random subset of 2–4 agents (biased toward the makespan bottleneck).
  2. Build space-time constraints from every agent NOT in the subset.
  3. Replan each subset agent with _constrained_astar; each newly replanned
     agent's path is immediately added to the constraint set for the next.
  4. Accept the candidate if total makespan strictly decreases.
  5. Repeat until the wall-clock deadline.

For box plans
-------------
"Restart improvement": try WA*(weight=2) with the remaining budget.
WA*(5) tends to be greedy; WA*(2) explores more and often finds shorter
solutions.  Only accepted if strictly shorter than the incumbent.
This is not classic LNS2 but provides an honest anytime improvement step
for box levels.
"""

import random
import sys
import time
from typing import TYPE_CHECKING

from searchclient.action import Action, ActionType
from searchclient.state import State

if TYPE_CHECKING:
    pass


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def lns2_improve(
    initial_plan: list[list[Action]],
    initial_state: "State",
    deadline: float,
    is_mapf: bool = False,
) -> list[list[Action]]:
    """
    Attempt to shorten *initial_plan*.

    Parameters
    ----------
    initial_plan  : plan returned by the cascade
    initial_state : initial level state (agent positions, goals, walls)
    deadline      : wall-clock time (perf_counter) at which we must stop
    is_mapf       : True for no-box MAPF levels, False for box levels

    Returns the improved plan, or *initial_plan* unchanged if no improvement
    was found or if any exception occurs.
    """
    remaining = deadline - time.perf_counter()
    if remaining < 1.0:
        return initial_plan

    try:
        if is_mapf:
            return _lns2_mapf(initial_plan, initial_state, deadline)
        else:
            return _lns2_box(initial_plan, initial_state, deadline)
    except Exception as exc:  # noqa: BLE001
        print(f"[lns2] Exception ({type(exc).__name__}: {exc}), keeping original plan.",
              file=sys.stderr, flush=True)
        return initial_plan


# ---------------------------------------------------------------------------
# MAPF LNS2
# ---------------------------------------------------------------------------

def _get_agent_goals(initial_state: "State") -> dict[int, tuple[int, int]]:
    """Return goal cell per agent (defaults to start cell for goal-less agents)."""
    n = len(initial_state.agent_rows)
    goals: dict[int, tuple[int, int]] = {}
    for r, row in enumerate(State.goals):
        for c, ch in enumerate(row):
            if "0" <= ch <= "9":
                idx = ord(ch) - ord("0")
                if idx < n:
                    goals[idx] = (r, c)
    # Agents without an explicit goal: stay at their start cell.
    for i in range(n):
        if i not in goals:
            goals[i] = (initial_state.agent_rows[i], initial_state.agent_cols[i])
    return goals


def _plan_to_paths(
    initial_state: "State",
    plan: list[list[Action]],
) -> list[list[tuple[int, int]]]:
    """
    Convert a MAPF joint-action plan to per-agent position paths.

    Each path is a list of (row, col) tuples starting with the initial
    position; length = len(plan) + 1.

    Only uses agent_row_delta / agent_col_delta — safe for Move/NoOp actions.
    """
    n = len(initial_state.agent_rows)
    paths: list[list[tuple[int, int]]] = [
        [(initial_state.agent_rows[i], initial_state.agent_cols[i])]
        for i in range(n)
    ]
    for joint in plan:
        for i, action in enumerate(joint):
            r, c = paths[i][-1]
            paths[i].append((r + action.agent_row_delta, c + action.agent_col_delta))
    return paths


def _paths_to_joint_actions(
    paths: list[list[tuple[int, int]]],
    num_agents: int,
) -> list[list[Action]]:
    """Convert per-agent position paths to a joint-action plan."""
    DELTA_TO_ACTION: dict[tuple[int, int], Action] = {
        (-1, 0): Action.MoveN,
        (1,  0): Action.MoveS,
        (0,  1): Action.MoveE,
        (0, -1): Action.MoveW,
        (0,  0): Action.NoOp,
    }
    max_len = max(len(p) for p in paths)
    joint_actions: list[list[Action]] = []
    for t in range(1, max_len):
        step: list[Action] = []
        for i in range(num_agents):
            path = paths[i]
            if t < len(path):
                pr, pc = path[t - 1]
                cr, cc = path[t]
                step.append(DELTA_TO_ACTION[(cr - pr, cc - pc)])
            else:
                step.append(Action.NoOp)
        joint_actions.append(step)
    return joint_actions


def _build_constraints(
    paths: list[list[tuple[int, int]]],
    fixed_agents: list[int],
    total_t: int,
) -> tuple[set, set]:
    """
    Return (vertex_constraints, edge_constraints) sets from fixed agents.

    Vertex constraint: (r, c, t)          — cell (r,c) is blocked at time t.
    Edge constraint:   (pr, pc, cr, cc, t) — move from (pr,pc)→(cr,cc) at
                       departure time t is blocked for any other agent (prevents swap).
    NOTE: t is the *departure* timestep (consistent with _constrained_astar).
    """
    vc: set = set()
    ec: set = set()
    for i in fixed_agents:
        path = paths[i]
        # Vertex reservation for each occupied cell.
        for t, (r, c) in enumerate(path):
            vc.add((r, c, t))
        # After path ends agent stays at goal.
        gr, gc = path[-1]
        for t in range(len(path), total_t + 1):
            vc.add((gr, gc, t))
        # Edge / swap prevention.
        # Agent at path[t-1]=(pr,pc) moves to path[t]=(cr,cc) at departure t-1.
        for t in range(1, len(path)):
            pr, pc = path[t - 1]
            cr, cc = path[t]
            ec.add((pr, pc, cr, cc, t - 1))  # (from, to, departure_t)
    return vc, ec


def _validate_mapf_paths(
    paths: list[list[tuple[int, int]]],
    agent_goals: dict[int, tuple[int, int]],
) -> bool:
    """
    Return True if paths are conflict-free and all agents reach their goals.
    Checks vertex conflicts, edge/swap conflicts, and goal attainment.
    """
    n = len(paths)
    max_t = max(len(p) for p in paths)

    def pos_at(i: int, t: int) -> tuple[int, int]:
        return paths[i][min(t, len(paths[i]) - 1)]

    for t in range(max_t + 1):
        positions = [pos_at(i, t) for i in range(n)]
        # Vertex conflicts.
        if len(set(positions)) < n:
            return False
        # Edge / swap conflicts.
        if t > 0:
            for i in range(n):
                for j in range(i + 1, n):
                    if pos_at(i, t) == pos_at(j, t - 1) and pos_at(i, t - 1) == pos_at(j, t):
                        return False

    # All agents must be at their goals at the end.
    for i in range(n):
        if paths[i][-1] != agent_goals[i]:
            return False

    return True


def _lns2_mapf(
    plan: list[list[Action]],
    initial_state: "State",
    deadline: float,
) -> list[list[Action]]:
    """LNS2 improve loop for MAPF (no-box) plans."""
    from searchclient.cbs import _bfs_distances, _constrained_astar  # local import avoids cycles

    n = len(initial_state.agent_rows)
    if n < 2:
        return plan  # Nothing to improve with a single agent.

    agent_goals = _get_agent_goals(initial_state)
    dist_grids = {i: _bfs_distances(*agent_goals[i]) for i in range(n)}

    paths = _plan_to_paths(initial_state, plan)
    best_makespan = max(len(p) for p in paths) - 1
    best_paths = [p[:] for p in paths]

    iters = 0
    improvements = 0

    print(f"[lns2] Starting MAPF LNS2 (makespan={best_makespan}, agents={n}, "
          f"budget={deadline - time.perf_counter():.1f}s)", file=sys.stderr, flush=True)

    while time.perf_counter() < deadline:
        # ── Select neighborhood ──────────────────────────────────────────────
        k = random.randint(2, min(4, n))
        # Bias selection toward agents with longer paths (makespan bottleneck).
        path_lens = [len(paths[i]) for i in range(n)]
        total_w = sum(path_lens)
        if total_w == 0:
            break
        # Weighted sampling without replacement.
        cumulative = []
        running = 0.0
        for w in path_lens:
            running += w / total_w
            cumulative.append(running)

        subset_set: set[int] = set()
        attempts = 0
        while len(subset_set) < k and attempts < k * 10:
            r_val = random.random()
            for idx, cum in enumerate(cumulative):
                if r_val <= cum:
                    subset_set.add(idx)
                    break
            attempts += 1
        if len(subset_set) < 2:
            # Fall back to uniform random sample.
            subset_set = set(random.sample(range(n), k))
        subset = list(subset_set)
        fixed = [i for i in range(n) if i not in subset_set]

        current_makespan = max(len(paths[i]) for i in range(n)) - 1
        # Allow up to current makespan timesteps (we only accept improvement).
        total_t = current_makespan

        # ── Build constraints from fixed agents ──────────────────────────────
        vc, ec = _build_constraints(paths, fixed, total_t)

        # ── Replan each agent in subset ──────────────────────────────────────
        new_sub_paths: dict[int, list[tuple[int, int]]] = {}
        failed = False
        for i in subset:
            if time.perf_counter() >= deadline:
                failed = True
                break
            new_path = _constrained_astar(
                initial_state.agent_rows[i], initial_state.agent_cols[i],
                agent_goals[i][0], agent_goals[i][1],
                dist_grids[i],
                frozenset(vc), frozenset(ec),
                max_t=total_t,
            )
            if new_path is None:
                failed = True
                break
            new_sub_paths[i] = new_path
            # Add this replanned agent's path to constraints for subsequent subset agents.
            for t, (r, c) in enumerate(new_path):
                vc.add((r, c, t))
            gr, gc = new_path[-1]
            for t in range(len(new_path), total_t + 1):
                vc.add((gr, gc, t))
            for t in range(1, len(new_path)):
                pr, pc = new_path[t - 1]
                cr, cc = new_path[t]
                ec.add((pr, pc, cr, cc, t - 1))  # (from, to, departure_t)

        if not failed and len(new_sub_paths) == len(subset):
            candidate = [new_sub_paths.get(i, paths[i]) for i in range(n)]
            new_makespan = max(len(p) for p in candidate) - 1
            if new_makespan < best_makespan:
                # Validate before accepting.
                if _validate_mapf_paths(candidate, agent_goals):
                    paths = candidate
                    best_makespan = new_makespan
                    best_paths = [p[:] for p in paths]
                    improvements += 1
                    print(f"[lns2] iter {iters}: makespan → {best_makespan} ✓",
                          file=sys.stderr, flush=True)

        iters += 1

    print(f"[lns2] Done. {iters} iterations, {improvements} improvements, "
          f"final makespan={best_makespan}.", file=sys.stderr, flush=True)

    if improvements == 0:
        return plan  # Return original unchanged.
    return _paths_to_joint_actions(best_paths, n)


# ---------------------------------------------------------------------------
# Box-plan restart improvement
# ---------------------------------------------------------------------------

def _lns2_box(
    plan: list[list[Action]],
    initial_state: "State",
    deadline: float,
) -> list[list[Action]]:
    from searchclient.ma_planner import decoupled_box_plan

    remaining = deadline - time.perf_counter()
    print(f"[lns2] Starting box multi-attempt LNS (budget={remaining:.1f}s, incumbent={len(plan)})",
          file=sys.stderr, flush=True)

    best_plan = plan
    attempt = 0
    while time.perf_counter() < deadline - 5.0:
        budget_this = min((deadline - time.perf_counter() - 5.0) * 0.4, 30.0)
        if budget_this < 3.0:
            break
        t0 = time.perf_counter()
        candidate = decoupled_box_plan(initial_state, time.perf_counter() + budget_this)
        elapsed = time.perf_counter() - t0
        attempt += 1
        if candidate is not None and len(candidate) < len(best_plan):
            best_plan = candidate
            print(f"[lns2] attempt {attempt}: improved → {len(best_plan)} ({elapsed:.2f}s)",
                  file=sys.stderr, flush=True)
        else:
            print(f"[lns2] attempt {attempt}: {'None' if candidate is None else len(candidate)} "
                  f"(no improvement, {elapsed:.2f}s)", file=sys.stderr, flush=True)

    return best_plan
