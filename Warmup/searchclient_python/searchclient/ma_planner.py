"""
Decoupled multi-agent box planner for hard MA levels.

Joint search fails on levels with many agents + boxes because the state space
grows exponentially.  This module decomposes the problem:

  1. Find every unsatisfied (box_letter, box_pos, goal_pos) triple.
  2. Greedily assign each triple to the nearest compatible agent.
  3. Plan each assignment independently with A* on the compact
     (agent_r, agent_c, box_r, box_c) sub-space, treating all other
     boxes and agents as unmovable walls.
  4. Execute plans sequentially — one agent acts at a time while the
     others issue NoOp — so there are never simultaneous conflicts.

Improvements over the base version:
  - Stale-box guard: validates box is still at expected position before planning
  - Recursive blocker resolution: when a task is blocked by box B that has its
    own goal, tries to complete B's goal task first (up to depth 4 recursion)
  - Dependency-aware ordering: tasks are topologically sorted so tasks blocking
    others are attempted before the tasks they block
  - Permissive path clearing: when completely stuck, plans ignoring non-goal
    boxes to find theoretical paths, then clears blockers one by one
  - Better parking: allows parking at a blocker's own goal cell when feasible

The resulting plan is always valid (each step is verified against
State.is_applicable / is_conflicting before use).
"""

from __future__ import annotations

import heapq
import sys
import time
from collections import deque

from searchclient.action import Action, ActionType
from searchclient.state import State

# ── Lookup tables built at import time ────────────────────────────────────────
_MOVE: dict[tuple[int, int], Action] = {
    (-1, 0): Action.MoveN,
    (1,  0): Action.MoveS,
    (0,  1): Action.MoveE,
    (0, -1): Action.MoveW,
}

# (agent_dr, agent_dc, box_dr, box_dc) → Push Action
_PUSH: dict[tuple[int, int, int, int], Action] = {
    (a.agent_row_delta, a.agent_col_delta, a.box_row_delta, a.box_col_delta): a
    for a in Action
    if a.type is ActionType.Push
}

# (agent_dr, agent_dc, box_dr, box_dc) → Pull Action
# Box source = (agent_row - box_dr, agent_col - box_dc); box lands at old agent cell.
_PULL: dict[tuple[int, int, int, int], Action] = {
    (a.agent_row_delta, a.agent_col_delta, a.box_row_delta, a.box_col_delta): a
    for a in Action
    if a.type is ActionType.Pull
}

_INF = 10_000_000
_DIRS = [(-1, 0), (1, 0), (0, 1), (0, -1)]


# ── BFS utility ───────────────────────────────────────────────────────────────

def _bfs_from(tr: int, tc: int, extra_walls: frozenset[tuple[int, int]]) -> list[list[int]]:
    """BFS distances from (tr, tc) treating extra_walls as additional walls."""
    rows = len(State.walls)
    cols = len(State.walls[0])
    dist = [[_INF] * cols for _ in range(rows)]
    if State.walls[tr][tc] or (tr, tc) in extra_walls:
        return dist
    dist[tr][tc] = 0
    q: deque[tuple[int, int]] = deque([(tr, tc)])
    while q:
        r, c = q.popleft()
        for dr, dc in _DIRS:
            nr, nc = r + dr, c + dc
            if (
                0 <= nr < rows
                and 0 <= nc < cols
                and not State.walls[nr][nc]
                and (nr, nc) not in extra_walls
                and dist[nr][nc] == _INF
            ):
                dist[nr][nc] = dist[r][c] + 1
                q.append((nr, nc))
    return dist


# ── Single-task A* ────────────────────────────────────────────────────────────

def _plan_task(
    ar: int, ac: int,
    br: int, bc: int,
    gr: int, gc: int,
    extra_walls: frozenset[tuple[int, int]],
    deadline: float,
) -> list[Action] | None:
    """
    A* on (agent_r, agent_c, box_r, box_c) to push the box from (br,bc) to (gr,gc).

    extra_walls  — cells the agent/box cannot enter (other boxes + other agents).
    Returns a list of single-agent Actions (Move/Push), or None on timeout/failure.
    """
    rows = len(State.walls)
    cols = len(State.walls[0])

    goal_dist = _bfs_from(gr, gc, extra_walls)
    if goal_dist[br][bc] == _INF:
        # Also check without extra_walls to distinguish wall-blocked vs box-blocked
        goal_dist_noextra = _bfs_from(gr, gc, frozenset())
        reason = "wall-blocked" if goal_dist_noextra[br][bc] == _INF else "box-blocked"
        print(f"[plan_task] ({ar},{ac}) push ({br},{bc})→({gr},{gc}): unreachable ({reason})", file=sys.stderr, flush=True)
        return None  # box can't reach goal given current obstacles

    start = (ar, ac, br, bc)
    g_cost: dict[tuple, int] = {start: 0}
    parent: dict[tuple, tuple] = {start: (None, None)}  # state → (parent, action)
    ctr = 0
    heap: list = [(goal_dist[br][bc], 0, ctr, start)]

    while heap:
        if time.perf_counter() > deadline:
            return None

        _, g, _, state = heapq.heappop(heap)
        ar_, ac_, br_, bc_ = state

        if g > g_cost.get(state, _INF):
            continue

        # Goal reached when box is at (gr, gc)
        if br_ == gr and bc_ == gc:
            acts: list[Action] = []
            cur = state
            while parent[cur][0] is not None:
                prev, act = parent[cur]
                acts.append(act)
                cur = prev
            acts.reverse()
            return acts

        # ── Move: agent moves, box stays ──────────────────────────────────────
        for dr, dc in _DIRS:
            nar, nac = ar_ + dr, ac_ + dc
            if not (0 <= nar < rows and 0 <= nac < cols):
                continue
            if State.walls[nar][nac] or (nar, nac) in extra_walls:
                continue
            if nar == br_ and nac == bc_:
                continue  # stepping onto box requires a push
            ns = (nar, nac, br_, bc_)
            ng = g + 1
            if ng < g_cost.get(ns, _INF):
                g_cost[ns] = ng
                ctr += 1
                heapq.heappush(heap, (ng + goal_dist[br_][bc_], ng, ctr, ns))
                parent[ns] = (state, _MOVE[(dr, dc)])

        # ── Push: agent steps onto box, box moves further ─────────────────────
        adr, adc = br_ - ar_, bc_ - ac_
        if abs(adr) + abs(adc) == 1:  # agent is directly adjacent to box
            for bdr, bdc in _DIRS:
                key = (adr, adc, bdr, bdc)
                if key not in _PUSH:
                    continue
                nbr, nbc = br_ + bdr, bc_ + bdc
                if not (0 <= nbr < rows and 0 <= nbc < cols):
                    continue
                if State.walls[nbr][nbc] or (nbr, nbc) in extra_walls:
                    continue
                ns = (br_, bc_, nbr, nbc)  # agent lands on old box cell
                ng = g + 1
                if ng < g_cost.get(ns, _INF):
                    g_cost[ns] = ng
                    ctr += 1
                    heapq.heappush(heap, (ng + goal_dist[nbr][nbc], ng, ctr, ns))
                    parent[ns] = (state, _PUSH[key])

        # ── Pull: agent moves, dragging adjacent box behind it ────────────────
        # Box source = (ar - box_dr, ac - box_dc); box lands at old agent cell.
        for (adr2, adc2, bdr2, bdc2), pull_act in _PULL.items():
            # Box must be at (ar - bdr2, ac - bdc2) relative to current agent
            req_br, req_bc = ar_ - bdr2, ac_ - bdc2
            if req_br != br_ or req_bc != bc_:
                continue
            # New agent position
            nar, nac = ar_ + adr2, ac_ + adc2
            if not (0 <= nar < rows and 0 <= nac < cols):
                continue
            if State.walls[nar][nac] or (nar, nac) in extra_walls:
                continue
            # New box position = old agent cell (ar_, ac_)
            nbr, nbc = ar_, ac_
            # (ar_, ac_) was the agent cell — not a wall, not in extra_walls (agent was there)
            ns = (nar, nac, nbr, nbc)
            ng = g + 1
            if ng < g_cost.get(ns, _INF):
                g_cost[ns] = ng
                ctr += 1
                heapq.heappush(heap, (ng + goal_dist[nbr][nbc], ng, ctr, ns))
                parent[ns] = (state, pull_act)

    return None  # exhausted search space


# ── Agent movement (BFS, box-aware) ──────────────────────────────────────────

def _plan_agent_move(
    ar: int, ac: int,
    gr: int, gc: int,
    extra_walls: frozenset[tuple[int, int]],
    deadline: float,
) -> list[Action] | None:
    """
    BFS path for a single agent from (ar,ac) to (gr,gc), treating extra_walls
    (which should include all box cells and other agents) as impassable.
    Returns a list of Move actions, or None if unreachable.
    """
    if ar == gr and ac == gc:
        return []
    rows = len(State.walls)
    cols = len(State.walls[0])
    if State.walls[gr][gc] or (gr, gc) in extra_walls:
        return None

    prev: dict[tuple[int, int], tuple] = {(ar, ac): (None, None)}
    q: deque[tuple[int, int]] = deque([(ar, ac)])
    while q:
        if time.perf_counter() > deadline:
            return None
        r, c = q.popleft()
        if r == gr and c == gc:
            acts: list[Action] = []
            cur = (r, c)
            while prev[cur][0] is not None:
                acts.append(prev[cur][1])
                cur = prev[cur][0]
            acts.reverse()
            return acts
        for dr, dc in _DIRS:
            nr, nc = r + dr, c + dc
            ns = (nr, nc)
            if (
                ns not in prev
                and 0 <= nr < rows
                and 0 <= nc < cols
                and not State.walls[nr][nc]
                and ns not in extra_walls
            ):
                prev[ns] = ((r, c), _MOVE[(dr, dc)])
                q.append(ns)
    return None


# ── Main entry point ──────────────────────────────────────────────────────────

def _bfs_ignore_boxes(tr: int, tc: int) -> list[list[int]]:
    """BFS distances ignoring all boxes (walls only)."""
    rows = len(State.walls)
    cols = len(State.walls[0])
    dist = [[_INF] * cols for _ in range(rows)]
    if State.walls[tr][tc]:
        return dist
    dist[tr][tc] = 0
    q: deque[tuple[int, int]] = deque([(tr, tc)])
    while q:
        r, c = q.popleft()
        for dr, dc in _DIRS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not State.walls[nr][nc] and dist[nr][nc] == _INF:
                dist[nr][nc] = dist[r][c] + 1
                q.append((nr, nc))
    return dist


def _trace_ideal_path(br: int, bc: int, gr: int, gc: int) -> list[tuple[int, int]]:
    """Greedy path from (br,bc) to (gr,gc) ignoring boxes (walls only)."""
    rows = len(State.walls)
    cols = len(State.walls[0])
    dist = _bfs_ignore_boxes(gr, gc)
    if dist[br][bc] == _INF:
        return []
    path: list[tuple[int, int]] = [(br, bc)]
    visited: set[tuple[int, int]] = {(br, bc)}
    r, c = br, bc
    while (r, c) != (gr, gc) and len(path) <= rows * cols:
        best: tuple[int, int] | None = None
        best_d = dist[r][c]
        for dr, dc in _DIRS:
            nr, nc = r + dr, c + dc
            if (
                0 <= nr < rows and 0 <= nc < cols
                and (nr, nc) not in visited
                and dist[nr][nc] < best_d
            ):
                best_d = dist[nr][nc]
                best = (nr, nc)
        if best is None:
            break
        r, c = best
        visited.add((r, c))
        path.append((r, c))
    return path


def _first_blocker_on_path(
    br: int, bc: int, gr: int, gc: int, state: State
) -> tuple[int, int, str] | None:
    """Return (r, c, letter) of the first blocking box on the ideal path, or None."""
    for r, c in _trace_ideal_path(br, bc, gr, gc):
        if (r, c) == (br, bc):
            continue
        if state.boxes[r][c]:
            return r, c, state.boxes[r][c]
    return None


def _plan_park(
    state: State,
    blocker_r: int,
    blocker_c: int,
    blocker_letter: str,
    forbidden: set[tuple[int, int]],
    num_agents: int,
    deadline: float,
    main_task_box: tuple[int, int] | None = None,
    main_task_goal: tuple[int, int] | None = None,
    blocker_own_goal: tuple[int, int] | None = None,
) -> tuple[int, list[Action], tuple[int, int]] | None:
    """
    Push the blocking box to the nearest cell NOT in *forbidden*.
    Returns (agent_idx, actions, parking_pos) or None.

    If blocker_own_goal is given and doesn't conflict with the main task
    endpoints, that cell is also considered as a candidate parking spot
    (allowing the blocker to reach its own goal via parking).
    """
    box_color = State.box_colors[ord(blocker_letter) - ord("A")]
    compatible = [i for i in range(num_agents) if State.agent_colors[i] == box_color]
    if not compatible:
        return None

    rows = len(State.walls)
    cols = len(State.walls[0])

    # Build effective forbidden set, potentially allowing blocker's own goal
    effective_forbidden = set(forbidden)
    if blocker_own_goal is not None:
        bg_r, bg_c = blocker_own_goal
        # Allow parking at own goal unless it's the main task's box position or goal
        if (main_task_box is None or blocker_own_goal != main_task_box) and \
           (main_task_goal is None or blocker_own_goal != main_task_goal):
            effective_forbidden.discard(blocker_own_goal)

    # BFS from blocker to find nearest free parking cell not in forbidden
    dist_from_blocker = _bfs_from(blocker_r, blocker_c, frozenset())
    candidates_park: list[tuple[int, int, int]] = []
    for r in range(rows):
        for c in range(cols):
            if (
                dist_from_blocker[r][c] < _INF
                and (r, c) not in effective_forbidden
                and not State.walls[r][c]
                and not state.boxes[r][c]
                and (r, c) != (blocker_r, blocker_c)
            ):
                candidates_park.append((dist_from_blocker[r][c], r, c))
    candidates_park.sort()

    PARK_BUDGET = 3.0
    for _, pr, pc in candidates_park[:15]:
        for park_agent in sorted(
            compatible,
            key=lambda i: abs(state.agent_rows[i] - blocker_r) + abs(state.agent_cols[i] - blocker_c),
        ):
            if time.perf_counter() > deadline - 1.0:
                return None
            par = state.agent_rows[park_agent]
            pac = state.agent_cols[park_agent]
            extra = frozenset(
                (r, c)
                for r in range(len(state.boxes))
                for c in range(len(state.boxes[r]))
                if state.boxes[r][c] and not (r == blocker_r and c == blocker_c)
            ) | frozenset(
                (state.agent_rows[j], state.agent_cols[j])
                for j in range(num_agents) if j != park_agent
            )
            acts = _plan_task(par, pac, blocker_r, blocker_c, pr, pc, extra, time.perf_counter() + PARK_BUDGET)
            if acts is not None:
                return park_agent, acts, (pr, pc)
    return None


def _dependency_sort_goals(
    unsatisfied: list[tuple[str, int, int]],
    state: "State",
) -> list[tuple[str, int, int]]:
    """
    Topologically sort unsatisfied goals so that tasks blocking others
    come first. Two types of dependencies:
      1. Box B currently sits on box A's ideal path → A depends on B.
      2. Goal cell of task Y lies on box X's ideal path → Y depends on X
         (X must finish before Y occupies its goal and blocks X's route).
    """
    # Build dependency graph: goal → set of goals it depends on
    deps: dict[tuple, set[tuple]] = {g: set() for g in unsatisfied}
    unsatisfied_set = set(unsatisfied)

    # Pre-build: goal cell → task that occupies it
    goal_cell_to_task: dict[tuple[int, int], tuple] = {
        (gr, gc): (letter, gr, gc) for letter, gr, gc in unsatisfied
    }

    for letter, gr, gc in unsatisfied:
        # Find current positions of this box letter that are not already at goal
        box_positions = [
            (r, c)
            for r in range(len(state.boxes))
            for c in range(len(state.boxes[r]))
            if state.boxes[r][c] == letter and not (r == gr and c == gc)
        ]
        for br, bc in box_positions:
            path = _trace_ideal_path(br, bc, gr, gc)
            for pr, pc in path[1:]:
                # Dependency type 1: another box currently sits on this path cell
                bl = state.boxes[pr][pc] if state.boxes[pr][pc] else ""
                if bl and bl != letter:
                    for other in unsatisfied_set:
                        if other[0] == bl and other != (letter, gr, gc):
                            deps[(letter, gr, gc)].add(other)
                            break

                # Dependency type 2: another task's goal destination is on this
                # path cell — that task must wait until after X finishes.
                other_task = goal_cell_to_task.get((pr, pc))
                if other_task and other_task != (letter, gr, gc):
                    # other_task's goal is on our path → other_task depends on us
                    deps[other_task].add((letter, gr, gc))

    # Kahn's algorithm (topological sort by in-degree)
    in_degree: dict[tuple, int] = {g: 0 for g in unsatisfied}
    for g, dep_set in deps.items():
        for dep in dep_set:
            if dep in in_degree:
                in_degree[g] += 1

    queue = deque([g for g in unsatisfied if in_degree[g] == 0])
    ordered: list[tuple[str, int, int]] = []
    while queue:
        node = queue.popleft()
        ordered.append(node)
        # Find nodes that depend on this node and reduce their in-degree
        for g in unsatisfied:
            if node in deps[g]:
                in_degree[g] -= 1
                if in_degree[g] == 0:
                    queue.append(g)

    # Append any remaining (cycle) nodes
    ordered_set = set(ordered)
    for g in unsatisfied:
        if g not in ordered_set:
            ordered.append(g)

    return ordered


def _parallelize_plan(joint_plan: list, initial_state: "State", num_agents: int) -> list:
    """
    Compress a sequential joint plan into a parallel plan where non-conflicting
    agents act simultaneously.

    Uses a step-based approach that preserves causal ordering:
    - Processes steps in original sequential order (sorted by step index).
    - For each parallel timestep, greedily merges applicable, non-conflicting
      steps into a combined action.
    - Causal safety: does NOT schedule a Push/Pull if it would permanently place
      a box onto a cell that an earlier pending step still needs to be free.
      This prevents box-swap and path-blocking deadlocks.
    """
    n = len(joint_plan)
    if n == 0:
        return joint_plan

    # Preprocess: each sequential step has at most one non-NoOp action.
    steps: list[tuple[int, object]] = []  # (agent_idx, action); (-1, NoOp) for all-NoOp
    for ja in joint_plan:
        found = False
        for i, act in enumerate(ja):
            if act != Action.NoOp:
                steps.append((i, act))
                found = True
                break
        if not found:
            steps.append((-1, Action.NoOp))

    # Precompute SEQUENTIAL agent positions at each step.
    # seq_pos[k] = (row, col) of the acting agent just BEFORE joint_plan[k] runs.
    seq_pos: list[tuple[int, int]] = []
    _sq = initial_state
    for k, (pi, _) in enumerate(steps):
        if pi >= 0:
            seq_pos.append((_sq.agent_rows[pi], _sq.agent_cols[pi]))
        else:
            seq_pos.append((-1, -1))
        _sq = _sq.result(joint_plan[k])

    state = initial_state
    new_plan: list = []
    remaining: set[int] = set(range(n))
    max_iters = n * 10 + 500
    _detour_steps = 0
    _max_detours = n * 5 + 500  # generous budget for done-agent clearance
    _last_detour: dict[int, object] = {}  # agent -> last detour action (anti-oscillation)

    for _iter in range(max_iters):
        if not remaining:
            break

        # Build "protected cells" map: cell -> smallest remaining step idx that
        # needs that cell to be free.  We refuse to schedule any action that
        # permanently places a box on a protected cell (causal safety).
        protected: dict[tuple[int, int], int] = {}
        for pidx in remaining:
            pi, pact = steps[pidx]
            if pi < 0:
                continue
            par, pac = seq_pos[pidx]
            if pact.type is ActionType.Push:
                cell = (par + pact.agent_row_delta + pact.box_row_delta,
                        pac + pact.agent_col_delta + pact.box_col_delta)
            elif pact.type is ActionType.Pull:
                cell = (par, pac)
            else:  # Move
                cell = (par + pact.agent_row_delta, pac + pact.agent_col_delta)
            if cell not in protected or pidx < protected[cell]:
                protected[cell] = pidx

        combined = [Action.NoOp] * num_agents
        consumed: list[int] = []

        # Each agent must execute its OWN steps in sequence.
        earliest_for_agent: dict[int, int] = {}
        for idx in sorted(remaining):
            ii, _ = steps[idx]
            if ii >= 0 and ii not in earliest_for_agent:
                earliest_for_agent[ii] = idx

        # Process steps in original order so earlier (causal) actions go first.
        for idx in sorted(remaining):
            i, act = steps[idx]
            if i == -1:
                consumed.append(idx)
                continue
            if combined[i] != Action.NoOp:
                continue
            if earliest_for_agent.get(i) != idx:
                continue
            if not state.is_applicable(i, act):
                continue

            # Causal safety for Push/Pull.
            if act.type is ActionType.Push:
                exp_r, exp_c = seq_pos[idx]
                box_dest = (exp_r + act.agent_row_delta + act.box_row_delta,
                            exp_c + act.agent_col_delta + act.box_col_delta)
                prot_idx = protected.get(box_dest, idx)
                if prot_idx != idx and prot_idx < idx:
                    continue
            elif act.type is ActionType.Pull:
                exp_r, exp_c = seq_pos[idx]
                box_dest = (exp_r, exp_c)
                prot_idx = protected.get(box_dest, idx)
                if prot_idx != idx and prot_idx < idx:
                    continue

            test = combined[:]
            test[i] = act
            if not state.is_conflicting(test):
                combined[i] = act
                consumed.append(idx)

        for idx in consumed:
            remaining.discard(idx)
        # Clear detour history for agents that made progress
        for idx in consumed:
            ii, _ = steps[idx]
            if ii >= 0:
                _last_detour.pop(ii, None)

        has_real_action = any(a != Action.NoOp for a in combined)

        if not has_real_action:
            if not remaining:
                break
            # Local deadlock recovery: force the globally-earliest applicable step.
            forced = None
            for idx in sorted(remaining):
                ii, aa = steps[idx]
                if ii < 0:
                    forced = idx
                    break
                if earliest_for_agent.get(ii) != idx:
                    continue
                if state.is_applicable(ii, aa):
                    forced = idx
                    break
            if forced is None:
                # True deadlock: all earliest steps are blocked.
                # Strategy: find what's BLOCKING each agent's step, and move THAT thing.
                # Moving the blocker frees the needed cell in one step.
                active_agents = set(earliest_for_agent.keys())
                _rev = {Action.MoveN: Action.MoveS, Action.MoveS: Action.MoveN,
                        Action.MoveE: Action.MoveW, Action.MoveW: Action.MoveE}
                _all_moves = [Action.MoveN, Action.MoveS, Action.MoveE, Action.MoveW]

                detour_agent = -1
                detour_act = None

                def _try_move_agent(bj: int) -> "tuple[int, object] | tuple[int, None]":
                    """Try to move agent bj to a good cell. Returns (bj, action) or (-1, None)."""
                    reverse_bj = _rev.get(_last_detour.get(bj))
                    best = None
                    # First pass: non-protected, non-reverse
                    for alt in _all_moves:
                        if alt == reverse_bj:
                            continue
                        test_det = [Action.NoOp] * num_agents
                        test_det[bj] = alt
                        if state.is_applicable(bj, alt) and not state.is_conflicting(test_det):
                            dest_r = state.agent_rows[bj] + alt.agent_row_delta
                            dest_c = state.agent_cols[bj] + alt.agent_col_delta
                            if (dest_r, dest_c) not in protected:
                                return bj, alt
                            if best is None:
                                best = alt
                    # Second pass: non-reverse, any destination
                    if best is not None:
                        return bj, best
                    # Third pass: allow reversal
                    for alt in _all_moves:
                        test_det = [Action.NoOp] * num_agents
                        test_det[bj] = alt
                        if state.is_applicable(bj, alt) and not state.is_conflicting(test_det):
                            return bj, alt
                    return -1, None

                # For each blocked active agent, identify what's blocking it.
                for idx in sorted(remaining):
                    ii, aa = steps[idx]
                    if ii < 0 or earliest_for_agent.get(ii) != idx:
                        continue
                    if state.is_applicable(ii, aa):
                        continue  # not actually blocked
                    # Find the needed (blocked) cell.
                    if aa.type is ActionType.Move:
                        need_r = state.agent_rows[ii] + aa.agent_row_delta
                        need_c = state.agent_cols[ii] + aa.agent_col_delta
                    elif aa.type is ActionType.Push:
                        need_r = state.agent_rows[ii] + aa.agent_row_delta + aa.box_row_delta
                        need_c = state.agent_cols[ii] + aa.agent_col_delta + aa.box_col_delta
                    elif aa.type is ActionType.Pull:
                        need_r = state.agent_rows[ii] + aa.agent_row_delta
                        need_c = state.agent_cols[ii] + aa.agent_col_delta
                    else:
                        continue
                    # Is any agent at that cell?
                    for bj in range(num_agents):
                        if bj == ii:
                            continue
                        if state.agent_rows[bj] == need_r and state.agent_cols[bj] == need_c:
                            bj_res, bj_act = _try_move_agent(bj)
                            if bj_res >= 0:
                                detour_agent = bj_res
                                detour_act = bj_act
                            break  # found the blocker (agent), stop looking
                    if detour_agent >= 0:
                        break

                if detour_agent >= 0:
                    if _detour_steps >= _max_detours:
                        print(f"[parallelize] detour limit reached remaining={len(remaining)}, returning original plan",
                              file=sys.stderr, flush=True)
                        return joint_plan
                    det = [Action.NoOp] * num_agents
                    det[detour_agent] = detour_act
                    new_plan.append(det)
                    state = state.result(det)
                    _last_detour[detour_agent] = detour_act
                    _detour_steps += 1
                    continue  # retry scheduling with cleared state

                # Last resort: try ANY applicable step from remaining,
                # ignoring order constraints (may fix box-blocking deadlocks).
                for idx in sorted(remaining):
                    ii, aa = steps[idx]
                    if ii < 0:
                        continue
                    if state.is_applicable(ii, aa):
                        lrec = [Action.NoOp] * num_agents
                        lrec[ii] = aa
                        if not state.is_conflicting(lrec):
                            new_plan.append(lrec)
                            state = state.result(lrec)
                            remaining.discard(idx)
                            _last_detour.pop(ii, None)
                            forced = -1  # signal: handled, skip normal forced
                            break
                if forced == -1:
                    continue

                # Cannot recover — exit main loop to try tail-sequential.
                print(f"[parallelize] deadlock remaining={len(remaining)}, trying tail",
                      file=sys.stderr, flush=True)
                break
            ii, aa = steps[forced]
            recovery = [Action.NoOp] * num_agents
            if ii >= 0:
                recovery[ii] = aa
            new_plan.append(recovery)
            state = state.result(recovery)
            remaining.discard(forced)
            continue

        new_plan.append(combined)
        state = state.result(combined)

    if remaining:
        # Try to complete the plan by navigating agents to their expected
        # sequential positions, then executing remaining steps.
        from collections import deque

        def _bfs_path(s, agent_idx, goal_r, goal_c):
            """BFS from agent_idx's current pos to (goal_r, goal_c). Treats
            walls and boxes as obstacles, ignores other agents.
            Returns list of Actions or None if unreachable."""
            sr = s.agent_rows[agent_idx]
            sc = s.agent_cols[agent_idx]
            if (sr, sc) == (goal_r, goal_c):
                return []
            moves = [(Action.MoveN, -1, 0), (Action.MoveS, 1, 0),
                     (Action.MoveE, 0, 1),  (Action.MoveW, 0, -1)]
            walls = s.walls
            boxes_set = {(r, c) for r, row_boxes in enumerate(s.boxes)
                         for c, bx in enumerate(row_boxes) if bx}
            visited = {(sr, sc): None}
            q = deque([(sr, sc)])
            while q:
                cr, cc = q.popleft()
                for act, dr, dc in moves:
                    nr, nc = cr + dr, cc + dc
                    if (nr, nc) in visited:
                        continue
                    if walls[nr][nc]:
                        continue
                    if (nr, nc) in boxes_set:
                        continue
                    visited[(nr, nc)] = ((cr, cc), act)
                    if (nr, nc) == (goal_r, goal_c):
                        # Reconstruct path
                        path = []
                        cur = (nr, nc)
                        while visited[cur] is not None:
                            parent, a = visited[cur]
                            path.append(a)
                            cur = parent
                        return list(reversed(path))
                    q.append((nr, nc))
            return None

        tail_plan = []
        tail_state = state
        tail_ok = True
        for idx in sorted(remaining):
            ii, aa = steps[idx]
            if ii < 0:
                row = [Action.NoOp] * num_agents
                tail_plan.append(row)
                tail_state = tail_state.result(row)
                continue
            er, ec = seq_pos[idx]
            if er >= 0 and (tail_state.agent_rows[ii], tail_state.agent_cols[ii]) != (er, ec):
                path = _bfs_path(tail_state, ii, er, ec)
                if path is None:
                    tail_ok = False
                    break
                for nav_act in path:
                    nav_row = [Action.NoOp] * num_agents
                    nav_row[ii] = nav_act
                    if not tail_state.is_applicable(ii, nav_act):
                        tail_ok = False
                        break
                    tail_plan.append(nav_row)
                    tail_state = tail_state.result(nav_row)
                if not tail_ok:
                    break
            row = [Action.NoOp] * num_agents
            row[ii] = aa
            if not tail_state.is_applicable(ii, aa):
                tail_ok = False
                break
            tail_plan.append(row)
            tail_state = tail_state.result(row)

        if tail_ok and tail_state.is_goal_state():
            combined_plan = new_plan + tail_plan
            if len(combined_plan) < len(joint_plan):
                print(f"[parallelize] {len(joint_plan)} \u2192 {len(combined_plan)} steps (with nav)",
                      file=sys.stderr, flush=True)
                return combined_plan

        print("[parallelize] incomplete, returning original plan",
              file=sys.stderr, flush=True)
        return joint_plan

    if not state.is_goal_state():
        print("[parallelize] goal not reached, returning original plan",
              file=sys.stderr, flush=True)
        return joint_plan

    print(f"[parallelize] {len(joint_plan)} \u2192 {len(new_plan)} steps",
          file=sys.stderr, flush=True)
    return new_plan

def decoupled_box_plan(initial_state: State, deadline: float) -> list[list[Action]] | None:
    """
    Decoupled multi-agent box planner with recursive blocker resolution.

    Each round tries every unsatisfied box goal in dependency order.
    When a task is blocked, the planner tries:
      1. Solve the blocker's own goal task first (recursive, depth ≤ 4)
      2. Park the blocker at its own goal (if safe) or a free cell
      3. Permissive path clearing (ignoring non-goal boxes to find a path,
         then systematically clearing each blocker)

    Returns a joint plan or None.
    """
    num_agents = len(initial_state.agent_rows)

    all_box_goals: list[tuple[str, int, int]] = [
        (State.goals[r][c], r, c)
        for r in range(len(State.goals))
        for c in range(len(State.goals[r]))
        if "A" <= State.goals[r][c] <= "Z"
    ]

    # Agent goal cells (digit goals) — prevent parking on them
    agent_goal_cells: frozenset[tuple[int, int]] = frozenset(
        (r, c)
        for r in range(len(State.goals))
        for c in range(len(State.goals[r]))
        if "0" <= State.goals[r][c] <= "9"
    )

    # Map box letter → list of its goal positions
    letter_goals: dict[str, list[tuple[int, int]]] = {}
    for letter, gr, gc in all_box_goals:
        letter_goals.setdefault(letter, []).append((gr, gc))

    joint_plan: list[list[Action]] = []
    state = initial_state
    TASK_BUDGET = 4.0

    # ── Shared helpers (closures over state/joint_plan) ───────────────────────

    def _build_extra_walls(agent_idx: int, box_r: int, box_c: int) -> frozenset:
        return frozenset(
            (r, c)
            for r in range(len(state.boxes))
            for c in range(len(state.boxes[r]))
            if state.boxes[r][c] and not (r == box_r and c == box_c)
        ) | frozenset(
            (state.agent_rows[j], state.agent_cols[j])
            for j in range(num_agents) if j != agent_idx
        )

    def _execute_actions(agent_idx: int, acts: list[Action]) -> bool:
        """Execute single-agent actions in joint-NoOp format. Returns True on success."""
        nonlocal state
        for act in acts:
            ja = [Action.NoOp] * num_agents
            ja[agent_idx] = act
            if not state.is_applicable(agent_idx, act) or state.is_conflicting(ja):
                print(f"[decoupled] Simulation mismatch agent {agent_idx}", file=sys.stderr, flush=True)
                return False
            joint_plan.append(ja)
            state = state.result(ja)
        return True

    def _move_agent_away(j: int, avoid_cells: set) -> bool:
        """
        Move agent j to the nearest free cell NOT in avoid_cells.
        Returns True if the agent was successfully moved.
        """
        ar_j, ac_j = state.agent_rows[j], state.agent_cols[j]
        if (ar_j, ac_j) not in avoid_cells:
            return True  # already outside avoid zone
        # Build walls for this agent: all boxes + all other agents
        ex_j: frozenset = frozenset(
            (r, c)
            for r in range(len(state.boxes))
            for c in range(len(state.boxes[r]))
            if state.boxes[r][c]
        ) | frozenset(
            (state.agent_rows[k], state.agent_cols[k])
            for k in range(num_agents) if k != j
        )
        rows = len(State.walls)
        cols = len(State.walls[0])
        # BFS from agent's position to find nearest reachable cell not in avoid_cells.
        # We MUST expand through avoid_cells (they're traversable), just not target them.
        from collections import deque as _deque
        visited: set = {(ar_j, ac_j)}
        q = _deque([(ar_j, ac_j)])
        candidates: list[tuple[int, int]] = []
        while q:
            r, c = q.popleft()
            if (r, c) not in avoid_cells and (r, c) != (ar_j, ac_j):
                candidates.append((r, c))
                if len(candidates) >= 5:
                    break  # have enough options
            for dr, dc in _DIRS:
                nr, nc = r + dr, c + dc
                if (0 <= nr < rows and 0 <= nc < cols
                        and not State.walls[nr][nc]
                        and (nr, nc) not in ex_j
                        and (nr, nc) not in visited):
                    visited.add((nr, nc))
                    q.append((nr, nc))
        # Try each candidate until one works
        for tr, tc in candidates:
            acts = _plan_agent_move(ar_j, ac_j, tr, tc, ex_j, time.perf_counter() + 3.0)
            if acts is not None and _execute_actions(j, acts):
                return True
        return False

    def _clear_agents_from_path(agent_idx: int, br: int, bc: int, gr: int, gc: int) -> None:
        """
        Move any other agent that is sitting on the goal cell or an adjacent
        push-approach cell to a nearby free cell.  This prevents other agents
        from blocking _plan_task's BFS-from-goal or the final push step.
        """
        rows = len(State.walls)
        cols = len(State.walls[0])
        # Only need to clear the goal cell and cells directly adjacent to it
        # (the 4 positions where the pushing agent must stand to push into goal).
        # Also include the ideal-path cells because those are where the box
        # must pass through and the agent must push from.
        path = _trace_ideal_path(br, bc, gr, gc)
        # Critical cells: goal + adjacent to goal.  Path cells are nice-to-have
        # but the BFS-from-goal only fails if (gr,gc) itself is in extra_walls.
        critical: set = {(gr, gc)}
        for dr, dc in _DIRS:
            nr, nc = gr + dr, gc + dc
            if 0 <= nr < rows and 0 <= nc < cols and not State.walls[nr][nc]:
                critical.add((nr, nc))
        # Add a few intermediate path cells so agents don't block the box mid-route
        for pr, pc in path[1:-1]:
            critical.add((pr, pc))
        # Also include cells adjacent to the box itself — these are push-approach
        # positions that the pushing agent must be able to occupy.  Placing another
        # agent there traps the pushing agent and causes A* to fail.
        for dr, dc in _DIRS:
            nr, nc = br + dr, bc + dc
            if 0 <= nr < rows and 0 <= nc < cols and not State.walls[nr][nc]:
                critical.add((nr, nc))

        rows_cap = len(State.walls)
        cols_cap = len(State.walls[0])
        for j in range(num_agents):
            if j == agent_idx:
                continue
            ar_j, ac_j = state.agent_rows[j], state.agent_cols[j]
            if (ar_j, ac_j) not in critical:
                continue
            # Try moving agent j to any free cell not in the critical zone.
            # Use a progressively smaller avoid set if needed.
            moved = False
            for avoid in [critical, {(gr, gc)}]:
                if _move_agent_away(j, avoid):
                    moved = True
                    break
            if not moved:
                # Agent j is trapped — check if a satisfied goal-box is blocking
                # its only exits and temporarily displace it to free the agent.
                for ddr, ddc in _DIRS:
                    adj_r, adj_c = ar_j + ddr, ac_j + ddc
                    if not (0 <= adj_r < rows_cap and 0 <= adj_c < cols_cap):
                        continue
                    adj_box = state.boxes[adj_r][adj_c]
                    if not adj_box:
                        continue
                    if not (adj_r < len(State.goals) and adj_c < len(State.goals[adj_r])
                            and State.goals[adj_r][adj_c] == adj_box):
                        continue
                    # Displace the goal-box so agent j can escape
                    gb_forbidden = set(critical) | {(ar_j, ac_j)}
                    if _resolve_blocker(adj_r, adj_c, adj_box, gb_forbidden, depth=0):
                        for avoid2 in [critical, {(gr, gc)}]:
                            if _move_agent_away(j, avoid2):
                                break
                        break  # one displacement attempt per agent

    def _try_direct_task(agent_idx: int, br: int, bc: int, gr: int, gc: int, letter: str) -> bool:
        """
        Plan and execute pushing `letter` from (br,bc) to (gr,gc) with agent_idx.
        Returns True on success.
        """
        # STALE-BOX GUARD: box must still be at (br,bc)
        if state.boxes[br][bc] != letter:
            return False
        # Already satisfied?
        if state.boxes[gr][gc] == letter:
            return True
        # Pre-clear: move any other agent off the ideal path and the goal cell
        _clear_agents_from_path(agent_idx, br, bc, gr, gc)
        ar, ac = state.agent_rows[agent_idx], state.agent_cols[agent_idx]
        extra = _build_extra_walls(agent_idx, br, bc)
        acts = _plan_task(ar, ac, br, bc, gr, gc, extra, time.perf_counter() + TASK_BUDGET)
        if acts is None:
            return False
        print(f"[decoupled] Agent {agent_idx}: push {letter} ({br},{bc})→({gr},{gc})",
              file=sys.stderr, flush=True)
        return _execute_actions(agent_idx, acts)

    def _resolve_blocker(b_r: int, b_c: int, b_letter: str,
                         forbidden_cells: set, depth: int = 0) -> bool:
        """
        Try to clear the blocker at (b_r, b_c, b_letter).
        Strategy (in order):
          1. Push blocker to one of its own unsatisfied goal positions
          2. Recursively clear the sub-blocker that blocks strategy 1
          3. Park blocker at a free non-forbidden cell (allowing own goal)
        Returns True if progress was made (state changed).
        """
        if depth > 4:
            return False
        if time.perf_counter() > deadline - 2.0:
            return False
        if state.boxes[b_r][b_c] != b_letter:
            return True  # already moved

        # A box at its own goal normally shouldn't be moved (avoid restore-loops).
        # However at depth=0 we allow one-time temporary displacement so that
        # another task can pass through, after which this box re-appears in
        # unsatisfied goals and is re-placed in the next round.
        if (b_r < len(State.goals) and b_c < len(State.goals[b_r])
                and State.goals[b_r][b_c] == b_letter):
            if depth > 0:
                return False  # never recursively displace a goal-box
            displaced_key = (b_letter, b_r, b_c)
            if goal_displaced_count.get(displaced_key, 0) >= 4:
                return False  # already displaced too many times
            # Extend the forbidden zone to include cells immediately adjacent to
            # the goal-box's current position — parking it one step away would
            # still block the agent routing needed for the main task.
            rows_g = len(State.walls)
            cols_g = len(State.walls[0])
            extended_forbidden = set(forbidden_cells)
            for ddr, ddc in _DIRS:
                nr_g, nc_g = b_r + ddr, b_c + ddc
                if 0 <= nr_g < rows_g and 0 <= nc_g < cols_g:
                    extended_forbidden.add((nr_g, nc_g))
            park_result = _plan_park(
                state, b_r, b_c, b_letter, extended_forbidden, num_agents, deadline,
                blocker_own_goal=None,
            )
            if park_result is not None:
                park_agent, park_acts, park_pos = park_result
                if _execute_actions(park_agent, park_acts):
                    goal_displaced_count[displaced_key] = (
                        goal_displaced_count.get(displaced_key, 0) + 1
                    )
                    print(
                        f"[decoupled] Temp-displaced goal-box {b_letter}"
                        f" ({b_r},{b_c})→{park_pos}",
                        file=sys.stderr, flush=True,
                    )
                    return True
            return False

        b_color = State.box_colors[ord(b_letter) - ord("A")]
        b_compat = [k for k in range(num_agents) if State.agent_colors[k] == b_color]
        if not b_compat:
            return False

        # Strategy 1: push blocker to one of its own unsatisfied goals
        # Skip goals that are in forbidden_cells (those cells must stay clear for the main task)
        blocker_goal_options = [
            (bgr, bgc)
            for bgr, bgc in letter_goals.get(b_letter, [])
            if state.boxes[bgr][bgc] != b_letter
            and (bgr, bgc) not in forbidden_cells
        ]
        # Also keep forbidden goals as fallback options (tried last via park)
        all_blocker_goals = [
            (bgr, bgc)
            for bgr, bgc in letter_goals.get(b_letter, [])
            if state.boxes[bgr][bgc] != b_letter
        ]
        blocker_goal_options.sort(key=lambda g: abs(g[0] - b_r) + abs(g[1] - b_c))

        for bgr, bgc in blocker_goal_options:
            for b_agent in sorted(b_compat,
                                   key=lambda k: abs(state.agent_rows[k] - b_r) + abs(state.agent_cols[k] - b_c)):
                if time.perf_counter() > deadline - 2.0:
                    return False
                if _try_direct_task(b_agent, b_r, b_c, bgr, bgc, b_letter):
                    return True

            # Strategy 2: the blocker's own goal path is itself blocked — recurse
            if depth < 3:
                sub_blocker = _first_blocker_on_path(b_r, b_c, bgr, bgc, state)
                if sub_blocker and (sub_blocker[0], sub_blocker[1]) != (b_r, b_c):
                    sb_r, sb_c, sb_letter = sub_blocker
                    if _resolve_blocker(sb_r, sb_c, sb_letter, forbidden_cells, depth + 1):
                        # Retry after clearing sub-blocker
                        for bgr2, bgc2 in blocker_goal_options:
                            for b_agent2 in sorted(b_compat,
                                                    key=lambda k: abs(state.agent_rows[k] - b_r) + abs(state.agent_cols[k] - b_c)):
                                if _try_direct_task(b_agent2, b_r, b_c, bgr2, bgc2, b_letter):
                                    return True

        # Strategy 3: park blocker at a non-forbidden free cell (allow own goal only if not forbidden)
        blocker_own_goal = (blocker_goal_options[0] if blocker_goal_options
                            else all_blocker_goals[0] if all_blocker_goals else None)
        # Don't allow own goal if it's in the forbidden zone (e.g., on main task's path)
        own_goal_for_park = blocker_own_goal if (blocker_own_goal and blocker_own_goal not in forbidden_cells) else None
        park_result = _plan_park(
            state, b_r, b_c, b_letter, forbidden_cells, num_agents, deadline,
            blocker_own_goal=own_goal_for_park,
        )
        if park_result is not None:
            park_agent, park_acts, park_pos = park_result
            if _execute_actions(park_agent, park_acts):
                print(f"[decoupled] Parked {b_letter} ({b_r},{b_c})→{park_pos}",
                      file=sys.stderr, flush=True)
                return True

        return False

    def _clear_obstacles_to_box(agent_idx: int, box_r: int, box_c: int,
                                main_goal: tuple | None = None) -> bool:
        """
        When the planning agent can't reach the target box (path blocked by agents or
        no-goal boxes), find and clear the FIRST obstacle on the theoretical (walls-only)
        path from agent to box. Returns True if any obstacle was cleared.
        main_goal: (gr, gc) goal cell for the main box being pushed — used to forbid
        intermediate push-path cells so displaced boxes don't land on them.
        """
        if time.perf_counter() > deadline - 2.0:
            return False
        ar, ac = state.agent_rows[agent_idx], state.agent_cols[agent_idx]
        rows_l, cols_l = len(State.walls), len(State.walls[0])

        # BFS with walls only (ignore extra_walls) to find shortest path to box vicinity
        parent: dict = {(ar, ac): None}
        bfs_q: deque = deque([(ar, ac)])
        found_adj: tuple | None = None
        while bfs_q and found_adj is None:
            r, c = bfs_q.popleft()
            for dr, dc in _DIRS:
                nr, nc = r + dr, c + dc
                if (nr, nc) in parent:
                    continue
                if not (0 <= nr < rows_l and 0 <= nc < cols_l):
                    continue
                if State.walls[nr][nc]:
                    continue
                if nr == box_r and nc == box_c:
                    continue  # can't enter box cell itself
                parent[(nr, nc)] = (r, c)
                # Check if this cell is adjacent to box
                for bdr, bdc in _DIRS:
                    if nr + bdr == box_r and nc + bdc == box_c:
                        found_adj = (nr, nc)
                        break
                if found_adj:
                    break
                bfs_q.append((nr, nc))

        if found_adj is None:
            return False  # truly unreachable even ignoring extra_walls

        # Reconstruct path from agent to found_adj
        path_cells: list = []
        cur = found_adj
        while cur is not None:
            path_cells.append(cur)
            cur = parent[cur]
        path_cells.reverse()
        path_set: set = set(path_cells)

        extra = _build_extra_walls(agent_idx, box_r, box_c)

        # Find and clear first obstacle on path
        for pr, pc in path_cells:
            if (pr, pc) not in extra:
                continue
            # Check if it's another agent
            cleared = False
            for j in range(num_agents):
                if j != agent_idx and state.agent_rows[j] == pr and state.agent_cols[j] == pc:
                    if _move_agent_away(j, path_set):
                        cleared = True
                    break
            if cleared:
                return True
            # Check if it's a box (no-goal or with goal)
            bl = state.boxes[pr][pc] if state.boxes[pr][pc] else None
            if bl:
                bl_goal_opts = [
                    (bgr, bgc) for bgr, bgc in letter_goals.get(bl, [])
                    if state.boxes[bgr][bgc] != bl
                ]
                bl_own_goal = bl_goal_opts[0] if bl_goal_opts else None
                # Check if this box is at its own satisfied goal position
                _bl_at_goal = (
                    bl and pr < len(State.goals) and pc < len(State.goals[pr])
                    and State.goals[pr][pc] == bl
                )
                # For a box at its satisfied goal, forbid ALL box goal cells to
                # prevent it from being parked at another box's goal (causing cycles).
                # For non-goal boxes, only forbid already-satisfied goals.
                park_forbidden: set = (
                    {(fg_r, fg_c) for fg_let, fg_r, fg_c in all_box_goals}
                    if _bl_at_goal else
                    {(fg_r, fg_c) for fg_let, fg_r, fg_c in all_box_goals
                     if state.boxes[fg_r][fg_c] == fg_let}
                ) | agent_goal_cells | {(box_r, box_c)} | path_set
                # Also forbid intermediate push-path cells of the main box so a
                # displaced obstacle doesn't land on a required push step.
                if main_goal is not None:
                    push_path = _trace_ideal_path(box_r, box_c, main_goal[0], main_goal[1])
                    park_forbidden |= set(push_path)
                park_result = _plan_park(
                    state, pr, pc, bl, park_forbidden, num_agents, deadline,
                    blocker_own_goal=bl_own_goal,
                )
                if park_result:
                    park_agent, park_acts, park_pos = park_result
                    if _execute_actions(park_agent, park_acts):
                        print(f"[decoupled] Cleared agent-path obstacle {bl}({pr},{pc})→{park_pos}",
                              file=sys.stderr, flush=True)
                        return True
        return False

    def _permissive_clear(letter: str, br: int, bc: int, gr: int, gc: int) -> bool:
        """
        Last-resort: plan path ignoring all non-goal boxes, then clear each
        blocker (box OR agent) on the theoretical path one by one.
        Returns True if ANY progress was made.
        """
        path = _trace_ideal_path(br, bc, gr, gc)
        if not path:
            return False
        path_set = frozenset(path)
        # Also include cells adjacent to the goal (push approach positions)
        rows = len(State.walls)
        cols = len(State.walls[0])
        approach_cells: set = set()
        for dr, dc in _DIRS:
            nr, nc = gr + dr, gc + dc
            if 0 <= nr < rows and 0 <= nc < cols and not State.walls[nr][nc]:
                approach_cells.add((nr, nc))
        full_clear_set = path_set | approach_cells | {(gr, gc)}

        made_progress = False
        for pr, pc in path[1:]:
            if time.perf_counter() > deadline - 2.0:
                break
            bl = state.boxes[pr][pc]
            if bl:
                # If a goal-box is blocking the path, try to temporarily displace it
                if (pr < len(State.goals) and pc < len(State.goals[pr])
                        and State.goals[pr][pc] == bl):
                    _goal_forbidden: set = (
                        {(fg_r, fg_c) for fg_let, fg_r, fg_c in all_box_goals
                         if state.boxes[fg_r][fg_c] == fg_let}
                        | agent_goal_cells
                        | {(br, bc), (gr, gc)}
                        | set(path_set)
                    )
                    if _resolve_blocker(pr, pc, bl, _goal_forbidden, depth=0):
                        made_progress = True
                        break
                    continue  # can't displace, skip and try next cell
                # Skip unmovable boxes (no compatible agent) — A* will route around them
                bl_color = State.box_colors[ord(bl) - ord("A")]
                bl_agents = [k for k in range(num_agents) if State.agent_colors[k] == bl_color]
                if not bl_agents:
                    continue
                # Park this on-path box blocker somewhere off the path
                # Only forbid satisfied goals (placed boxes) + path + endpoints
                forbidden_for_park: set = (
                    {(fg_r, fg_c) for fg_let, fg_r, fg_c in all_box_goals
                     if state.boxes[fg_r][fg_c] == fg_let}
                    | agent_goal_cells
                    | {(br, bc), (gr, gc)}
                    | set(path_set)
                )
                bl_goal_opts = [
                    (bgr, bgc) for bgr, bgc in letter_goals.get(bl, [])
                    if state.boxes[bgr][bgc] != bl
                ]
                bl_own_goal = (min(bl_goal_opts, key=lambda g: abs(g[0] - pr) + abs(g[1] - pc))
                               if bl_goal_opts else None)
                park_result = _plan_park(
                    state, pr, pc, bl, forbidden_for_park, num_agents, deadline,
                    main_task_box=(br, bc), main_task_goal=(gr, gc),
                    blocker_own_goal=bl_own_goal,
                )
                if park_result is not None:
                    park_agent, park_acts, _ = park_result
                    if _execute_actions(park_agent, park_acts):
                        made_progress = True
                        break  # restart outer round
            else:
                # Check if an agent is blocking this path cell
                for j in range(num_agents):
                    if state.agent_rows[j] == pr and state.agent_cols[j] == pc:
                        # Try progressively relaxed avoid sets
                        moved = False
                        for avoid_set in [full_clear_set, path_set, {(gr, gc)}]:
                            if _move_agent_away(j, avoid_set):
                                moved = True
                                break
                        if not moved:
                            # Agent trapped — try displacing adjacent goal-boxes
                            rows_pc = len(State.walls)
                            cols_pc = len(State.walls[0])
                            for ddr2, ddc2 in _DIRS:
                                tr2, tc2 = pr + ddr2, pc + ddc2
                                if not (0 <= tr2 < rows_pc and 0 <= tc2 < cols_pc):
                                    continue
                                gb2 = state.boxes[tr2][tc2]
                                if (gb2 and tr2 < len(State.goals)
                                        and tc2 < len(State.goals[tr2])
                                        and State.goals[tr2][tc2] == gb2):
                                    if _resolve_blocker(tr2, tc2, gb2,
                                                        set(path_set) | {(pr, pc)},
                                                        depth=0):
                                        _move_agent_away(j, path_set)
                                        break
                        if state.agent_rows[j] != pr or state.agent_cols[j] != pc:
                            made_progress = True
                        break
        return made_progress

    # ── Box-goal phase ────────────────────────────────────────────────────────
    max_rounds = len(all_box_goals) * 8 + 20
    goal_displaced_count: dict = {}   # tracks temporary displacements of goal-boxes
    seen_box_configs: set = set()     # cycle detection
    _cycle_resets = 0                 # how many times we've reset after a cycle
    _MAX_CYCLE_RESETS = 6

    # Pre-compute walls-only BFS distances from each unique goal position.
    # Used to (a) filter impossible box-goal assignments and (b) improve cost sorting.
    _goal_bfs_cache: dict[tuple[int, int], list[list[int]]] = {}
    for _letter, _gr, _gc in all_box_goals:
        if (_gr, _gc) not in _goal_bfs_cache:
            _goal_bfs_cache[(_gr, _gc)] = _bfs_from(_gr, _gc, frozenset())
    for _round in range(max_rounds):
        if time.perf_counter() > deadline:
            return None

        unsatisfied = [
            (letter, gr, gc)
            for letter, gr, gc in all_box_goals
            if state.boxes[gr][gc] != letter
        ]
        if not unsatisfied:
            break

        # Cycle detection: if we've seen the exact same configuration of
        # UNSATISFIED boxes before (with the same unsatisfied goal count),
        # we're stuck in a loop → abort early.
        # Only track unsatisfied boxes to avoid false cycles when satisfied
        # boxes remain in place while others cycle.
        satisfied_goals_set = frozenset(  # noqa: F841 (used for conceptual clarity)
            (fg_let, fg_r, fg_c) for fg_let, fg_r, fg_c in all_box_goals
            if state.boxes[fg_r][fg_c] == fg_let
        )
        unsat_box_config = frozenset(
            (r, c, state.boxes[r][c])
            for r in range(len(state.boxes))
            for c in range(len(state.boxes[r]))
            if state.boxes[r][c]
            and not (r < len(State.goals) and c < len(State.goals[r])
                     and State.goals[r][c] == state.boxes[r][c])
        )
        cycle_key = (len(unsatisfied), unsat_box_config)
        if cycle_key in seen_box_configs:
            if _cycle_resets >= _MAX_CYCLE_RESETS:
                print("[decoupled] Cycle detected — aborting", file=sys.stderr, flush=True)
                return None
            # Try to break the cycle by resetting detection and changing goal order
            import random as _random
            _cycle_resets += 1
            seen_box_configs.clear()
            goal_displaced_count.clear()
            print(f"[decoupled] Cycle detected — reset #{_cycle_resets}, changing order",
                  file=sys.stderr, flush=True)
            # Alternate between reversed order and random shuffle each reset
            if _cycle_resets % 2 == 1:
                all_box_goals.reverse()
            else:
                _random.shuffle(all_box_goals)
        seen_box_configs.add(cycle_key)
        if time.perf_counter() > deadline:
            return None

        unsatisfied = [
            (letter, gr, gc)
            for letter, gr, gc in all_box_goals
            if state.boxes[gr][gc] != letter
        ]
        if not unsatisfied:
            break

        # Check no compatible agents exist
        for letter, gr, gc in unsatisfied:
            box_color = State.box_colors[ord(letter) - ord("A")]
            compatible = [i for i in range(num_agents) if State.agent_colors[i] == box_color]
            if not compatible:
                print(f"[decoupled] No compatible agent for box {letter}", file=sys.stderr, flush=True)
                return None

        # Dependency-aware ordering: tasks blocking others come first.
        # After a cycle reset, skip the dep sort on odd resets so the
        # reversed/shuffled order from all_box_goals actually takes effect.
        if _cycle_resets % 2 == 1:
            ordered_goals = list(unsatisfied)  # already reflects reversed/shuffled all_box_goals
        else:
            ordered_goals = _dependency_sort_goals(unsatisfied, state)

        # Build candidate map
        from collections import defaultdict
        by_goal: dict[tuple[str, int, int], list[tuple]] = defaultdict(list)
        for letter, gr, gc in unsatisfied:
            box_color = State.box_colors[ord(letter) - ord("A")]
            compatible = [i for i in range(num_agents) if State.agent_colors[i] == box_color]
            avail = [
                (r, c)
                for r in range(len(state.boxes))
                for c in range(len(state.boxes[r]))
                if state.boxes[r][c] == letter and not (r == gr and c == gc)
                and not (r < len(State.goals) and c < len(State.goals[r]) and State.goals[r][c] == letter)
            ]
            for i in compatible:
                ar, ac = state.agent_rows[i], state.agent_cols[i]
                for br, bc in avail:
                    # Skip box-goal pairs that are impossible even without other boxes
                    walls_only_dist = _goal_bfs_cache.get((gr, gc))
                    if walls_only_dist and walls_only_dist[br][bc] == _INF:
                        continue
                    box_to_goal = (walls_only_dist[br][bc] if walls_only_dist and walls_only_dist[br][bc] != _INF
                                   else abs(br - gr) + abs(bc - gc))
                    cost = abs(ar - br) + abs(ac - bc) + box_to_goal
                    by_goal[(letter, gr, gc)].append((cost, i, br, bc))
            by_goal[(letter, gr, gc)].sort()

        progress = 0
        solved_goals: set[tuple[str, int, int]] = set()

        # Pre-compute agent goals list (used in agent-goal phase)
        _agent_goals_all: list[tuple[int, int, int]] = [
            (int(State.goals[r][c]), r, c)
            for r in range(len(State.goals))
            for c in range(len(State.goals[r]))
            if "0" <= State.goals[r][c] <= "9"
            and int(State.goals[r][c]) < num_agents
        ]

        for goal_key in ordered_goals:
            if goal_key in solved_goals:
                continue
            if time.perf_counter() > deadline - 2.0:
                return None

            letter, gr, gc = goal_key
            # Re-check if already satisfied during this round
            if state.boxes[gr][gc] == letter:
                progress += 1
                solved_goals.add(goal_key)
                continue

            # ── Pre-placement agent rescue ─────────────────────────────────
            # If placing box letter at (gr,gc) would permanently block any
            # agent from reaching its own goal, navigate that agent to its
            # goal NOW while the path is still open.
            if _agent_goals_all and time.perf_counter() < deadline - 3.0:
                _cur_all_boxes: frozenset = frozenset(
                    (r, c)
                    for r in range(len(state.boxes))
                    for c in range(len(state.boxes[r]))
                    if state.boxes[r][c]
                )
                for _ag_idx, _ag_gr, _ag_gc in _agent_goals_all:
                    _ar2, _ac2 = state.agent_rows[_ag_idx], state.agent_cols[_ag_idx]
                    if _ar2 == _ag_gr and _ac2 == _ag_gc:
                        continue  # already at goal
                    _ag_other = frozenset(
                        (state.agent_rows[k], state.agent_cols[k])
                        for k in range(num_agents) if k != _ag_idx
                    )
                    _ag_cur_extra = _cur_all_boxes | _ag_other
                    # Quick walls-only check first
                    _walls_dist = _bfs_ignore_boxes(_ag_gr, _ag_gc)
                    if _walls_dist[_ar2][_ac2] == _INF:
                        continue  # goal structurally unreachable, skip
                    # Check: can agent reach goal NOW (before box placement)?
                    if _bfs_from(_ag_gr, _ag_gc, _ag_cur_extra)[_ar2][_ac2] == _INF:
                        continue  # already unreachable — can't help
                    # Check: would placing box at (gr,gc) block it?
                    _ag_extra_new = _ag_cur_extra | frozenset({(gr, gc)})
                    if _bfs_from(_ag_gr, _ag_gc, _ag_extra_new)[_ar2][_ac2] != _INF:
                        continue  # still reachable after placement, no problem
                    # Placing this box would trap the agent — navigate it first
                    _pre_acts = _plan_agent_move(
                        _ar2, _ac2, _ag_gr, _ag_gc, _ag_cur_extra,
                        time.perf_counter() + 5.0,
                    )
                    if _pre_acts is not None:
                        _rescued = True
                        for _act in _pre_acts:
                            _ja = [Action.NoOp] * num_agents
                            _ja[_ag_idx] = _act
                            if (state.is_applicable(_ag_idx, _act)
                                    and not state.is_conflicting(_ja)):
                                joint_plan.append(_ja)
                                state = state.result(_ja)
                            else:
                                _rescued = False
                                break
                        if _rescued:
                            print(
                                f"[decoupled] Pre-rescued agent {_ag_idx}"
                                f" → ({_ag_gr},{_ag_gc})"
                                f" before {letter}@({gr},{gc})",
                                file=sys.stderr, flush=True,
                            )

            goal_cands = by_goal.get(goal_key, [])
            succeeded = False
            first_blocker = None

            # Try top-5 (agent, box) candidates
            for cost, i, br, bc in goal_cands[:5]:
                if time.perf_counter() > deadline - 2.0:
                    return None

                # STALE-BOX GUARD
                if state.boxes[br][bc] != letter:
                    continue

                if _try_direct_task(i, br, bc, gr, gc, letter):
                    progress += 1
                    solved_goals.add(goal_key)
                    succeeded = True
                    break

                if first_blocker is None:
                    first_blocker = _first_blocker_on_path(br, bc, gr, gc, state)

            if succeeded:
                continue

            # Direct planning failed — try blocker resolution
            if first_blocker is not None and time.perf_counter() < deadline - 2.0:
                b_r, b_c, b_letter = first_blocker
                # Forbidden: only SATISFIED goals (placed boxes) + main task's path
                # Unsatisfied goal cells are allowed for temporary parking
                main_path_cells: set = set()
                if goal_cands:
                    main_br, main_bc = goal_cands[0][2], goal_cands[0][3]
                    main_path_cells = set(_trace_ideal_path(main_br, main_bc, gr, gc))
                forbidden: set = (
                    {(fg_r, fg_c) for fg_let, fg_r, fg_c in all_box_goals
                     if state.boxes[fg_r][fg_c] == fg_let}
                    | agent_goal_cells
                    | {(gr, gc)}
                    | main_path_cells
                )
                if goal_cands:
                    forbidden.add((goal_cands[0][2], goal_cands[0][3]))

                print(
                    f"[decoupled] Task {letter}({goal_cands[0][2] if goal_cands else '?'},"
                    f"{goal_cands[0][3] if goal_cands else '?'})→({gr},{gc})"
                    f" blocked by {first_blocker}; resolving...",
                    file=sys.stderr, flush=True,
                )
                if _resolve_blocker(b_r, b_c, b_letter, forbidden, depth=0):
                    # Blocker moved — always counts as progress (prevents false "No progress")
                    progress += 1
                    # After clearing a blocker, rescue any agent whose path to its goal
                    # passes through (gr,gc) — the blocker clearance may have opened a
                    # corridor that (gr,gc) is about to close permanently.
                    if _agent_goals_all and time.perf_counter() < deadline - 3.0:
                        _post_boxes: frozenset = frozenset(
                            (r, c)
                            for r in range(len(state.boxes))
                            for c in range(len(state.boxes[r]))
                            if state.boxes[r][c]
                        )
                        for _ag_idx2, _ag_gr2, _ag_gc2 in _agent_goals_all:
                            _ar3 = state.agent_rows[_ag_idx2]
                            _ac3 = state.agent_cols[_ag_idx2]
                            if _ar3 == _ag_gr2 and _ac3 == _ag_gc2:
                                continue
                            _ag_other2 = frozenset(
                                (state.agent_rows[k], state.agent_cols[k])
                                for k in range(num_agents) if k != _ag_idx2
                            )
                            _post_extra2 = _post_boxes | _ag_other2
                            if _bfs_from(_ag_gr2, _ag_gc2, _post_extra2)[_ar3][_ac3] == _INF:
                                continue  # still unreachable after clear
                            _post_with_new2 = _post_extra2 | frozenset({(gr, gc)})
                            if _bfs_from(_ag_gr2, _ag_gc2, _post_with_new2)[_ar3][_ac3] != _INF:
                                continue  # placing box won't block this agent
                            _rescue_acts = _plan_agent_move(
                                _ar3, _ac3, _ag_gr2, _ag_gc2, _post_extra2,
                                time.perf_counter() + 5.0,
                            )
                            if _rescue_acts is not None:
                                _ok = True
                                for _ract in _rescue_acts:
                                    _rja = [Action.NoOp] * num_agents
                                    _rja[_ag_idx2] = _ract
                                    if (state.is_applicable(_ag_idx2, _ract)
                                            and not state.is_conflicting(_rja)):
                                        joint_plan.append(_rja)
                                        state = state.result(_rja)
                                    else:
                                        _ok = False
                                        break
                                if _ok:
                                    print(
                                        f"[decoupled] Post-resolve rescued agent {_ag_idx2}"
                                        f" → ({_ag_gr2},{_ag_gc2}) before {letter}@({gr},{gc})",
                                        file=sys.stderr, flush=True,
                                    )
                    # Iterative retry: keep clearing blockers until main task succeeds
                    # or we run out of clearable blockers (handles multi-level blocking chains)
                    last_blocker = (b_r, b_c, b_letter)
                    for _clearing_attempt in range(7):
                        solved_in_retry = False
                        for cost, i, br, bc in goal_cands[:5]:
                            if state.boxes[br][bc] != letter:
                                continue
                            if _try_direct_task(i, br, bc, gr, gc, letter):
                                solved_goals.add(goal_key)
                                solved_in_retry = True
                                break
                        if solved_in_retry:
                            break
                        # Still failing — find the *new* first blocker and clear it
                        if not goal_cands or time.perf_counter() > deadline - 2.0:
                            break
                        next_br, next_bc = goal_cands[0][2], goal_cands[0][3]
                        if state.boxes[next_br][next_bc] != letter:
                            break
                        next_blocker = _first_blocker_on_path(next_br, next_bc, gr, gc, state)
                        if (next_blocker is None
                                or (next_blocker[0], next_blocker[1], next_blocker[2]) == last_blocker):
                            break
                        nb_r, nb_c, nb_letter = next_blocker
                        if _resolve_blocker(nb_r, nb_c, nb_letter, forbidden, depth=0):
                            progress += 1
                            last_blocker = (nb_r, nb_c, nb_letter)
                        else:
                            break

            # Check if the AGENT can't reach the box
            # (e.g., another agent or no-goal box is blocking agent's corridor)
            # Also call this when resolve_blocker failed, not just when first_blocker is None
            if not succeeded and time.perf_counter() < deadline - 2.0:
                for cost, i, br, bc in goal_cands[:3]:
                    if state.boxes[br][bc] != letter:
                        continue
                    if _clear_obstacles_to_box(i, br, bc, main_goal=(gr, gc)):
                        progress += 1
                        break

            # If plan_task failed with no box-blocker found, try clearing agents
            # that are sitting on the BOX's path to its goal (not just agent approach).
            # Handles cases where a pushing agent ends up stranded on a later task's path.
            if not succeeded and first_blocker is None and time.perf_counter() < deadline - 2.0:
                for cost, i, br, bc in goal_cands[:3]:
                    if state.boxes[br][bc] != letter:
                        continue
                    path_to_goal = _trace_ideal_path(br, bc, gr, gc)
                    path_cells_set = set(path_to_goal[1:])
                    cleared_any = False
                    for pr2, pc2 in path_to_goal[1:]:
                        if time.perf_counter() > deadline - 2.0:
                            break
                        for j in range(num_agents):
                            if (state.agent_rows[j] == pr2 and state.agent_cols[j] == pc2):
                                moved = False
                                for avoid_s in [path_cells_set, {(gr, gc)}]:
                                    if _move_agent_away(j, avoid_s):
                                        moved = True
                                        break
                                if not moved:
                                    # Trapped — try displacing adjacent satisfied goal-box
                                    for ddr3, ddc3 in _DIRS:
                                        nr3 = pr2 + ddr3
                                        nc3 = pc2 + ddc3
                                        if not (0 <= nr3 < len(state.boxes)
                                                and 0 <= nc3 < len(state.boxes[nr3])):
                                            continue
                                        gb3 = state.boxes[nr3][nc3]
                                        if (gb3 and nr3 < len(State.goals)
                                                and nc3 < len(State.goals[nr3])
                                                and State.goals[nr3][nc3] == gb3):
                                            if _resolve_blocker(
                                                    nr3, nc3, gb3,
                                                    path_cells_set | {(pr2, pc2)},
                                                    depth=0):
                                                _move_agent_away(j, path_cells_set)
                                                break
                                if (state.agent_rows[j] != pr2
                                        or state.agent_cols[j] != pc2):
                                    cleared_any = True
                                break
                    if cleared_any:
                        progress += 1
                        break

        # Second-pass retry: after all goals processed, retry any unsolved goals
        # with iterative blocker clearing. Handles cases like DASH where box A fails
        # initially (D blocks), D is displaced, other boxes are placed, then A's
        # path is clear; also handles multi-level blocking chains.
        second_pass_goals = [
            gk for gk in ordered_goals
            if gk not in solved_goals and state.boxes[gk[1]][gk[2]] != gk[0]
        ]
        for goal_key in second_pass_goals:
            if time.perf_counter() > deadline - 2.0:
                break
            letter, gr, gc = goal_key
            goal_cands = by_goal.get(goal_key, [])
            # Try direct first
            for cost, i, br, bc in goal_cands[:5]:
                if state.boxes[br][bc] != letter:
                    continue
                if _try_direct_task(i, br, bc, gr, gc, letter):
                    progress += 1
                    solved_goals.add(goal_key)
                    break
            if goal_key in solved_goals:
                continue
            # Iterative blocker clearing in second pass
            if not goal_cands or time.perf_counter() > deadline - 2.0:
                continue
            main_br, main_bc = goal_cands[0][2], goal_cands[0][3]
            if state.boxes[main_br][main_bc] != letter:
                continue
            sp_forbidden: set = (
                {(fg_r, fg_c) for fg_let, fg_r, fg_c in all_box_goals
                 if state.boxes[fg_r][fg_c] == fg_let}
                | agent_goal_cells
                | {(gr, gc)}
                | set(_trace_ideal_path(main_br, main_bc, gr, gc))
            )
            last_sp_blocker = None
            for _sp_attempt in range(7):
                sp_blocker = _first_blocker_on_path(main_br, main_bc, gr, gc, state)
                if sp_blocker is None or sp_blocker == last_sp_blocker:
                    break
                if not _resolve_blocker(sp_blocker[0], sp_blocker[1], sp_blocker[2],
                                        sp_forbidden, depth=0):
                    break
                progress += 1
                last_sp_blocker = sp_blocker
                for cost, i, br, bc in goal_cands[:5]:
                    if state.boxes[br][bc] != letter:
                        continue
                    if _try_direct_task(i, br, bc, gr, gc, letter):
                        solved_goals.add(goal_key)
                        break
                if goal_key in solved_goals:
                    break

        # If still no progress, try permissive path clearing as last resort
        if progress == 0:
            permissive_progress = False
            for goal_key in ordered_goals:
                if goal_key in solved_goals:
                    continue
                letter, gr, gc = goal_key
                if state.boxes[gr][gc] == letter:
                    continue
                goal_cands = by_goal.get(goal_key, [])
                for cost, i, br, bc in goal_cands[:3]:
                    if state.boxes[br][bc] != letter:
                        continue
                    if _permissive_clear(letter, br, bc, gr, gc):
                        permissive_progress = True
                        progress += 1
                        break
                if permissive_progress:
                    break

        if progress == 0:
            print("[decoupled] No progress this round — stuck, aborting", file=sys.stderr, flush=True)
            return None

    # ── Agent-goal phase ──────────────────────────────────────────────────────
    agent_goals: list[tuple[int, int, int]] = [
        (int(State.goals[r][c]), r, c)
        for r in range(len(State.goals))
        for c in range(len(State.goals[r]))
        if "0" <= State.goals[r][c] <= "9"
        and int(State.goals[r][c]) < num_agents
    ]

    if agent_goals:
        box_obstacles: frozenset[tuple[int, int]] = frozenset(
            (r, c)
            for r in range(len(state.boxes))
            for c in range(len(state.boxes[r]))
            if state.boxes[r][c]
        )
        agent_pos: dict[int, tuple[int, int]] = {
            i: (state.agent_rows[i], state.agent_cols[i]) for i in range(num_agents)
        }
        rows_g = len(State.walls)
        cols_g = len(State.walls[0])

        # Sort agent goals so "harder to reach" (fewer free neighbours) come first.
        # This prevents an agent parked at a transit cell from blocking a deeper goal.
        def _goal_access_count(gr: int, gc: int) -> int:
            goal_cells = {(r2, c2) for _, r2, c2 in agent_goals}
            return sum(
                1 for dr, dc in _DIRS
                if (0 <= gr + dr < rows_g and 0 <= gc + dc < cols_g
                    and not State.walls[gr + dr][gc + dc]
                    and (gr + dr, gc + dc) not in box_obstacles
                    and (gr + dr, gc + dc) not in goal_cells)
            )

        agent_goals.sort(key=lambda t: _goal_access_count(t[1], t[2]))

        def _try_agent_goal(agent_idx: int, gr: int, gc: int) -> bool:
            nonlocal state
            ar, ac = state.agent_rows[agent_idx], state.agent_cols[agent_idx]
            agent_pos[agent_idx] = (ar, ac)  # re-sync from actual state
            if ar == gr and ac == gc:
                return True
            extra: frozenset[tuple[int, int]] = box_obstacles | frozenset(
                (state.agent_rows[ai], state.agent_cols[ai])
                for ai in range(num_agents) if ai != agent_idx
            )
            acts = _plan_agent_move(ar, ac, gr, gc, extra, time.perf_counter() + 5.0)
            if acts is None:
                return False
            for act in acts:
                ja = [Action.NoOp] * num_agents
                ja[agent_idx] = act
                if not state.is_applicable(agent_idx, act) or state.is_conflicting(ja):
                    print(f"[decoupled] Agent-goal simulation mismatch for agent {agent_idx}",
                          file=sys.stderr, flush=True)
                    return False
                joint_plan.append(ja)
                state = state.result(ja)
            agent_pos[agent_idx] = (state.agent_rows[agent_idx], state.agent_cols[agent_idx])
            return True

        # First pass: move each agent to their goal
        failed_agent_goals: list[tuple[int, int, int]] = []
        for agent_idx, gr, gc in agent_goals:
            if time.perf_counter() > deadline - 1.0:
                break
            if not _try_agent_goal(agent_idx, gr, gc):
                print(f"[decoupled] Cannot reach agent goal for agent {agent_idx} → ({gr},{gc})",
                      file=sys.stderr, flush=True)
                failed_agent_goals.append((agent_idx, gr, gc))

        # Second pass: retry failed agents (blocking agents may have moved)
        still_failed_goals: list[tuple[int, int, int]] = []
        for agent_idx, gr, gc in failed_agent_goals:
            if time.perf_counter() > deadline - 1.0:
                still_failed_goals.append((agent_idx, gr, gc))
                break
            if not _try_agent_goal(agent_idx, gr, gc):
                print(f"[decoupled] Still cannot reach agent goal for agent {agent_idx} → ({gr},{gc})",
                      file=sys.stderr, flush=True)
                still_failed_goals.append((agent_idx, gr, gc))

        # Third pass: loop until fixed point. Each iteration first retries normal
        # routing (corridor may have cleared), then falls back to box-only obstacles
        # with dynamic blocker clearing. This handles chains like: agent3 clears
        # agent2, then agent4 can finally pass through the corridor agent3 vacated.
        # Sort remaining by goal_access_count DESCENDING so agents blocking corridors
        # (deeper goals) move out first and free space for others.
        remaining = sorted(still_failed_goals,
                           key=lambda t: -_goal_access_count(t[1], t[2]))
        for _iter in range(num_agents * 3 + 2):
            if not remaining or time.perf_counter() > deadline - 2.0:
                break
            # Recompute box_obstacles from current state — boxes may have been
            # displaced since the start of the agent-goal phase.
            box_obstacles = frozenset(
                (r, c)
                for r in range(len(state.boxes))
                for c in range(len(state.boxes[r]))
                if state.boxes[r][c]
            )
            next_remaining: list[tuple[int, int, int]] = []
            made_progress = False
            for agent_idx, gr, gc in remaining:
                if time.perf_counter() > deadline - 2.0:
                    next_remaining.append((agent_idx, gr, gc))
                    continue
                ar, ac = state.agent_rows[agent_idx], state.agent_cols[agent_idx]
                agent_pos[agent_idx] = (ar, ac)
                if ar == gr and ac == gc:
                    made_progress = True
                    continue
                # Try normal routing first — maybe another agent cleared the path
                if _try_agent_goal(agent_idx, gr, gc):
                    made_progress = True
                    continue
                # Fall back: path using only box obstacles (other stuck agents passable)
                acts = _plan_agent_move(ar, ac, gr, gc, box_obstacles, time.perf_counter() + 5.0)
                if acts is None:
                    print(f"[decoupled] Agent {agent_idx} truly unreachable → ({gr},{gc})",
                          file=sys.stderr, flush=True)
                    # Try pushing boxes off agent's ideal path to make it reachable.
                    # Add path cells to forbidden so boxes don't get re-parked on the path.
                    _ag_path = _trace_ideal_path(ar, ac, gr, gc)
                    _ag_forbidden: set = (
                        {(fg_r, fg_c) for fg_let, fg_r, fg_c in all_box_goals
                         if state.boxes[fg_r][fg_c] == fg_let}
                        | agent_goal_cells | {(gr, gc)} | set(_ag_path)
                    )
                    for _pr, _pc in _ag_path[1:]:
                        _bl = state.boxes[_pr][_pc] if state.boxes[_pr][_pc] else ""
                        if _bl and _resolve_blocker(_pr, _pc, _bl, _ag_forbidden, depth=0):
                            made_progress = True
                            # Refresh box_obstacles so subsequent checks use new positions
                            box_obstacles = frozenset(
                                (r, c)
                                for r in range(len(state.boxes))
                                for c in range(len(state.boxes[r]))
                                if state.boxes[r][c]
                            )
                            # keep clearing — remove all boxes from path in one pass
                    next_remaining.append((agent_idx, gr, gc))
                    continue
                # Pre-compute all cells on this path so blockers are moved completely off it
                path_cells: set[tuple[int, int]] = set()
                _tmp_r, _tmp_c = ar, ac
                for _act in acts:
                    _tmp_r += _act.agent_row_delta
                    _tmp_c += _act.agent_col_delta
                    path_cells.add((_tmp_r, _tmp_c))
                success = True
                blocker_agent = -1
                displaced_goal_agents: list[tuple[int, int, int]] = []
                for act in acts:
                    tr = state.agent_rows[agent_idx] + act.agent_row_delta
                    tc = state.agent_cols[agent_idx] + act.agent_col_delta
                    blocker_agent = -1
                    for other in range(num_agents):
                        if (other != agent_idx
                                and state.agent_rows[other] == tr
                                and state.agent_cols[other] == tc):
                            blocker_agent = other
                            # Track if blocker is at its goal (will need re-queuing)
                            other_goal = next(((bgr, bgc) for bi, bgr, bgc in _agent_goals_all
                                               if bi == other), None)
                            at_goal = (other_goal is not None
                                       and state.agent_rows[other] == other_goal[0]
                                       and state.agent_cols[other] == other_goal[1])
                            # Move blocker away from the ENTIRE path (not just current cell)
                            # so it doesn't keep getting displaced to the next path cell
                            _move_agent_away(other, path_cells)
                            # If we displaced an agent from its goal, re-queue it
                            if at_goal and (state.agent_rows[other] != other_goal[0]
                                            or state.agent_cols[other] != other_goal[1]):
                                displaced_goal_agents.append(
                                    (other, other_goal[0], other_goal[1]))
                            break
                    ja = [Action.NoOp] * num_agents
                    ja[agent_idx] = act
                    if not state.is_applicable(agent_idx, act) or state.is_conflicting(ja):
                        success = False
                        break
                    joint_plan.append(ja)
                    state = state.result(ja)
                if success:
                    agent_pos[agent_idx] = (state.agent_rows[agent_idx],
                                            state.agent_cols[agent_idx])
                    made_progress = True
                    # Re-queue any displaced goal agents
                    for disp in displaced_goal_agents:
                        if disp not in next_remaining and disp not in [(ai, gr2, gc2) for ai, gr2, gc2 in next_remaining]:
                            next_remaining.append(disp)
                else:
                    # Execution failed — try "cooperative yield": move current agent
                    # to a cell NOT on the blocker's path, so blocker can pass first.
                    yielded = False
                    if blocker_agent >= 0:
                        blocker_r = state.agent_rows[blocker_agent]
                        blocker_c = state.agent_cols[blocker_agent]
                        # Find blocker's goal
                        b_goal = next(((bgr, bgc) for bi, bgr, bgc in remaining
                                       if bi == blocker_agent), None)
                        if b_goal is None:
                            b_goal = next(((bgr, bgc) for bi, bgr, bgc in _agent_goals_all
                                           if bi == blocker_agent), None)
                        if b_goal is not None:
                            b_path = _plan_agent_move(blocker_r, blocker_c,
                                                      b_goal[0], b_goal[1],
                                                      box_obstacles,
                                                      time.perf_counter() + 2.0)
                            if b_path is not None:
                                b_path_cells: set[tuple[int, int]] = {
                                    (blocker_r, blocker_c)}
                                tmp_rb, tmp_cb = blocker_r, blocker_c
                                for _pb in b_path:
                                    tmp_rb += _pb.agent_row_delta
                                    tmp_cb += _pb.agent_col_delta
                                    b_path_cells.add((tmp_rb, tmp_cb))
                                # Move current agent away from blocker's path
                                if _move_agent_away(agent_idx, b_path_cells):
                                    yielded = True
                                    made_progress = True
                    next_remaining.append((agent_idx, gr, gc))
            if not made_progress:
                break
            remaining = next_remaining

    if not state.is_goal_state():
        print("[decoupled] Iterations exhausted, goal not fully reached", file=sys.stderr, flush=True)
        return None

    print(f"[decoupled] Solved — plan length {len(joint_plan)}", file=sys.stderr, flush=True)
    parallel_plan = _parallelize_plan(joint_plan, initial_state, num_agents)
    return parallel_plan
