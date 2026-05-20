import argparse
import sys
import time
from typing import TextIO

from searchclient import memory
from searchclient.action import Action
from searchclient.color import Color
from searchclient.frontier import Frontier, FrontierBestFirst, FrontierBFS, FrontierDFS
from searchclient.graphsearch import search, windowed_search
from searchclient.heuristic import HeuristicAStar, HeuristicGreedy, HeuristicPredictabilityAware, HeuristicWeightedAStar
from searchclient.state import State

# ── Phase 1 flag ──────────────────────────────────────────────────────────────
# Set False to revert to pre-Phase-1 cascade:
#   boxes: WA*(5) for 90s only (no cheap Greedy probe, no WA*(2) fallback)
#   MAPF:  old order — JointA*(60s) → CBS(10s) → GreedyMAPF → PIBT → DFS-CBS(20s) → CoopA* → Greedy(30s)
ENABLE_CASCADE_REORDER = True

# ── Phase 5 flag ──────────────────────────────────────────────────────────────
# Disabled: adaptive cascade added noise without clear benefit.
# Phase-1 fixed MAPF order is used when ENABLE_CASCADE_REORDER is True.
ENABLE_ADAPTIVE_CASCADE = False


def _select_mapf_cascade(num_agents: int, free_cells: int, density: float, accessible: int) -> list[str]:
    """
    Select the ordered list of MAPF algorithm names to try based on level features.

    Decision rules:
    # Rule 1: Very few agents in a small space → Joint A* first (optimal, fast)
    # Rule 2: High density (many agents / free cells) → PIBT excels
    # Rule 3: Many agents (>8) → skip Joint A* entirely (state space too large)
    # Rule 4: Default order: PIBT → GreedyMAPF → TokenPassing → CBS → CoopA* → JointA* → DFS-CBS → Greedy
    """
    default = ["PIBT", "GreedyMAPF", "TokenPassing", "CBS", "CoopA*", "JointA*", "DFS-CBS", "Greedy"]

    # Rule 1: Few agents in small accessible space — Joint A* first (optimal + fast).
    if num_agents <= 3 and accessible <= 50:
        return ["JointA*", "PIBT", "GreedyMAPF", "TokenPassing", "CBS", "CoopA*", "DFS-CBS", "Greedy"]

    # Rule 2: High density → PIBT handles corridor/tight problems best (already first in default).
    # density >= 0.3 means more than 30% of free cells have agents — very dense.
    if density >= 0.3:
        return ["PIBT", "GreedyMAPF", "TokenPassing", "CBS", "CoopA*", "DFS-CBS", "Greedy"]

    # Rule 3: Many agents → Joint A* state space is intractable, remove it.
    if num_agents > 8:
        return ["PIBT", "GreedyMAPF", "TokenPassing", "CBS", "CoopA*", "DFS-CBS", "Greedy"]

    return default


class SearchClient:
    @staticmethod
    def parse_level(server_messages: TextIO) -> State:
        # We can assume that the level file is conforming to specification, since the server verifies this.
        # Read domain.
        server_messages.readline()  # #domain
        server_messages.readline()  # hospital

        # Read Level name.
        server_messages.readline()  # #levelname
        server_messages.readline()  # <name>

        # Read colors.
        server_messages.readline()  # #colors
        agent_colors: list[Color | None] = [None for _ in range(10)]
        box_colors: list[Color | None] = [None for _ in range(26)]
        line = server_messages.readline()
        while not line.startswith("#"):
            split = line.split(":")
            color = Color.from_string(split[0].strip())
            entities = [e.strip() for e in split[1].split(",")]
            for e in entities:
                if "0" <= e <= "9":
                    agent_colors[ord(e) - ord("0")] = color
                elif "A" <= e <= "Z":
                    box_colors[ord(e) - ord("A")] = color
            line = server_messages.readline()

        # Read initial state.
        # line is currently "#initial".
        num_rows = 0
        num_cols = 0
        level_lines: list[str] = []
        line = server_messages.readline()
        while not line.startswith("#"):
            level_lines.append(line)
            num_cols = max(num_cols, len(line))
            num_rows += 1
            line = server_messages.readline()

        num_agents = 0
        agent_rows: list[int] = [-1 for _ in range(10)]
        agent_cols: list[int] = [-1 for _ in range(10)]
        walls = [[False for _ in range(num_cols)] for _ in range(num_rows)]
        boxes = [["" for _ in range(num_cols)] for _ in range(num_rows)]
        for row, line in enumerate(level_lines):
            for col, c in enumerate(line):
                if "0" <= c <= "9":
                    agent_rows[ord(c) - ord("0")] = row
                    agent_cols[ord(c) - ord("0")] = col
                    num_agents += 1
                elif "A" <= c <= "Z":
                    boxes[row][col] = c
                elif c == "+":
                    walls[row][col] = True
        del agent_rows[num_agents:]
        del agent_cols[num_agents:]

        # Read goal state.
        # line is currently "#goal".
        goals = [["" for _ in range(num_cols)] for _ in range(num_rows)]
        line = server_messages.readline()
        row = 0
        while not line.startswith("#"):
            for col, c in enumerate(line):
                if "0" <= c <= "9" or "A" <= c <= "Z":
                    goals[row][col] = c

            row += 1
            line = server_messages.readline()

        # End.
        # line is currently "#end".

        State.agent_colors = agent_colors
        State.walls = walls
        State.box_colors = box_colors
        State.goals = goals
        State._static_hash = hash((
            tuple(agent_colors),
            tuple(box_colors),
            tuple(tuple(row) for row in goals),
            tuple(tuple(row) for row in walls),
        ))
        State._dead_cells = State._compute_dead_cells()
        State._dead_pairs = State._compute_dead_pairs()
        State._init_zobrist(num_rows, num_cols)
        return State(agent_rows, agent_cols, boxes)

    @staticmethod
    def print_search_status(start_time: int, explored: set[State], frontier: Frontier) -> None:
        elapsed_time = time.perf_counter() - start_time
        print(
            f"#Expanded: {len(explored):8,}, #Frontier: {frontier.size():8,}, "
            f"#Generated: {len(explored) + frontier.size():8,}, Time: {elapsed_time:3.3f} s\n"
            f"[Alloc: {memory.get_usage():4.2f} MB, MaxAlloc: {memory.max_usage:4.2f} MB]",
            file=sys.stderr,
            flush=True,
        )

    @staticmethod
    def main(args: argparse.Namespace) -> None:
        # Use stderr to print to the console.
        print(
            "SearchClient initializing. I am sending this using the error output stream.", file=sys.stderr, flush=True
        )

        # Send client name to server.
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="ASCII")
        print("SearchClient", flush=True)

        # We can also print comments to stdout by prefixing with a #.
        print("#This is a comment.", flush=True)

        # Parse the level.
        server_messages = sys.stdin
        if hasattr(server_messages, "reconfigure"):
            server_messages.reconfigure(encoding="ASCII")
        initial_state = SearchClient.parse_level(server_messages)

        # Select search strategy.
        frontier: Frontier
        if args.bfs:
            frontier = FrontierBFS()
        elif args.dfs:
            frontier = FrontierDFS()
        elif args.astar:
            _h: object = HeuristicAStar(initial_state)
            if args.predictable is not False:
                _h = HeuristicPredictabilityAware(initial_state, _h, lambda_=args.predictable)
            frontier = FrontierBestFirst(_h)
        elif args.wastar is not False:
            _h = HeuristicWeightedAStar(initial_state, args.wastar)
            if args.predictable is not False:
                _h = HeuristicPredictabilityAware(initial_state, _h, lambda_=args.predictable)
            frontier = FrontierBestFirst(_h)
        elif args.greedy:
            _h = HeuristicGreedy(initial_state)
            if args.predictable is not False:
                _h = HeuristicPredictabilityAware(initial_state, _h, lambda_=args.predictable)
            frontier = FrontierBestFirst(_h)
        else:
            has_boxes = any(
                initial_state.boxes[r][c]
                for r in range(len(initial_state.boxes))
                for c in range(len(initial_state.boxes[r]))
            )

            _cascade_start = time.perf_counter()
            _server_deadline = _cascade_start + 165.0  # leave 15s buffer before server's 180s

            if not has_boxes:
                from searchclient.cbs import (
                    cbs_search, _count_reachable_cells, _joint_astar,
                    _greedy_mapf, pibt_search, dfs_cbs_search, cooperative_astar,
                    token_passing_search,
                )
                num_agents = len(initial_state.agent_rows)

                # Compute level features for adaptive cascade selection.
                accessible = _count_reachable_cells(initial_state)
                _rows = len(State.walls)
                _cols = len(State.walls[0]) if _rows > 0 else 0
                free_cells = sum(1 for _r in range(_rows) for _c in range(_cols) if not State.walls[_r][_c])
                density = num_agents / max(1, free_cells)
                if not ENABLE_CASCADE_REORDER:
                    # Pre-Phase-1 MAPF order: JointA* first, then CBS, then rest
                    _cascade_order = ["JointA*", "CBS", "GreedyMAPF", "TokenPassing", "PIBT", "DFS-CBS", "CoopA*", "Greedy"]
                elif ENABLE_ADAPTIVE_CASCADE:
                    _cascade_order = _select_mapf_cascade(num_agents, free_cells, density, accessible)
                else:
                    _cascade_order = ["JointA*", "PIBT", "GreedyMAPF", "TokenPassing", "CBS", "CoopA*", "DFS-CBS", "Greedy"]
                print(f"[cascade] Level features: agents={num_agents}, free={free_cells}, density={density:.3f}, accessible={accessible}", file=sys.stderr, flush=True)
                print(f"[cascade] Selected order: {_cascade_order}", file=sys.stderr, flush=True)

                def _send_plan(plan: list) -> None:
                    for joint_action in plan:
                        print("|".join(a.name_ + "@" + a.name_ for a in joint_action), flush=True)
                        server_messages.readline()

                def _try_algorithm(name: str) -> "list | None":
                    """Run the named algorithm and return its plan or None."""
                    nonlocal accessible
                    if name == "PIBT":
                        _t0 = time.perf_counter()
                        print("[cascade] Trying PIBT (budget: 1.0s)...", file=sys.stderr, flush=True)
                        result = pibt_search(initial_state)
                        _el = time.perf_counter() - _t0
                        if result is not None:
                            print(f"[cascade] PIBT solved in {_el:.3f}s (length {len(result)})", file=sys.stderr, flush=True)
                        else:
                            print(f"[cascade] PIBT failed in {_el:.3f}s", file=sys.stderr, flush=True)
                        return result
                    elif name == "GreedyMAPF":
                        _t0 = time.perf_counter()
                        print("[cascade] Trying Greedy MAPF (budget: 1.0s)...", file=sys.stderr, flush=True)
                        result = _greedy_mapf(initial_state, deadline=time.perf_counter() + 1.0)
                        _el = time.perf_counter() - _t0
                        if result is not None:
                            print(f"[cascade] Greedy MAPF solved in {_el:.3f}s (length {len(result)})", file=sys.stderr, flush=True)
                        else:
                            print(f"[cascade] Greedy MAPF failed in {_el:.3f}s", file=sys.stderr, flush=True)
                        return result
                    elif name == "TokenPassing":
                        _t0 = time.perf_counter()
                        _tp_budget = min(8.0, _server_deadline - time.perf_counter() - 10.0)
                        if _tp_budget < 2.0:
                            return None
                        print(f"[cascade] Trying Token Passing (budget: {_tp_budget:.1f}s)...",
                              file=sys.stderr, flush=True)
                        result = token_passing_search(initial_state, deadline=time.perf_counter() + _tp_budget)
                        _el = time.perf_counter() - _t0
                        if result is not None:
                            print(f"[cascade] Token Passing solved in {_el:.3f}s (length {len(result)})",
                                  file=sys.stderr, flush=True)
                        else:
                            print(f"[cascade] Token Passing failed in {_el:.3f}s",
                                  file=sys.stderr, flush=True)
                        return result
                    elif name == "CBS":
                        _t0 = time.perf_counter()
                        print("[cascade] Trying CBS (budget: 15.0s)...", file=sys.stderr, flush=True)
                        result = cbs_search(initial_state, deadline=time.perf_counter() + 15.0)
                        _el = time.perf_counter() - _t0
                        if result is not None:
                            print(f"[cascade] CBS solved in {_el:.3f}s (length {len(result)})", file=sys.stderr, flush=True)
                        else:
                            print(f"[cascade] CBS failed in {_el:.3f}s", file=sys.stderr, flush=True)
                        return result
                    elif name == "CoopA*":
                        _t0 = time.perf_counter()
                        print("[cascade] Trying Cooperative A* (budget: 5.0s)...", file=sys.stderr, flush=True)
                        result = cooperative_astar(initial_state, deadline=time.perf_counter() + 5.0)
                        _el = time.perf_counter() - _t0
                        if result is not None:
                            print(f"[cascade] Cooperative A* solved in {_el:.3f}s (length {len(result)})", file=sys.stderr, flush=True)
                        else:
                            print(f"[cascade] Cooperative A* failed in {_el:.3f}s", file=sys.stderr, flush=True)
                        return result
                    elif name == "JointA*":
                        joint_size = 1
                        for k in range(num_agents):
                            joint_size *= max(1, accessible - k)
                            if joint_size > 6_000_000:
                                break
                        if joint_size > 6_000_000:
                            print(f"[cascade] Skipping Joint A* (state space too large: {accessible} cells)", file=sys.stderr, flush=True)
                            return None
                        _t0 = time.perf_counter()
                        print(f"[cascade] Trying Joint A* (budget: 20.0s, state space: {accessible} cells)...", file=sys.stderr, flush=True)
                        result = _joint_astar(initial_state, deadline=time.perf_counter() + 20.0)
                        _el = time.perf_counter() - _t0
                        if result is not None:
                            print(f"[cascade] Joint A* solved in {_el:.3f}s (length {len(result)})", file=sys.stderr, flush=True)
                        else:
                            print(f"[cascade] Joint A* failed in {_el:.3f}s", file=sys.stderr, flush=True)
                        return result
                    elif name == "DFS-CBS":
                        _remaining = _server_deadline - time.perf_counter()
                        _dfs_budget = max(1.0, _remaining / 2)
                        _t0 = time.perf_counter()
                        print(f"[cascade] Trying DFS-CBS (budget: {_dfs_budget:.1f}s)...", file=sys.stderr, flush=True)
                        result = dfs_cbs_search(initial_state, deadline=time.perf_counter() + _dfs_budget)
                        _el = time.perf_counter() - _t0
                        if result is not None:
                            print(f"[cascade] DFS-CBS solved in {_el:.3f}s (length {len(result)})", file=sys.stderr, flush=True)
                        else:
                            print(f"[cascade] DFS-CBS failed in {_el:.3f}s", file=sys.stderr, flush=True)
                        return result
                    elif name == "Greedy":
                        _t0 = time.perf_counter()
                        _gbf_budget = _server_deadline - time.perf_counter()
                        print(f"[cascade] Trying Greedy best-first (budget: {_gbf_budget:.1f}s)...", file=sys.stderr, flush=True)
                        _h_mapf_gbf = HeuristicGreedy(initial_state)
                        if args.predictable is not False:
                            _h_mapf_gbf = HeuristicPredictabilityAware(initial_state, _h_mapf_gbf, lambda_=args.predictable)
                        _frontier = FrontierBestFirst(_h_mapf_gbf)
                        result = search(initial_state, _frontier, deadline=_server_deadline)
                        _el = time.perf_counter() - _t0
                        if result is not None:
                            print(f"[cascade] Greedy best-first solved in {_el:.3f}s (length {len(result)})", file=sys.stderr, flush=True)
                        else:
                            print(f"[cascade] Greedy best-first failed in {_el:.3f}s", file=sys.stderr, flush=True)
                        return result
                    return None

                def _validate_plan(plan: list, state: "State") -> bool:
                    """
                    Validate a joint-action plan by simulating it step by step.

                    For each step: checks every agent's action is individually
                    applicable in the pre-step state (catches following, wall moves,
                    etc.), checks no two agents conflict on destinations, then
                    advances the state.  Finally checks the goal condition.

                    Applicable-check semantics match the hospital server: a Move is
                    only valid if the destination cell is free *before* any actions
                    are applied in that step, so simultaneous "follow" moves are
                    correctly rejected.
                    """
                    cur = state
                    for joint_action in plan:
                        if not all(cur.is_applicable(i, a) for i, a in enumerate(joint_action)):
                            return False
                        if cur.is_conflicting(joint_action):
                            return False
                        cur = cur.result(joint_action)
                    return cur.is_goal_state()

                for _algo_name in _cascade_order:
                    plan = _try_algorithm(_algo_name)
                    if plan is not None:
                        if not _validate_plan(plan, initial_state):
                            print(f"[cascade] {_algo_name} returned invalid plan, rejecting",
                                  file=sys.stderr, flush=True)
                            continue
                        # LNS2 post-processing: improve plan if ≥30s remain.
                        _lns_remaining = _server_deadline - time.perf_counter()
                        if _lns_remaining >= 30.0:
                            from searchclient.lns import lns2_improve
                            _lns_deadline = _server_deadline - 5.0
                            _improved = lns2_improve(plan, initial_state, _lns_deadline, is_mapf=True)
                            if not _validate_plan(_improved, initial_state):
                                print("[lns2] WARNING: returned invalid plan, falling back to original",
                                      file=sys.stderr, flush=True)
                            else:
                                plan = _improved
                        _send_plan(plan)
                        return

                print("Unable to solve level.", file=sys.stderr, flush=True)
                sys.exit(0)

            # Box-path cascade.
            # For levels with many agents the joint state space is intractable,
            # so we try the decoupled sub-task planner first and fall back to
            # joint WA* only for simpler instances.
            if args.predictable is not False:
                print(f"[predictability] enabled on box cascade, λ={args.predictable}", file=sys.stderr, flush=True)

            from searchclient.ma_planner import decoupled_box_plan, get_box_blocked_hits
            plan = None
            _num_box_agents = len(initial_state.agent_rows)

            # Count boxes to gauge state-space complexity
            _num_boxes = sum(
                1
                for r in range(len(initial_state.boxes))
                for c in range(len(initial_state.boxes[r]))
                if initial_state.boxes[r][c]
            )

            # Estimate state-space complexity upfront (used for WA* and LNS2 skipping)
            _state_space_estimate = _num_boxes * _num_box_agents
            _skip_wa = _state_space_estimate > 50  # too large for WA* / LNS2

            def _try_independence_decoupled(state: State, deadline: float) -> list[list[Action]] | None:
                from collections import deque
                rows = len(State.walls)
                cols = len(State.walls[0]) if rows > 0 else 0
                comp = [[-1] * cols for _ in range(rows)]
                comp_id = 0
                for r in range(rows):
                    for c in range(cols):
                        if State.walls[r][c] or comp[r][c] != -1:
                            continue
                        q = deque([(r, c)])
                        comp[r][c] = comp_id
                        while q:
                            cr, cc = q.popleft()
                            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                                nr, nc = cr + dr, cc + dc
                                if (
                                    0 <= nr < rows
                                    and 0 <= nc < cols
                                    and not State.walls[nr][nc]
                                    and comp[nr][nc] == -1
                                ):
                                    comp[nr][nc] = comp_id
                                    q.append((nr, nc))
                        comp_id += 1

                comp_agents: dict[int, list[int]] = {i: [] for i in range(comp_id)}
                comp_boxes: dict[int, list[tuple[int, int, str]]] = {i: [] for i in range(comp_id)}
                comp_box_goals: dict[int, list[tuple[str, int, int]]] = {i: [] for i in range(comp_id)}
                comp_agent_goals: dict[int, list[tuple[int, int, int]]] = {i: [] for i in range(comp_id)}

                for i in range(len(state.agent_rows)):
                    cid = comp[state.agent_rows[i]][state.agent_cols[i]]
                    if cid >= 0:
                        comp_agents[cid].append(i)

                for r in range(rows):
                    for c in range(cols):
                        if state.boxes[r][c]:
                            cid = comp[r][c]
                            if cid >= 0:
                                comp_boxes[cid].append((r, c, state.boxes[r][c]))
                        g = State.goals[r][c]
                        if "A" <= g <= "Z":
                            cid = comp[r][c]
                            if cid >= 0:
                                comp_box_goals[cid].append((g, r, c))
                        elif "0" <= g <= "9":
                            idx = ord(g) - ord("0")
                            cid = comp[r][c]
                            if cid >= 0:
                                comp_agent_goals[cid].append((idx, r, c))

                active_components = [cid for cid, agents in comp_agents.items() if agents]
                if len(active_components) <= 1:
                    return None

                # Independence sanity checks.
                for cid in active_components:
                    if comp_box_goals[cid] and not comp_agents[cid]:
                        return None
                    box_letters = {b for _, _, b in comp_boxes[cid]}
                    for letter, _, _ in comp_box_goals[cid]:
                        if letter not in box_letters:
                            return None
                    for idx, gr, gc in comp_agent_goals[cid]:
                        if idx >= len(state.agent_rows):
                            continue
                        start_cid = comp[state.agent_rows[idx]][state.agent_cols[idx]]
                        if start_cid != cid:
                            return None

                full_plan: list[list[Action]] = []
                total_agents = len(state.agent_rows)
                orig_agent_colors = State.agent_colors
                orig_box_colors = State.box_colors
                orig_goals = State.goals
                orig_walls = State.walls
                orig_static = State._static_hash
                orig_dead = State._dead_cells
                orig_dead_pairs = State._dead_pairs
                orig_rows = rows
                orig_cols = cols

                try:
                    remaining_components = len(active_components)
                    for cid in active_components:
                        if time.perf_counter() > deadline - 2.0:
                            return None
                        sub_agent_indices = comp_agents[cid]
                        sub_agent_rows = [state.agent_rows[i] for i in sub_agent_indices]
                        sub_agent_cols = [state.agent_cols[i] for i in sub_agent_indices]
                        sub_agent_colors = [orig_agent_colors[i] for i in sub_agent_indices]
                        sub_boxes = [["" for _ in range(cols)] for _ in range(rows)]
                        for br, bc, bl in comp_boxes[cid]:
                            sub_boxes[br][bc] = bl
                        sub_goals = [["" for _ in range(cols)] for _ in range(rows)]
                        for letter, gr, gc in comp_box_goals[cid]:
                            sub_goals[gr][gc] = letter
                        for idx, gr, gc in comp_agent_goals[cid]:
                            if idx in sub_agent_indices:
                                sub_goals[gr][gc] = str(sub_agent_indices.index(idx))

                        State.agent_colors = sub_agent_colors
                        State.box_colors = orig_box_colors
                        State.goals = sub_goals
                        State.walls = orig_walls
                        State._static_hash = hash((
                            tuple(State.agent_colors),
                            tuple(State.box_colors),
                            tuple(tuple(row) for row in State.goals),
                            tuple(tuple(row) for row in State.walls),
                        ))
                        State._dead_cells = State._compute_dead_cells()
                        State._dead_pairs = State._compute_dead_pairs()
                        State._init_zobrist(orig_rows, orig_cols)

                        comp_deadline = time.perf_counter() + max(
                            5.0, (deadline - time.perf_counter()) / max(1, remaining_components)
                        )
                        sub_state = State(sub_agent_rows, sub_agent_cols, sub_boxes)
                        sub_plan = decoupled_box_plan(sub_state, comp_deadline)
                        if sub_plan is None:
                            return None
                        for joint_action in sub_plan:
                            full_ja = [Action.NoOp] * total_agents
                            for sub_idx, orig_idx in enumerate(sub_agent_indices):
                                full_ja[orig_idx] = joint_action[sub_idx]
                            full_plan.append(full_ja)
                        remaining_components -= 1
                finally:
                    State.agent_colors = orig_agent_colors
                    State.box_colors = orig_box_colors
                    State.goals = orig_goals
                    State.walls = orig_walls
                    State._static_hash = orig_static
                    State._dead_cells = orig_dead
                    State._dead_pairs = orig_dead_pairs
                    State._init_zobrist(orig_rows, orig_cols)

                return full_plan

            def _rolling_horizon_search(state: State, deadline: float) -> list[list[Action]] | None:
                plan: list[list[Action]] = []
                current_state = state
                window = 30
                per_window = 6.0
                while time.perf_counter() < deadline - 2.0:
                    if current_state.is_goal_state():
                        return plan
                    _h_win = HeuristicWeightedAStar(current_state, 5)
                    frontier = FrontierBestFirst(_h_win)
                    win_deadline = min(deadline - 1.0, time.perf_counter() + per_window)
                    partial = windowed_search(current_state, frontier, window, deadline=win_deadline)
                    if not partial:
                        break
                    for ja in partial:
                        plan.append(ja)
                        current_state = current_state.result(ja)
                if current_state.is_goal_state():
                    return plan
                return None

            # ── Decoupled planner first ────────────────────────────────────────
            # Give more time to the decoupled planner for complex levels
            if _num_box_agents >= 1:
                if _num_boxes >= 10 or _num_box_agents >= 3:
                    # Complex level: give decoupled most of the budget
                    _dec_budget = min(150.0, _server_deadline - time.perf_counter() - 10.0)
                else:
                    _dec_budget = min(100.0, _server_deadline - time.perf_counter() - 20.0)
                if _dec_budget >= 5.0:
                    print(f"[cascade] {_num_box_agents} agents, {_num_boxes} boxes — trying decoupled planner "
                          f"(budget: {_dec_budget:.1f}s)...", file=sys.stderr, flush=True)
                    _t0 = time.perf_counter()
                    plan = decoupled_box_plan(initial_state, time.perf_counter() + _dec_budget)
                    _elapsed = time.perf_counter() - _t0
                    if plan is not None:
                        print(f"[cascade] Decoupled planner solved in {_elapsed:.3f}s "
                              f"(length {len(plan)})", file=sys.stderr, flush=True)
                    else:
                        print(f"[cascade] Decoupled planner failed in {_elapsed:.3f}s — "
                              "falling through to WA*", file=sys.stderr, flush=True)

            # ── Independence Detection (ID) before WA* ─────────────────────────
            if plan is None:
                _id_budget = min(20.0, _server_deadline - time.perf_counter() - 15.0)
                if _id_budget >= 5.0:
                    print(f"[cascade] Trying Independence Detection (budget: {_id_budget:.1f}s)...",
                          file=sys.stderr, flush=True)
                    _t0 = time.perf_counter()
                    plan = _try_independence_decoupled(initial_state, time.perf_counter() + _id_budget)
                    _elapsed = time.perf_counter() - _t0
                    if plan is not None:
                        print(f"[cascade] Independence Detection solved in {_elapsed:.3f}s "
                              f"(length {len(plan)})", file=sys.stderr, flush=True)
                    else:
                        print(f"[cascade] Independence Detection failed in {_elapsed:.3f}s",
                              file=sys.stderr, flush=True)

            # ── Rolling-Horizon Planning (RH) ─────────────────────────────────
            if plan is None:
                _blocked_hits = get_box_blocked_hits()
                _rh_budget = min(20.0, _server_deadline - time.perf_counter() - 12.0)
                if _blocked_hits >= 200 and _rh_budget >= 6.0:
                    print(f"[cascade] Triggering rolling-horizon (blocked hits={_blocked_hits}, "
                          f"budget: {_rh_budget:.1f}s)...", file=sys.stderr, flush=True)
                    _t0 = time.perf_counter()
                    plan = _rolling_horizon_search(initial_state, time.perf_counter() + _rh_budget)
                    _elapsed = time.perf_counter() - _t0
                    if plan is not None:
                        print(f"[cascade] Rolling-horizon solved in {_elapsed:.3f}s "
                              f"(length {len(plan)})", file=sys.stderr, flush=True)
                    else:
                        print(f"[cascade] Rolling-horizon failed in {_elapsed:.3f}s",
                              file=sys.stderr, flush=True)

            # ── WA* cascade — skip for very large state spaces ─────────────────
            if plan is None:

                if _skip_wa:
                    print(f"[cascade] Skipping WA* (too large: {_num_boxes} boxes × "
                          f"{_num_box_agents} agents)", file=sys.stderr, flush=True)
                    # Still allow a short WA*/Greedy probe if time permits.
                    _mini_remaining = _server_deadline - time.perf_counter()
                    _mini_wa_budget = min(6.0, _mini_remaining - 25.0)
                    if _mini_wa_budget >= 3.0:
                        print(f"[cascade] Trying short WA*(5) despite size (budget: {_mini_wa_budget:.1f}s)...",
                              file=sys.stderr, flush=True)
                        _t0 = time.perf_counter()
                        _h_box = HeuristicWeightedAStar(initial_state, 5)
                        if args.predictable is not False:
                            _h_box = HeuristicPredictabilityAware(initial_state, _h_box, lambda_=args.predictable)
                        frontier = FrontierBestFirst(_h_box)
                        plan = search(initial_state, frontier, deadline=time.perf_counter() + _mini_wa_budget)
                        _elapsed = time.perf_counter() - _t0
                        if plan is not None:
                            print(f"[cascade] Short WA*(5) solved in {_elapsed:.3f}s (length {len(plan)})",
                                  file=sys.stderr, flush=True)
                        else:
                            print(f"[cascade] Short WA*(5) failed in {_elapsed:.3f}s",
                                  file=sys.stderr, flush=True)
                    if plan is None:
                        _mini_gbf_budget = min(6.0, _server_deadline - time.perf_counter() - 15.0)
                        if _mini_gbf_budget >= 3.0:
                            print(f"[cascade] Trying short Greedy (budget: {_mini_gbf_budget:.1f}s)...",
                                  file=sys.stderr, flush=True)
                            _t0 = time.perf_counter()
                            _h_gbf = HeuristicGreedy(initial_state)
                            if args.predictable is not False:
                                _h_gbf = HeuristicPredictabilityAware(initial_state, _h_gbf, lambda_=args.predictable)
                            frontier = FrontierBestFirst(_h_gbf)
                            plan = search(initial_state, frontier, deadline=time.perf_counter() + _mini_gbf_budget)
                            _elapsed = time.perf_counter() - _t0
                            if plan is not None:
                                print(f"[cascade] Short Greedy solved in {_elapsed:.3f}s (length {len(plan)})",
                                      file=sys.stderr, flush=True)
                            else:
                                print(f"[cascade] Short Greedy failed in {_elapsed:.3f}s",
                                      file=sys.stderr, flush=True)
                else:
                    # For small state spaces, start with lower weights to find solutions faster
                    # (high weights like 100/50 often time out on tight corridors)
                    if _state_space_estimate <= 15:
                        _wa_schedule = [(20, 10.0), (10, 12.0), (5, 20.0), (2, 30.0)]
                    else:
                        _wa_schedule = [(100, 15.0), (50, 15.0), (20, 15.0), (10, 20.0), (5, 30.0)]
                    for _w, _bud in _wa_schedule:
                        _remaining = _server_deadline - time.perf_counter()
                        if _remaining < 35.0:
                            break
                        _bud = min(_bud, _remaining - 35.0)
                        if _bud < 3.0:
                            break
                        print(f"[cascade] Trying WA*({_w}) (budget: {_bud:.1f}s)...", file=sys.stderr, flush=True)
                        _t0 = time.perf_counter()
                        _h_box = HeuristicWeightedAStar(initial_state, _w)
                        if args.predictable is not False:
                            _h_box = HeuristicPredictabilityAware(initial_state, _h_box, lambda_=args.predictable)
                        frontier = FrontierBestFirst(_h_box)
                        plan = search(initial_state, frontier, deadline=time.perf_counter() + _bud)
                        _elapsed = time.perf_counter() - _t0
                        if plan is not None:
                            print(f"[cascade] WA*({_w}) solved in {_elapsed:.3f}s (length {len(plan)})", file=sys.stderr, flush=True)
                            break
                        print(f"[cascade] WA*({_w}) failed in {_elapsed:.3f}s", file=sys.stderr, flush=True)

            # ── Greedy best-first fallback — skip for huge state spaces ──────────
            if plan is None and not _skip_wa:
                _gbf_budget = max(0.0, _server_deadline - time.perf_counter() - 15.0)
                if _gbf_budget >= 3.0:
                    print(f"[cascade] Trying Greedy fallback (budget: {_gbf_budget:.1f}s)...", file=sys.stderr, flush=True)
                    _t0 = time.perf_counter()
                    _h_gbf = HeuristicGreedy(initial_state)
                    if args.predictable is not False:
                        _h_gbf = HeuristicPredictabilityAware(initial_state, _h_gbf, lambda_=args.predictable)
                    frontier = FrontierBestFirst(_h_gbf)
                    plan = search(initial_state, frontier, deadline=time.perf_counter() + _gbf_budget)
                    _elapsed = time.perf_counter() - _t0
                    if plan is not None:
                        print(f"[cascade] Greedy fallback solved in {_elapsed:.3f}s (length {len(plan)})", file=sys.stderr, flush=True)
                    else:
                        print(f"[cascade] Greedy fallback failed in {_elapsed:.3f}s", file=sys.stderr, flush=True)

            # ── Second decoupled pass with remaining time ──────────────────────
            if plan is None:
                _dec_budget2 = _server_deadline - time.perf_counter() - 3.0
                if _dec_budget2 >= 3.0:
                    print(f"[cascade] Trying decoupled planner 2nd pass (budget: {_dec_budget2:.1f}s)...", file=sys.stderr, flush=True)
                    _t0 = time.perf_counter()
                    plan = decoupled_box_plan(initial_state, time.perf_counter() + _dec_budget2)
                    _elapsed = time.perf_counter() - _t0
                    if plan is not None:
                        print(f"[cascade] Decoupled planner solved in {_elapsed:.3f}s (length {len(plan)})", file=sys.stderr, flush=True)
                    else:
                        print(f"[cascade] Decoupled planner failed in {_elapsed:.3f}s", file=sys.stderr, flush=True)

            if plan is None:
                print("Unable to solve level.", file=sys.stderr, flush=True)
                sys.exit(0)

            # LNS2 post-processing: skip for huge state spaces (LNS2 won't help).
            # Cap to 12s so plan is always output well before server deadline.
            _lns_remaining = _server_deadline - time.perf_counter()
            _lns_cap = min(12.0, _lns_remaining - 10.0)
            if _lns_cap >= 10.0 and not _skip_wa:
                from searchclient.lns import lns2_improve
                _lns_deadline = time.perf_counter() + _lns_cap
                _improved = lns2_improve(plan, initial_state, _lns_deadline, is_mapf=False)
                # Box-plan validator: simulate and check goal state.
                def _validate_box_plan(p: list, s: "State") -> bool:
                    cur = s
                    for joint_action in p:
                        if not all(cur.is_applicable(i, a) for i, a in enumerate(joint_action)):
                            return False
                        if cur.is_conflicting(joint_action):
                            return False
                        cur = cur.result(joint_action)
                    return cur.is_goal_state()
                if not _validate_box_plan(_improved, initial_state):
                    print("[lns2] WARNING: returned invalid plan, falling back to original",
                          file=sys.stderr, flush=True)
                else:
                    plan = _improved

            print(f"Found solution of length {len(plan)}.", file=sys.stderr, flush=True)
            for joint_action in plan:
                print("|".join(a.name_ + "@" + a.name_ for a in joint_action), flush=True)
                _response = server_messages.readline()
            return  # avoid double-printing below

        # Search for a plan (explicit strategy path).
        print(f"Starting {frontier.get_name()}.", file=sys.stderr, flush=True)
        plan = search(initial_state, frontier)

        # Print plan to server.
        if plan is None:
            print("Unable to solve level.", file=sys.stderr, flush=True)
            sys.exit(0)
        else:
            print(f"Found solution of length {len(plan)}.", file=sys.stderr, flush=True)

            for joint_action in plan:
                print("|".join(a.name_ + "@" + a.name_ for a in joint_action), flush=True)
                # We must read the server's response to not fill up the stdin buffer and block the server.
                _response = server_messages.readline()


if __name__ == "__main__":
    # Program arguments.
    parser = argparse.ArgumentParser(description="Simple client based on state-space graph search.")
    parser.add_argument(
        "--max-memory",
        metavar="<MB>",
        type=float,
        default=2048.0,
        help="The maximum memory usage allowed in MB (soft limit, default 2048).",
    )

    strategy_group = parser.add_mutually_exclusive_group()
    strategy_group.add_argument("-bfs", action="store_true", dest="bfs", help="Use the BFS strategy.")
    strategy_group.add_argument("-dfs", action="store_true", dest="dfs", help="Use the DFS strategy.")
    strategy_group.add_argument("-astar", action="store_true", dest="astar", help="Use the A* strategy.")
    strategy_group.add_argument(
        "-wastar",
        action="store",
        dest="wastar",
        nargs="?",
        type=int,
        default=False,
        const=5,
        help="Use the WA* strategy.",
    )
    strategy_group.add_argument("-greedy", action="store_true", dest="greedy", help="Use the Greedy strategy.")

    parser.add_argument(
        "--predictable",
        metavar="<λ>",
        nargs="?",
        type=float,
        default=False,
        const=2.5,
        help=(
            "Add the predictability-awareness penalty from arXiv:2411.06223v2. "
            "λ (default 2.5) controls the KL-divergence weight: higher values "
            "push agents to follow their BFS-predicted paths more closely."
        ),
    )

    args = parser.parse_args()

    # Set max memory usage allowed (soft limit).
    memory.max_usage = args.max_memory

    # Run client.
    SearchClient.main(args)

"""
It reads the level from the server, selects a search strategy based on the program arguments, and then searches for a plan using that strategy. Finally, it sends the plan to the server.
It reads the level from the server through standard input (stdin)
  server_messages = sys.stdin  # ← Read from stdin (server sends level here)
The method reads the level line by line in parse_level method which processes the domain, level name, colors, initial state, and goal state until it encounters the "#end" line. The parsed information is used to create an initial State object that represents the starting configuration of the level.

Initial state is constructed in 2 parts:
    i) Class variables (shared across all states) like agent colors, walls, box colors, and goals are set directly on the State class.
    ii) Instance variables (specific to each state) like agent positions and box positions are set in the State constructor when creating the initial state object.

Invoking the search startegy:
 Select search strategy based on command-line arguments and then call the search function with the initial state and the selected frontier (which implements the search strategy). The search function will return a plan (sequence of joint actions) if a solution is found, or None if no solution exists.

It sends actions back to the server by printing them to standard output (stdout) in the required format. Each joint action is printed as a line where individual agent actions are separated by "|". After sending each joint action, it reads the server's response to ensure synchronization and prevent filling up the stdin buffer. 
"""
