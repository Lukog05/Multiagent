import argparse
import sys
import time
from typing import TextIO

from searchclient import memory
from searchclient.color import Color
from searchclient.frontier import Frontier, FrontierBestFirst, FrontierBFS, FrontierDFS
from searchclient.graphsearch import search
from searchclient.heuristic import HeuristicAStar, HeuristicGreedy, HeuristicPredictabilityAware, HeuristicWeightedAStar
from searchclient.state import State

ENABLE_CASCADE_REORDER = True

ENABLE_ADAPTIVE_CASCADE = False


def _select_mapf_cascade(num_agents: int, free_cells: int, density: float, accessible: int) -> list[str]:
    """
    Select the ordered list of MAPF algorithm names to try based on level features.

    Decision rules:
    # Rule 1: Very few agents in a small space → Joint A* first (optimal, fast)
    # Rule 2: High density (many agents / free cells) → PIBT excels
    # Rule 3: Many agents (>8) → skip Joint A* entirely (state space too large)
    # Rule 4: Default order: PIBT → GreedyMAPF → CBS → CoopA* → JointA* → DFS-CBS → Greedy
    """
    default = ["PIBT", "GreedyMAPF", "CBS", "CoopA*", "JointA*", "DFS-CBS", "Greedy"]

    if num_agents <= 3 and accessible <= 50:
        return ["JointA*", "PIBT", "GreedyMAPF", "CBS", "CoopA*", "DFS-CBS", "Greedy"]

    if density >= 0.3:
        return ["PIBT", "GreedyMAPF", "CBS", "CoopA*", "DFS-CBS", "Greedy"]

    if num_agents > 8:
        return ["PIBT", "GreedyMAPF", "CBS", "CoopA*", "DFS-CBS", "Greedy"]

    return default


class SearchClient:
    @staticmethod
    def parse_level(server_messages: TextIO) -> State:
        server_messages.readline()
        server_messages.readline()

        server_messages.readline()
        server_messages.readline()

        server_messages.readline()
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

        goals = [["" for _ in range(num_cols)] for _ in range(num_rows)]
        line = server_messages.readline()
        row = 0
        while not line.startswith("#"):
            for col, c in enumerate(line):
                if "0" <= c <= "9" or "A" <= c <= "Z":
                    goals[row][col] = c

            row += 1
            line = server_messages.readline()


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
        print(
            "SearchClient initializing. I am sending this using the error output stream.", file=sys.stderr, flush=True
        )

        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="ASCII")
        print("SearchClient", flush=True)

        print("#This is a comment.", flush=True)

        server_messages = sys.stdin
        if hasattr(server_messages, "reconfigure"):
            server_messages.reconfigure(encoding="ASCII")
        initial_state = SearchClient.parse_level(server_messages)

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
            _server_deadline = _cascade_start + max(10.0, args.time - 10.0)
            _total_budget = _server_deadline - _cascade_start

            if not has_boxes:
                from searchclient.cbs import (
                    cbs_search, _count_reachable_cells, _joint_astar,
                    _greedy_mapf, pibt_search, dfs_cbs_search, cooperative_astar,
                )
                num_agents = len(initial_state.agent_rows)

                accessible = _count_reachable_cells(initial_state)
                _rows = len(State.walls)
                _cols = len(State.walls[0]) if _rows > 0 else 0
                free_cells = sum(1 for _r in range(_rows) for _c in range(_cols) if not State.walls[_r][_c])
                density = num_agents / max(1, free_cells)
                if not ENABLE_CASCADE_REORDER:
                    _cascade_order = ["JointA*", "CBS", "GreedyMAPF", "PIBT", "DFS-CBS", "CoopA*", "Greedy"]
                elif ENABLE_ADAPTIVE_CASCADE:
                    _cascade_order = _select_mapf_cascade(num_agents, free_cells, density, accessible)
                else:
                    _cascade_order = ["JointA*", "PIBT", "GreedyMAPF", "CBS", "CoopA*", "DFS-CBS", "Greedy"]
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

            if args.predictable is not False:
                print(f"[predictability] enabled on box cascade, λ={args.predictable}", file=sys.stderr, flush=True)

            from searchclient.ma_planner import decoupled_box_plan
            plan = None
            _num_box_agents = len(initial_state.agent_rows)

            _num_boxes = sum(
                1
                for r in range(len(initial_state.boxes))
                for c in range(len(initial_state.boxes[r]))
                if initial_state.boxes[r][c]
            )

            _state_space_estimate = _num_boxes * _num_box_agents
            _skip_wa = _state_space_estimate >= 100

            if _num_box_agents >= 1:
                if _num_boxes >= 10 or _num_box_agents >= 3:
                    _dec_budget = min(140.0, _server_deadline - time.perf_counter() - 15.0)
                else:
                    _dec_budget = min(90.0, _server_deadline - time.perf_counter() - 30.0)
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

            if plan is None:

                if _skip_wa:
                    print(f"[cascade] Skipping WA* (too large: {_num_boxes} boxes × "
                          f"{_num_box_agents} agents)", file=sys.stderr, flush=True)
                else:
                    _wa_per = max(3.0, _total_budget * 0.09)
                    _wa_min_rem = max(5.0, _total_budget * 0.15)
                    if _state_space_estimate <= 15:
                        _wa_schedule = [
                            (20, max(3.0, _total_budget * 0.06)),
                            (10, max(3.0, _total_budget * 0.07)),
                            (5,  max(3.0, _total_budget * 0.12)),
                            (2,  max(3.0, _total_budget * 0.18)),
                        ]
                    else:
                        _wa_schedule = [
                            (100, _wa_per),
                            (50,  _wa_per),
                            (20,  _wa_per),
                            (10,  _wa_per * 1.3),
                            (5,   _wa_per * 2.0),
                        ]
                    for _w, _bud in _wa_schedule:
                        _remaining = _server_deadline - time.perf_counter()
                        if _remaining < _wa_min_rem:
                            break
                        _bud = min(_bud, _remaining - _wa_min_rem)
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

            if plan is None and not _skip_wa:
                _gbf_budget = max(0.0, _server_deadline - time.perf_counter() - 5.0)
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

            if plan is None:
                _dec_budget2 = _server_deadline - time.perf_counter() - 5.0
                if _dec_budget2 >= 5.0:
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

            _lns_remaining = _server_deadline - time.perf_counter()
            _lns_cap = min(12.0, _lns_remaining - 10.0)
            if _lns_cap >= 10.0 and not _skip_wa:
                from searchclient.lns import lns2_improve
                _lns_deadline = time.perf_counter() + _lns_cap
                _improved = lns2_improve(plan, initial_state, _lns_deadline, is_mapf=False)
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
            return

        print(f"Starting {frontier.get_name()}.", file=sys.stderr, flush=True)
        plan = search(initial_state, frontier)

        if plan is None:
            print("Unable to solve level.", file=sys.stderr, flush=True)
            sys.exit(0)
        else:
            print(f"Found solution of length {len(plan)}.", file=sys.stderr, flush=True)

            for joint_action in plan:
                print("|".join(a.name_ + "@" + a.name_ for a in joint_action), flush=True)
                _response = server_messages.readline()


if __name__ == "__main__":
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
        "--time",
        metavar="<seconds>",
        type=float,
        default=170.0,
        help=(
            "Server time limit in seconds (default 170, matching the 180s server default). "
            "Set this to match the -t argument passed to the server jar so the cascade "
            "allocates budgets correctly (e.g., --time 60 when server uses -t 60)."
        ),
    )

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

    memory.max_usage = args.max_memory

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
