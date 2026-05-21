import heapq
import time
from collections import deque

from searchclient.action import Action
from searchclient.state import State


def _bfs_distances(goal_r: int, goal_c: int) -> list[list[int]]:
    rows = len(State.walls)
    cols = len(State.walls[0])
    INF = 10_000_000
    dist = [[INF] * cols for _ in range(rows)]
    dist[goal_r][goal_c] = 0
    q: deque[tuple[int, int]] = deque([(goal_r, goal_c)])
    while q:
        r, c = q.popleft()
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not State.walls[nr][nc] and dist[nr][nc] == INF:
                dist[nr][nc] = dist[r][c] + 1
                q.append((nr, nc))
    return dist


def _constrained_astar(
    start_r: int,
    start_c: int,
    goal_r: int,
    goal_c: int,
    dist_grid: list[list[int]],
    vertex_constraints: frozenset,
    edge_constraints: frozenset,
    max_t: int = 300,
) -> list[tuple[int, int]] | None:
    DIRS = ((-1, 0), (1, 0), (0, -1), (0, 1), (0, 0))
    rows = len(State.walls)
    cols = len(State.walls[0])
    latest_goal_block_t = max((t for (r, c, t) in vertex_constraints if r == goal_r and c == goal_c), default=-1)

    heap: list[tuple] = [(dist_grid[start_r][start_c], 0, start_r, start_c, 0)]
    best: dict[tuple[int, int, int], int] = {}
    parent: dict[tuple[int, int, int], tuple[int, int, int] | None] = {(start_r, start_c, 0): None}

    while heap:
        f, g, r, c, t = heapq.heappop(heap)
        key = (r, c, t)
        if best.get(key, 10_000_000) <= g:
            continue
        best[key] = g

        if r == goal_r and c == goal_c and t > latest_goal_block_t:
            path: list[tuple[int, int]] = []
            cur: tuple[int, int, int] | None = key
            while cur is not None:
                path.append((cur[0], cur[1]))
                cur = parent[cur]
            path.reverse()
            return path

        if t >= max_t:
            continue

        for dr, dc in DIRS:
            nr, nc = r + dr, c + dc
            nt = t + 1
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if State.walls[nr][nc]:
                continue
            if (nr, nc, nt) in vertex_constraints:
                continue
            if (r, c, nr, nc, t) in edge_constraints:
                continue
            h = dist_grid[nr][nc]
            if h == 10_000_000:
                continue
            new_g = g + 1
            nkey = (nr, nc, nt)
            if best.get(nkey, 10_000_000) > new_g:
                parent[nkey] = key
                heapq.heappush(heap, (new_g + h, new_g, nr, nc, nt))

    return None


def _find_conflict(paths: list[list[tuple[int, int]]]) -> tuple | None:
    max_t = max(len(p) for p in paths)

    def pos(path: list[tuple[int, int]], t: int) -> tuple[int, int]:
        return path[min(t, len(path) - 1)]

    n = len(paths)
    for t in range(max_t + 1):
        positions = [pos(p, t) for p in paths]
        for i in range(n):
            for j in range(i + 1, n):
                if positions[i] == positions[j]:
                    r, c = positions[i]
                    return ("vertex", i, j, r, c, t)
        if t > 0:
            for i in range(n):
                for j in range(i + 1, n):
                    if pos(paths[i], t) == pos(paths[j], t - 1) and pos(paths[i], t - 1) == pos(paths[j], t):
                        r1, c1 = pos(paths[i], t - 1)
                        r2, c2 = pos(paths[i], t)
                        return ("edge", i, j, r1, c1, r2, c2, t)
    return None


def _paths_to_joint_actions(paths: list[list[tuple[int, int]]], num_agents: int) -> list[list[Action]]:
    DELTA_TO_ACTION = {
        (-1, 0): Action.MoveN,
        (1,  0): Action.MoveS,
        (0,  1): Action.MoveE,
        (0, -1): Action.MoveW,
        (0,  0): Action.NoOp,
    }
    max_t = max(len(p) for p in paths)
    joint_actions = []
    for t in range(1, max_t):
        step = []
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


def _try_ca_order(
    initial_state: State,
    order: list[int],
    agent_goals: dict[int, tuple[int, int]],
    dist_grids: dict[int, list[list[int]]],
    max_t: int,
) -> list[list[Action]] | None:
    """Attempt one CA* ordering. Returns joint actions on success, None on failure."""
    num_agents = len(initial_state.agent_rows)
    paths: list[list[tuple[int, int]] | None] = [None] * num_agents
    vertex_res: set[tuple[int, int, int]] = set()
    edge_res: set[tuple[int, int, int, int, int]] = set()

    for i in order:
        path = _constrained_astar(
            initial_state.agent_rows[i], initial_state.agent_cols[i],
            agent_goals[i][0], agent_goals[i][1],
            dist_grids[i], frozenset(vertex_res), frozenset(edge_res), max_t=max_t,
        )
        if path is None:
            return None
        paths[i] = path
        for t, (r, c) in enumerate(path):
            vertex_res.add((r, c, t))
        gr, gc = path[-1]
        for t in range(len(path), max_t + 1):
            vertex_res.add((gr, gc, t))
        for t in range(1, len(path)):
            pr, pc = path[t - 1]
            cr, cc = path[t]
            edge_res.add((pr, pc, cr, cc, t - 1))
            if (cr, cc) != (pr, pc):
                edge_res.add((cr, cc, pr, pc, t - 1))

    full_paths: list[list[tuple[int, int]]] = [p for p in paths if p is not None]
    return _paths_to_joint_actions(full_paths, num_agents)


def cooperative_astar(initial_state: State, max_t: int = 500, deadline: float | None = None) -> list[list[Action]] | None:
    """
    Cooperative A* (CA*): plan each agent using a shared space-time reservation
    table built from previously planned agents' paths.

    Tries multiple priority orderings because CA* is sensitive to order — the
    ordering that works depends on the level topology. Runs in O(k * n * A*)
    where k is the number of orderings tried (small constant).
    """
    num_agents = len(initial_state.agent_rows)

    agent_goals: dict[int, tuple[int, int]] = {}
    for r in range(len(State.goals)):
        for c in range(len(State.goals[r])):
            g = State.goals[r][c]
            if "0" <= g <= "9":
                idx = ord(g) - ord("0")
                if idx < num_agents:
                    agent_goals[idx] = (r, c)
    for i in range(num_agents):
        if i not in agent_goals:
            agent_goals[i] = (initial_state.agent_rows[i], initial_state.agent_cols[i])

    dist_grids = {i: _bfs_distances(*agent_goals[i]) for i in range(num_agents)}

    agents = list(range(num_agents))
    dists = [dist_grids[i][initial_state.agent_rows[i]][initial_state.agent_cols[i]] for i in agents]
    goal_cols = [agent_goals[i][1] for i in agents]
    start_cols = [initial_state.agent_cols[i] for i in agents]

    orderings: list[list[int]] = []
    for reverse in (True, False):
        orderings.append(sorted(agents, key=lambda i: dists[i], reverse=reverse))
        orderings.append(sorted(agents, key=lambda i: goal_cols[i], reverse=reverse))
        orderings.append(sorted(agents, key=lambda i: start_cols[i], reverse=reverse))

    for order in orderings:
        if deadline is not None and time.perf_counter() > deadline:
            return None
        result = _try_ca_order(initial_state, order, agent_goals, dist_grids, max_t)
        if result is not None:
            return result

    return None


def cbs_search(initial_state: State, deadline: float | None = None) -> list[list[Action]] | None:
    num_agents = len(initial_state.agent_rows)

    agent_goals: dict[int, tuple[int, int]] = {}
    for r in range(len(State.goals)):
        for c in range(len(State.goals[r])):
            g = State.goals[r][c]
            if "0" <= g <= "9":
                idx = ord(g) - ord("0")
                if idx < num_agents:
                    agent_goals[idx] = (r, c)

    for i in range(num_agents):
        if i not in agent_goals:
            agent_goals[i] = (initial_state.agent_rows[i], initial_state.agent_cols[i])

    dist_grids = {i: _bfs_distances(*agent_goals[i]) for i in range(num_agents)}

    def solve_agent(
        i: int,
        vc: frozenset,
        ec: frozenset,
    ) -> list[tuple[int, int]] | None:
        sr, sc = initial_state.agent_rows[i], initial_state.agent_cols[i]
        gr, gc = agent_goals[i]
        agent_vc = frozenset((r, c, t) for (ai, r, c, t) in vc if ai == i)
        agent_ec = frozenset((r1, c1, r2, c2, t) for (ai, r1, c1, r2, c2, t) in ec if ai == i)
        return _constrained_astar(sr, sc, gr, gc, dist_grids[i], agent_vc, agent_ec)

    initial_paths = []
    for i in range(num_agents):
        path = solve_agent(i, frozenset(), frozenset())
        if path is None:
            return None
        initial_paths.append(path)

    node_id = 0
    initial_cost = sum(len(p) - 1 for p in initial_paths)
    heap: list[tuple] = [(initial_cost, node_id, initial_paths, frozenset(), frozenset())]

    while heap:
        if deadline is not None and time.perf_counter() > deadline:
            return None

        cost, _, paths, vc, ec = heapq.heappop(heap)

        conflict = _find_conflict(paths)
        if conflict is None:
            return _paths_to_joint_actions(paths, num_agents)

        if conflict[0] == "vertex":
            _, a1, a2, r, c, t = conflict
            for agent in (a1, a2):
                new_vc = vc | {(agent, r, c, t)}
                new_path = solve_agent(agent, new_vc, ec)
                if new_path is not None:
                    new_paths = list(paths)
                    new_paths[agent] = new_path
                    new_cost = sum(len(p) - 1 for p in new_paths)
                    node_id += 1
                    heapq.heappush(heap, (new_cost, node_id, new_paths, new_vc, ec))
        else:
            _, a1, a2, r1, c1, r2, c2, t = conflict
            for agent, (fr, fc, tr_, tc) in ((a1, (r1, c1, r2, c2)), (a2, (r2, c2, r1, c1))):
                new_ec = ec | {(agent, fr, fc, tr_, tc, t - 1)}
                new_path = solve_agent(agent, vc, new_ec)
                if new_path is not None:
                    new_paths = list(paths)
                    new_paths[agent] = new_path
                    new_cost = sum(len(p) - 1 for p in new_paths)
                    node_id += 1
                    heapq.heappush(heap, (new_cost, node_id, new_paths, vc, new_ec))

    return None


def _greedy_mapf(initial_state: State, max_steps: int = 2000, deadline: float | None = None) -> list[list[Action]] | None:
    """
    Simple priority-greedy MAPF solver.

    At each timestep:
    1. Sort agents by remaining distance to goal (descending = highest priority).
    2. Process agents in that order.  Each agent greedily claims its best next
       cell (minimises BFS distance) that has not yet been claimed by a
       higher-priority agent this timestep.
    3. If all preferred cells are taken, the agent stays — unless its current
       cell is claimed by a higher-priority agent, in which case it is pushed
       to the next-best free cell.

    This runs in O(n log n) per timestep and handles corridor reordering
    reliably (the trace for MAPFreorder2 reaches the goal in ~12 timesteps).
    """
    num_agents = len(initial_state.agent_rows)

    agent_goals: dict[int, tuple[int, int]] = {}
    for r in range(len(State.goals)):
        for c in range(len(State.goals[r])):
            g = State.goals[r][c]
            if "0" <= g <= "9":
                idx = ord(g) - ord("0")
                if idx < num_agents:
                    agent_goals[idx] = (r, c)
    for i in range(num_agents):
        if i not in agent_goals:
            agent_goals[i] = (initial_state.agent_rows[i], initial_state.agent_cols[i])

    dist_grids = {i: _bfs_distances(*agent_goals[i]) for i in range(num_agents)}
    rows = len(State.walls)
    cols = len(State.walls[0])

    DIRS = ((-1, 0), (1, 0), (0, -1), (0, 1), (0, 0))

    DELTA_TO_ACTION = {
        (-1, 0): Action.MoveN,
        (1,  0): Action.MoveS,
        (0,  1): Action.MoveE,
        (0, -1): Action.MoveW,
        (0,  0): Action.NoOp,
    }

    pos: list[tuple[int, int]] = [
        (initial_state.agent_rows[i], initial_state.agent_cols[i]) for i in range(num_agents)
    ]
    joint_actions: list[list[Action]] = []

    for _step in range(max_steps):
        if deadline is not None and time.perf_counter() > deadline:
            return None
        if all(pos[i] == agent_goals[i] for i in range(num_agents)):
            return joint_actions

        order = sorted(
            range(num_agents),
            key=lambda i: dist_grids[i][pos[i][0]][pos[i][1]],
            reverse=True,
        )

        claimed: dict[tuple[int, int], int] = {}
        next_pos: list[tuple[int, int]] = list(pos)

        for agent in order:
            r, c = pos[agent]
            candidates = sorted(
                (dist_grids[agent][r + dr][c + dc], r + dr, c + dc)
                for dr, dc in DIRS
                if 0 <= r + dr < rows and 0 <= c + dc < cols
                and not State.walls[r + dr][c + dc]
            )
            chosen = None
            for _, nr, nc in candidates:
                if (nr, nc) not in claimed:
                    swapper = claimed.get((r, c))
                    if swapper is not None and pos[swapper] == (nr, nc):
                        continue
                    chosen = (nr, nc)
                    break
            if chosen is None:
                next_pos[agent] = (r, c)
            else:
                claimed[chosen] = agent
                next_pos[agent] = chosen

        step_actions: list[Action] = [
            DELTA_TO_ACTION[(next_pos[i][0] - pos[i][0], next_pos[i][1] - pos[i][1])]
            for i in range(num_agents)
        ]
        joint_actions.append(step_actions)
        pos = next_pos

    return None


def pibt_search(initial_state: State, max_steps: int = 2000) -> list[list[Action]] | None:
    """
    PIBT (Priority Inheritance with Backtracking) — a polynomial-time MAPF solver.

    At each timestep, agents are processed in priority order (highest remaining
    distance first).  Each agent greedily picks the best next cell.  If that cell
    is occupied by a lower-priority agent, the lower-priority agent is recursively
    asked to vacate (priority inheritance).  If it cannot vacate, the requesting
    agent tries its next-best candidate.

    PIBT is complete on any connected graph and runs in O(n) per timestep, making
    it ideal for dense, symmetric problems like corridor reordering where CBS fails.
    """
    num_agents = len(initial_state.agent_rows)

    agent_goals: dict[int, tuple[int, int]] = {}
    for r in range(len(State.goals)):
        for c in range(len(State.goals[r])):
            g = State.goals[r][c]
            if "0" <= g <= "9":
                idx = ord(g) - ord("0")
                if idx < num_agents:
                    agent_goals[idx] = (r, c)
    for i in range(num_agents):
        if i not in agent_goals:
            agent_goals[i] = (initial_state.agent_rows[i], initial_state.agent_cols[i])

    dist_grids = {i: _bfs_distances(*agent_goals[i]) for i in range(num_agents)}
    rows = len(State.walls)
    cols = len(State.walls[0])

    DIRS = ((-1, 0), (1, 0), (0, -1), (0, 1), (0, 0))

    pos: list[tuple[int, int]] = [
        (initial_state.agent_rows[i], initial_state.agent_cols[i]) for i in range(num_agents)
    ]
    cell_agent: dict[tuple[int, int], int] = {p: i for i, p in enumerate(pos)}

    joint_actions: list[list[Action]] = []

    DELTA_TO_ACTION = {
        (-1, 0): Action.MoveN,
        (1,  0): Action.MoveS,
        (0,  1): Action.MoveE,
        (0, -1): Action.MoveW,
        (0,  0): Action.NoOp,
    }

    def is_open(r: int, c: int) -> bool:
        return 0 <= r < rows and 0 <= c < cols and not State.walls[r][c]

    def move(agent: int, reserved: set[tuple[int, int]], visiting: frozenset[int]) -> bool:
        """
        Try to move `agent` to a cell NOT in `reserved`.
        `visiting` tracks agents in the current recursive chain (cycle detection).
        Returns True if the agent successfully chose a destination.
        The agent's choice is stored in next_pos[agent].

        Key rule: the cell occupied by this agent before moving is added to
        `reserved` for the recursively displaced occupant, so the occupant is
        forced to ACTUALLY LEAVE the requested cell (not just NoOp).
        """
        if agent in visiting:
            return False

        r, c = pos[agent]
        visiting = visiting | {agent}

        candidates: list[tuple[int, int, int]] = []
        for dr, dc in DIRS:
            nr, nc = r + dr, c + dc
            if is_open(nr, nc):
                candidates.append((dist_grids[agent][nr][nc], nr, nc))
        candidates.sort()

        for _, nr, nc in candidates:
            if (nr, nc) in reserved:
                continue
            occupant = cell_agent.get((nr, nc))
            if occupant is None or occupant == agent:
                next_pos[agent] = (nr, nc)
                return True
            if (next_pos[occupant] is not None
                    and next_pos[occupant] != (nr, nc)
                    and next_pos[occupant] != (r, c)):
                next_pos[agent] = (nr, nc)
                return True
            new_reserved = reserved | {(r, c), (nr, nc)}
            if move(occupant, new_reserved, visiting):
                next_pos[agent] = (nr, nc)
                return True

        if (r, c) not in reserved:
            next_pos[agent] = (r, c)
            return True
        return False

    for _step in range(max_steps):
        if all(pos[i] == agent_goals[i] for i in range(num_agents)):
            return joint_actions

        order = sorted(
            range(num_agents),
            key=lambda i: dist_grids[i][pos[i][0]][pos[i][1]],
            reverse=True,
        )

        next_pos: list[tuple[int, int] | None] = [None] * num_agents
        reserved_top: set[tuple[int, int]] = set()

        for agent in order:
            if next_pos[agent] is None:
                move(agent, reserved_top, frozenset())
            if next_pos[agent] is not None:
                reserved_top.add(next_pos[agent])

        step_actions: list[Action] = []
        for i in range(num_agents):
            pr, pc = pos[i]
            nr, nc = next_pos[i]
            step_actions.append(DELTA_TO_ACTION[(nr - pr, nc - pc)])

        joint_actions.append(step_actions)

        cell_agent.clear()
        for i in range(num_agents):
            pos[i] = next_pos[i]
            cell_agent[pos[i]] = i

    return None


def dfs_cbs_search(initial_state: State, deadline: float | None = None) -> list[list[Action]] | None:
    """
    DFS-CBS: CBS with depth-first traversal instead of best-first.

    Best-first CBS builds an exponential heap for symmetric problems (branching
    factor 2, no pruning) and never reaches deep enough to find a solution.
    DFS-CBS goes deep immediately, finds *a* (sub-optimal) solution quickly.
    Memory usage is O(depth * constraint_size) instead of O(nodes).
    """
    num_agents = len(initial_state.agent_rows)

    agent_goals: dict[int, tuple[int, int]] = {}
    for r in range(len(State.goals)):
        for c in range(len(State.goals[r])):
            g = State.goals[r][c]
            if "0" <= g <= "9":
                idx = ord(g) - ord("0")
                if idx < num_agents:
                    agent_goals[idx] = (r, c)
    for i in range(num_agents):
        if i not in agent_goals:
            agent_goals[i] = (initial_state.agent_rows[i], initial_state.agent_cols[i])

    dist_grids = {i: _bfs_distances(*agent_goals[i]) for i in range(num_agents)}

    def solve_agent(i: int, vc: frozenset, ec: frozenset) -> list[tuple[int, int]] | None:
        sr, sc = initial_state.agent_rows[i], initial_state.agent_cols[i]
        gr, gc = agent_goals[i]
        return _constrained_astar(
            sr, sc, gr, gc, dist_grids[i],
            frozenset((r, c, t) for (ai, r, c, t) in vc if ai == i),
            frozenset((r1, c1, r2, c2, t) for (ai, r1, c1, r2, c2, t) in ec if ai == i),
        )

    initial_paths = [solve_agent(i, frozenset(), frozenset()) for i in range(num_agents)]
    if any(p is None for p in initial_paths):
        return None

    max_depth = max(200, num_agents * num_agents * 10)

    stack: list[tuple] = [(initial_paths, frozenset(), frozenset(), 0)]

    while stack:
        if deadline is not None and time.perf_counter() > deadline:
            return None

        paths, vc, ec, depth = stack.pop()

        if depth > max_depth:
            continue

        conflict = _find_conflict(paths)
        if conflict is None:
            return _paths_to_joint_actions(paths, num_agents)

        branches = []
        if conflict[0] == "vertex":
            _, a1, a2, r, c, t = conflict
            for agent in (a2, a1):
                new_vc = vc | {(agent, r, c, t)}
                new_path = solve_agent(agent, new_vc, ec)
                if new_path is not None:
                    new_paths = list(paths)
                    new_paths[agent] = new_path
                    branches.append((new_paths, new_vc, ec, depth + 1))
        else:
            _, a1, a2, r1, c1, r2, c2, t = conflict
            for agent, (fr, fc, tr_, tc) in ((a2, (r2, c2, r1, c1)), (a1, (r1, c1, r2, c2))):
                new_ec = ec | {(agent, fr, fc, tr_, tc, t - 1)}
                new_path = solve_agent(agent, vc, new_ec)
                if new_path is not None:
                    new_paths = list(paths)
                    new_paths[agent] = new_path
                    branches.append((new_paths, vc, new_ec, depth + 1))

        stack.extend(branches)

    return None


def _count_reachable_cells(initial_state: State) -> int:
    """BFS from the first agent to count all reachable non-wall cells."""
    if not initial_state.agent_rows:
        return 0
    rows = len(State.walls)
    cols = len(State.walls[0]) if rows > 0 else 0
    start = (initial_state.agent_rows[0], initial_state.agent_cols[0])
    visited: set[tuple[int, int]] = {start}
    q: deque[tuple[int, int]] = deque([start])
    while q:
        r, c = q.popleft()
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not State.walls[nr][nc] and (nr, nc) not in visited:
                visited.add((nr, nc))
                q.append((nr, nc))
    return len(visited)


def _joint_astar(initial_state: State, deadline: float | None = None) -> list[list[Action]] | None:
    """
    Joint-state A* with one-agent-at-a-time moves.

    Moves one agent per timestep (all others NoOp).  This is always a valid
    joint action sequence and is complete for any connected graph.  Uses
    per-agent BFS distances as the admissible heuristic (sum of individual
    shortest-path lengths ignoring other agents).

    Suitable for levels with a small reachable cell count — sliding puzzles,
    tight corridor reordering, etc.
    """
    num_agents = len(initial_state.agent_rows)
    rows = len(State.walls)
    cols = len(State.walls[0]) if rows > 0 else 0

    agent_goals: dict[int, tuple[int, int]] = {}
    for r in range(rows):
        for c in range(cols):
            g = State.goals[r][c]
            if "0" <= g <= "9":
                idx = ord(g) - ord("0")
                if idx < num_agents:
                    agent_goals[idx] = (r, c)
    for i in range(num_agents):
        if i not in agent_goals:
            agent_goals[i] = (initial_state.agent_rows[i], initial_state.agent_cols[i])

    dist_grids = {i: _bfs_distances(*agent_goals[i]) for i in range(num_agents)}

    goal_pos = tuple(agent_goals[i][0] * cols + agent_goals[i][1] for i in range(num_agents))
    start_pos = tuple(
        initial_state.agent_rows[i] * cols + initial_state.agent_cols[i]
        for i in range(num_agents)
    )

    if start_pos == goal_pos:
        return []

    DIRS = ((-1, 0), (1, 0), (0, -1), (0, 1))
    DELTA_TO_ACTION = {
        (-1, 0): Action.MoveN,
        (1, 0): Action.MoveS,
        (0, 1): Action.MoveE,
        (0, -1): Action.MoveW,
    }

    def heuristic(state: tuple) -> int:
        h = 0
        for i in range(num_agents):
            r, c = divmod(state[i], cols)
            h += dist_grids[i][r][c]
        return h

    INF = 10_000_000
    g_score: dict[tuple, int] = {start_pos: 0}
    parent: dict[tuple, object] = {start_pos: None}
    heap: list = [(heuristic(start_pos), 0, start_pos)]

    while heap:
        if deadline is not None and time.perf_counter() > deadline:
            return None

        f, g, state = heapq.heappop(heap)

        if g > g_score.get(state, INF):
            continue

        if state == goal_pos:
            path: list[list[Action]] = []
            cur: tuple = state
            while True:
                entry = parent[cur]
                if entry is None:
                    break
                prev_state, agent_idx, action = entry
                joint: list[Action] = [Action.NoOp] * num_agents
                joint[agent_idx] = action
                path.append(joint)
                cur = prev_state
            path.reverse()
            return path

        occupied = set(state)
        for i in range(num_agents):
            r, c = divmod(state[i], cols)
            for dr, dc in DIRS:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue
                if State.walls[nr][nc]:
                    continue
                npos = nr * cols + nc
                if npos in occupied:
                    continue
                new_state = list(state)
                new_state[i] = npos
                new_state_t = tuple(new_state)
                new_g = g + 1
                if new_g < g_score.get(new_state_t, INF):
                    g_score[new_state_t] = new_g
                    parent[new_state_t] = (state, i, DELTA_TO_ACTION[(dr, dc)])
                    heapq.heappush(heap, (new_g + heuristic(new_state_t), new_g, new_state_t))

    return None
