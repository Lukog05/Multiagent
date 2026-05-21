from abc import ABC, abstractmethod
from collections import deque

from searchclient.state import State

ENABLE_HUNGARIAN_HEURISTIC = True

def _hungarian(cost_matrix: list[list[int]]) -> list[int]:
    """
    Solve the assignment problem using the O(n^3) Hungarian algorithm.
    Given an n×m cost matrix (n ≤ m), returns a list `assign` of length n
    where assign[i] is the column assigned to row i, minimising total cost.
    """
    n = len(cost_matrix)
    if n == 0:
        return []
    m = len(cost_matrix[0])
    size = max(n, m)
    INF = 10_000_000
    a = [[cost_matrix[i][j] if i < n and j < m else 0 for j in range(size)] for i in range(size)]

    u = [0] * (size + 1)
    v = [0] * (size + 1)
    p = [0] * (size + 1)
    way = [0] * (size + 1)

    for i in range(1, size + 1):
        p[0] = i
        j0 = 0
        minval = [INF] * (size + 1)
        used = [False] * (size + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = INF
            j1 = -1
            for j in range(1, size + 1):
                if not used[j]:
                    cur = a[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minval[j]:
                        minval[j] = cur
                        way[j] = j0
                    if minval[j] < delta:
                        delta = minval[j]
                        j1 = j
            for j in range(size + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minval[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            p[j0] = p[way[j0]]
            j0 = way[j0]

    assign = [0] * n
    for j in range(1, size + 1):
        if 1 <= p[j] <= n:
            assign[p[j] - 1] = j - 1
    return assign


class Heuristic(ABC):
    def __init__(self, initial_state: State) -> None:
        self.box_goals: dict[str, list[tuple[int, int]]] = {}
        self.agent_goals: dict[int, tuple[int, int]] = {}
        self.agent_goal_distances: dict[int, list[list[int]]] = {}

        for row in range(len(State.goals)):
            for col in range(len(State.goals[row])):
                g = State.goals[row][col]
                if "A" <= g <= "Z":
                    self.box_goals.setdefault(g, []).append((row, col))
                elif "0" <= g <= "9":
                    self.agent_goals[ord(g) - ord("0")] = (row, col)

        self.goal_distances: dict[tuple[int, int], list[list[int]]] = {}
        for goals in self.box_goals.values():
            for goal_pos in goals:
                if goal_pos not in self.goal_distances:
                    self.goal_distances[goal_pos] = self._bfs_distances(*goal_pos)

        for agent_idx, (goal_r, goal_c) in self.agent_goals.items():
            self.agent_goal_distances[agent_idx] = self._bfs_distances(goal_r, goal_c)

    @staticmethod
    def _bfs_distances(goal_r: int, goal_c: int) -> list[list[int]]:
        rows = len(State.walls)
        cols = len(State.walls[0])
        INF = 10_000_000
        dist = [[INF] * cols for _ in range(rows)]
        dist[goal_r][goal_c] = 0
        queue: deque[tuple[int, int]] = deque([(goal_r, goal_c)])
        while queue:
            r, c = queue.popleft()
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and not State.walls[nr][nc] and dist[nr][nc] == INF:
                    dist[nr][nc] = dist[r][c] + 1
                    queue.append((nr, nc))
        return dist

    def h(self, state: State) -> int:
        total = 0

        for letter, goals in self.box_goals.items():
            letter_idx = ord(letter) - ord("A")
            letter_color = State.box_colors[letter_idx]

            box_positions = [
                (br, bc)
                for br in range(len(state.boxes))
                for bc in range(len(state.boxes[br]))
                if state.boxes[br][bc] == letter
            ]
            if not box_positions:
                continue

            unsatisfied = [(gr, gc) for gr, gc in goals if state.boxes[gr][gc] != letter]
            if not unsatisfied:
                continue

            n_goals = len(unsatisfied)
            n_boxes = len(box_positions)

            if n_goals == 1 or n_boxes == 1:
                for gr, gc in unsatisfied:
                    dist_grid = self.goal_distances[(gr, gc)]
                    min_box_dist = min(dist_grid[br][bc] for br, bc in box_positions)
                    total += min_box_dist
                    best_br, best_bc = min(
                        box_positions, key=lambda p: dist_grid[p[0]][p[1]]
                    )
                    if any(State.agent_colors[ai] == letter_color for ai in range(len(state.agent_rows))):
                        min_agent = min(
                            abs(state.agent_rows[ai] - best_br) + abs(state.agent_cols[ai] - best_bc)
                            for ai in range(len(state.agent_rows))
                            if State.agent_colors[ai] == letter_color
                        )
                        total += max(0, min_agent - 1)
            else:
                cost = [
                    [self.goal_distances[(gr, gc)][br][bc] for br, bc in box_positions]
                    for gr, gc in unsatisfied
                ]
                if ENABLE_HUNGARIAN_HEURISTIC and n_goals <= n_boxes:
                    assignment = _hungarian(cost)
                    for i, (gr, gc) in enumerate(unsatisfied):
                        br, bc = box_positions[assignment[i]]
                        total += cost[i][assignment[i]]
                        if any(State.agent_colors[ai] == letter_color for ai in range(len(state.agent_rows))):
                            min_agent = min(
                                abs(state.agent_rows[ai] - br) + abs(state.agent_cols[ai] - bc)
                                for ai in range(len(state.agent_rows))
                                if State.agent_colors[ai] == letter_color
                            )
                            total += max(0, min_agent - 1)
                else:
                    for i, (gr, gc) in enumerate(unsatisfied):
                        dist_grid = self.goal_distances[(gr, gc)]
                        min_box_dist = min(dist_grid[br][bc] for br, bc in box_positions)
                        total += min_box_dist
                        best_br, best_bc = min(box_positions, key=lambda p: dist_grid[p[0]][p[1]])
                        if any(State.agent_colors[ai] == letter_color for ai in range(len(state.agent_rows))):
                            min_agent = min(
                                abs(state.agent_rows[ai] - best_br) + abs(state.agent_cols[ai] - best_bc)
                                for ai in range(len(state.agent_rows))
                                if State.agent_colors[ai] == letter_color
                            )
                            total += max(0, min_agent - 1)

        for agent_idx, (goal_row, goal_col) in self.agent_goals.items():
            if agent_idx < len(state.agent_rows):
                dist_grid = self.agent_goal_distances[agent_idx]
                total += dist_grid[state.agent_rows[agent_idx]][state.agent_cols[agent_idx]]

        return total

    @abstractmethod
    def f(self, state: State) -> int: ...

    @abstractmethod
    def __repr__(self) -> str: ...


class HeuristicAStar(Heuristic):
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return state.g + self.h(state)

    def __repr__(self) -> str:
        return "A* evaluation"


class HeuristicWeightedAStar(Heuristic):
    def __init__(self, initial_state: State, w: int) -> None:
        super().__init__(initial_state)
        self.w = w

    def f(self, state: State) -> int:
        return state.g + self.w * self.h(state)

    def __repr__(self) -> str:
        return f"WA*({self.w}) evaluation"


class HeuristicGreedy(Heuristic):
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return self.h(state)

    def __repr__(self) -> str:
        return "greedy evaluation"


class HeuristicPredictabilityAware:
    """
    Wraps any heuristic and adds the predictability penalty from
    arXiv:2411.06223v2 (Eq. 5).

    Total priority: f_base(state) + λ · γ^t · Σ_i ‖actual_i − predicted_i(t)‖²

    The penalty biases best-first search toward trajectories that stay close
    to the BFS-predicted path (constant-velocity prediction toward each
    agent's goal), fostering the 'soft social convention' described in the
    paper without requiring explicit inter-agent communication.

    Parameters
    ----------
    base : Heuristic
        Any existing heuristic (A*, WA*, Greedy, …) that provides f(state).
    lambda_ : float
        Predictability weight λ.  Paper experiments: {0, 2.5, 5.0}.
    gamma : float
        Horizon discount factor γ (paper default: 0.6).
    """

    def __init__(
        self,
        initial_state: State,
        base: Heuristic,
        lambda_: float = 2.5,
        gamma: float = 0.6,
    ) -> None:
        from searchclient.predictability import PredictabilityModel
        self._base = base
        self._pred = PredictabilityModel(initial_state, lambda_=lambda_, gamma=gamma)

    def f(self, state: State) -> float:
        return self._base.f(state) + self._pred.penalty(state)

    def __repr__(self) -> str:
        return (
            f"predictability-aware {self._base!r} "
            f"(λ={self._pred.lambda_}, γ={self._pred.gamma})"
        )
