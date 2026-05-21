
from __future__ import annotations

from collections import deque

from searchclient.state import State

def _bfs_path(start_r: int, start_c: int, goal_r: int, goal_c: int) -> list[tuple[int, int]]:
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

    path: list[tuple[int, int]] = [(start_r, start_c)]
    r, c = start_r, start_c
    visited: set[tuple[int, int]] = {(r, c)}
    while (r, c) != (goal_r, goal_c) and len(path) <= rows * cols:
        best: tuple[int, int] | None = None
        best_d = dist[r][c]
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = r + dr, c + dc
            if (
                0 <= nr < rows
                and 0 <= nc < cols
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

class PredictabilityModel:

    def __init__(
        self,
        initial_state: State,
        lambda_: float = 2.5,
        gamma: float = 0.6,
    ) -> None:
        self.lambda_ = lambda_
        self.gamma = gamma
        self._paths: list[list[tuple[int, int]]] = []

        agent_goals: dict[int, tuple[int, int]] = {}
        for row in range(len(State.goals)):
            for col in range(len(State.goals[row])):
                g = State.goals[row][col]
                if "0" <= g <= "9":
                    agent_goals[ord(g) - ord("0")] = (row, col)

        for i in range(len(initial_state.agent_rows)):
            sr, sc = initial_state.agent_rows[i], initial_state.agent_cols[i]
            if i in agent_goals:
                gr, gc = agent_goals[i]
                path = _bfs_path(sr, sc, gr, gc)
            else:
                path = [(sr, sc)]
            self._paths.append(path)

    def penalty(self, state: State) -> float:
        t = state.g
        if self.lambda_ == 0.0:
            return 0.0
        discount = self.gamma ** t
        if discount < 1e-9:
            return 0.0

        total = 0.0
        for i in range(len(state.agent_rows)):
            path = self._paths[i]
            pr, pc = path[min(t, len(path) - 1)]
            dr = state.agent_rows[i] - pr
            dc = state.agent_cols[i] - pc
            total += dr * dr + dc * dc

        return self.lambda_ * discount * total
