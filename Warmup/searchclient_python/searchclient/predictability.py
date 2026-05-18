"""
Predictability-aware cost penalty (arXiv:2411.06223v2).

The paper's cost function (Eq. 5):
    J(τ) = Σ_k [ J_k(x_k, u_k) + γ^k · λ · KL(q(x_k) ‖ p(x_k)) ]

Adaptation for discrete grid worlds
────────────────────────────────────
• Prediction model p  : BFS shortest path from each agent's initial
                        position toward its goal — the discrete analogue
                        of a constant-velocity predictor.
• Plan distribution q : the agent's actual position at step t (point mass).
• KL divergence       : for equal-variance Gaussians,
                        KL(N(μ_q,σ²) ‖ N(μ_p,σ²)) = ‖μ_q−μ_p‖² / (2σ²).
                        With σ normalised to 1 this reduces to squared
                        Euclidean distance between actual and predicted pos.
• Penalty at step t   : λ · γ^t · Σ_i  ‖actual_i − predicted_i(t)‖²

The penalty is evaluated inside the heuristic so it participates in the
frontier's priority and biases the search toward more "predictable" paths
without modifying the graph-search loop itself.
"""

from __future__ import annotations

from collections import deque

from searchclient.state import State


def _bfs_path(start_r: int, start_c: int, goal_r: int, goal_c: int) -> list[tuple[int, int]]:
    """Return the BFS shortest path from (start_r, start_c) to (goal_r, goal_c)."""
    rows = len(State.walls)
    cols = len(State.walls[0])
    INF = 10_000_000

    # Backward BFS from goal to get distance-to-goal for every cell.
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

    # Trace forward from start by greedily moving to the neighbour with the
    # smallest BFS distance (ties broken by iteration order).
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
            break  # goal unreachable
        r, c = best
        visited.add((r, c))
        path.append((r, c))

    return path


class PredictabilityModel:
    """
    Evaluates the predictability penalty term from arXiv:2411.06223v2 (Eq. 5).

    Parameters
    ----------
    initial_state : State
        The level's initial state — used to set up per-agent predicted paths.
    lambda_ : float
        Weight on the KL-divergence term (λ in the paper).
        Paper experiments used λ ∈ {0, 2.5, 5.0}; start with 2.5.
        Larger values push agents to follow the predicted path more strictly.
    gamma : float
        Horizon discount factor (γ in the paper, default 0.6).
        Downweights the predictability cost for later time steps, reflecting
        greater uncertainty about long-range predictions.
    """

    def __init__(
        self,
        initial_state: State,
        lambda_: float = 2.5,
        gamma: float = 0.6,
    ) -> None:
        self.lambda_ = lambda_
        self.gamma = gamma
        self._paths: list[list[tuple[int, int]]] = []

        # Map agent index → goal position (only agents that have explicit goals).
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
                # No explicit goal → prediction model expects the agent to
                # remain at its initial position (zero-velocity prediction).
                path = [(sr, sc)]
            self._paths.append(path)

    def penalty(self, state: State) -> float:
        """
        Compute the predictability penalty for *state*.

        Returns λ · γ^t · Σ_i ‖actual_i − predicted_i(t)‖²
        where t = state.g (number of steps taken so far).
        """
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
            total += dr * dr + dc * dc  # squared Euclidean ≈ KL for equal-σ Gaussians

        return self.lambda_ * discount * total
