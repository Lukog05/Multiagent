from abc import ABC, abstractmethod
from collections import deque

from searchclient.state import State


# BASE HEURISTIC CLASS

class Heuristic(ABC):
    """
    Abstract base class for heuristics.
    Subclasses must implement h() to define the heuristic function.
    """
    def __init__(self, initial_state: State) -> None:
        # Find all agent goals (used by all heuristic implementations)
        self.agent_goals = {}  # Map agent_id -> (goal_row, goal_col)
        
        for row in range(len(State.goals)):
            for col in range(len(State.goals[row])):
                goal = State.goals[row][col]
                if "0" <= goal <= "9":
                    agent_id = ord(goal) - ord("0")
                    self.agent_goals[agent_id] = (row, col)

    @abstractmethod
    def h(self, state: State) -> int:
        """Heuristic function - estimates cost from state to goal."""
        ...

    @abstractmethod
    def f(self, state: State) -> int:
        """Evaluation function - determines node expansion order."""
        ...

    @abstractmethod
    def __repr__(self) -> str: ...



# EXERCISE 4 PART 2: GOAL COUNT HEURISTIC

class HeuristicGoalCount(Heuristic):
    """
    Goal Count Heuristic (Exercise 4 Part 2)
    
    Counts how many agents are not at their goal position.
    
    Properties:
    - Admissible: Each unsatisfied goal requires at least 1 action
    - Simple: O(agents) to compute
    - Weak: Doesn't consider distance to goals
    """
    
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)
    
    def h(self, state: State) -> int:
        """Count agents not at their goal positions."""
        count = 0
        
        for agent_id in range(len(state.agent_rows)):
            if agent_id in self.agent_goals:
                agent_pos = (state.agent_rows[agent_id], state.agent_cols[agent_id])
                goal_pos = self.agent_goals[agent_id]
                
                if agent_pos != goal_pos:
                    count += 1
        
        return count



# EXERCISE 4 PART 3: IMPROVED DISTANCE-BASED HEURISTIC


class HeuristicDistanceBased(Heuristic):
    """
    Improved Distance-Based Heuristic (Exercise 4 Part 3)
    
    Sums the actual shortest path distances from each agent to its goal.
    Uses BFS preprocessing to compute all-pairs shortest paths.
    
    Mathematical formulation:
        h'(s) = Σ dist(pos(agent_i, s), goal(agent_i))
    
    Properties:
    - Admissible: Each agent must travel at least dist(agent, goal) steps
    - Informed: Considers actual distances, not just presence of goals
    - Wall-aware: Uses BFS to find paths around obstacles
    - Fast: O(agents) lookup after O(cells²) preprocessing
    
    Weaknesses:
    - Ignores agent conflicts (assumes agents can pass through each other)
    - Memory intensive: O(cells²) space
    """
    
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)
        
        # Preprocessing: Compute all-pairs shortest paths using BFS
        # This allows O(1) distance lookups during heuristic computation
        self.distances = {}
        
        # Compute distances from all non-wall cells
        for row in range(len(State.walls)):
            for col in range(len(State.walls[row])):
                if not State.walls[row][col]:
                    self.distances[(row, col)] = self._bfs_distances(row, col)
    
    def _bfs_distances(self, start_row: int, start_col: int) -> dict:
        """
        Compute shortest distances from (start_row, start_col) to all reachable cells.
        Uses BFS to account for walls and actual grid topology.
        """
        distances = {}
        queue = deque([(start_row, start_col, 0)])
        visited = {(start_row, start_col)}
        
        while queue:
            row, col, dist = queue.popleft()
            distances[(row, col)] = dist
            
            # Explore neighbors (up, down, left, right)
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                new_row, new_col = row + dr, col + dc
                
                # Check bounds and walls
                if (0 <= new_row < len(State.walls) and 
                    0 <= new_col < len(State.walls[0]) and
                    not State.walls[new_row][new_col] and
                    (new_row, new_col) not in visited):
                    
                    visited.add((new_row, new_col))
                    queue.append((new_row, new_col, dist + 1))
        
        return distances
    
    def h(self, state: State) -> int:
        """
        Sum of actual shortest path distances from each agent to its goal.
        
        This is admissible because:
        - Each agent must travel at least dist(agent, goal) steps
        - We underestimate conflicts between agents
        - Therefore h(n) ≤ h*(n) always
        """
        total_distance = 0
        
        # For each agent, add distance to its goal
        for agent_id in range(len(state.agent_rows)):
            if agent_id in self.agent_goals:
                agent_pos = (state.agent_rows[agent_id], state.agent_cols[agent_id])
                goal_pos = self.agent_goals[agent_id]
                
                # Look up precomputed distance
                if agent_pos in self.distances and goal_pos in self.distances[agent_pos]:
                    total_distance += self.distances[agent_pos][goal_pos]
                # If no path exists (shouldn't happen), use Manhattan as fallback
                else:
                    total_distance += abs(agent_pos[0] - goal_pos[0]) + abs(agent_pos[1] - goal_pos[1])
        
        return total_distance


# DEFAULT HEURISTIC 

# To switch between heuristics, change the parent class here:

class HeuristicDefault(HeuristicDistanceBased):
    """
    Default heuristic used by search algorithms.
    Currently set to HeuristicDistanceBased (Exercise 4 Part 3).
    
    To benchmark goal count heuristic, change to:
        class HeuristicDefault(HeuristicGoalCount):
    """
    pass


# Alias for backward compatibility
Heuristic = HeuristicDefault



# EVALUATION FUNCTION WRAPPERS (A*, Greedy, Weighted A*)


class HeuristicAStar(HeuristicDefault):
    """A* evaluation: f(n) = g(n) + h(n)"""
    
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return state.g + self.h(state)

    def __repr__(self) -> str:
        return "A* evaluation"


class HeuristicWeightedAStar(HeuristicDefault):
    """Weighted A* evaluation: f(n) = g(n) + w*h(n)"""
    
    def __init__(self, initial_state: State, w: int) -> None:
        super().__init__(initial_state)
        self.w = w

    def f(self, state: State) -> int:
        return state.g + self.w * self.h(state)

    def __repr__(self) -> str:
        return f"WA*({self.w}) evaluation"


class HeuristicGreedy(HeuristicDefault):
    """Greedy best-first evaluation: f(n) = h(n)"""
    
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return self.h(state)

    def __repr__(self) -> str:
        return "greedy evaluation"
