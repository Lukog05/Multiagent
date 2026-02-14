from abc import ABC, abstractmethod
from collections import deque

from searchclient.state import State


class Heuristic(ABC):
    def __init__(self, initial_state: State) -> None:
        self.distances = {}
        self.agent_goals = {}
        
        # Find all agent goals
        for row in range(len(State.goals)):
            for col in range(len(State.goals[row])):
                goal = State.goals[row][col]
                if "0" <= goal <= "9":
                    agent_id = ord(goal) - ord("0")
                    self.agent_goals[agent_id] = (row, col)
        
        # Compute distances from all non-wall cells
        for row in range(len(State.walls)):
            for col in range(len(State.walls[row])):
                if not State.walls[row][col]:
                    self.distances[(row, col)] = self._bfs_distances(row, col)

    def _bfs_distances(self, start_row: int, start_col: int) -> dict:
        distances = {}
        queue = deque([(start_row, start_col, 0)])
        visited = {(start_row, start_col)}
        
        while queue:
            row, col, dist = queue.popleft()
            distances[(row, col)] = dist
            
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                new_row, new_col = row + dr, col + dc
                
                if (0 <= new_row < len(State.walls) and 
                    0 <= new_col < len(State.walls[0]) and
                    not State.walls[new_row][new_col] and
                    (new_row, new_col) not in visited):
                    
                    visited.add((new_row, new_col))
                    queue.append((new_row, new_col, dist + 1))
        
        return distances

    def h(self, state: State) -> int:
        has_boxes = any("A" <= state.boxes[row][col] <="Z" for row in range(len(state.boxes)) for col in range(len(state.boxes[row])))
        if has_boxes:
            return self.sokoban_h(state)
        else:
            return self.multiagent_h(state)    
            
    def multiagent_h(self, state: State) -> int:
        total_distance = 0
        
        for agent_id in range(len(state.agent_rows)):
            if agent_id in self.agent_goals:
                agent_pos = (state.agent_rows[agent_id], state.agent_cols[agent_id])
                goal_pos = self.agent_goals[agent_id]
                
                if agent_pos in self.distances and goal_pos in self.distances[agent_pos]:
                    total_distance += self.distances[agent_pos][goal_pos]
                else:
                    total_distance += abs(agent_pos[0] - goal_pos[0]) + abs(agent_pos[1] - goal_pos[1])
        
        return total_distance

    def sokoban_h(self, state: State) -> int:
        goals=[]
        boxes=[]

        for row in range(len(State.goals)):
            for col in range(len(State.goals[row])):
                if "A" <= State.goals[row][col] <= "Z":
                    goals.append((row,col))

        for row in range(len(state.boxes)):
            for col in range(len(state.boxes[row])):
                if "A" <= state.boxes[row][col] <= "Z":
                    if State.goals[row][col] != state.boxes[row][col]:
                        boxes.append((row,col))

        if not boxes:
            return 0

        for (row,col) in boxes:
            if self.is_deadlock(state,row,col):
                return 10**9

            reachable = False
            for goal in goals:
                if (row,col) in self.distances and goal in self.distances[(row,col)]:
                    reachable = True
                    break
            if not reachable:
                return 10**9

        if len(boxes) <= 6:
            cost_matrix=[]
            for box in boxes:
                row_cost=[]
                for goal in goals:
                    if box in self.distances and goal in self.distances[box]:
                        row_cost.append(self.distances[box][goal])
                    else:
                        row_cost.append(1000)
                cost_matrix.append(row_cost)

            total_distance = self.min_assignment(cost_matrix)

        else:
            total_distance = 0
            used_goals = set()

            for box in boxes:
                best = float("inf")
                best_goal = None

                for goal in goals:
                    if goal in used_goals:
                        continue

                    if box in self.distances and goal in self.distances[box]:
                        dist = self.distances[box][goal]
                    else:
                        dist = abs(box[0] - goal[0]) + abs(box[1] - goal[1])

                    if dist < best:
                        best = dist
                        best_goal = goal

                if best_goal:
                    used_goals.add(best_goal)

                total_distance += best

        # Agent to nearest box
        agent_pos =(state.agent_rows[0],state.agent_cols[0])
        min_agent_dist=float("inf")

        for box in boxes:
            if agent_pos in self.distances and box in self.distances[agent_pos]:
                dist = self.distances[agent_pos][box]
            else:
                dist = abs(agent_pos[0] - box[0]) + abs(agent_pos[1] - box[1])

            min_agent_dist=min(min_agent_dist,dist)

        if min_agent_dist != float("inf"):
            total_distance += min_agent_dist

        # Small penalty
        total_distance += len(boxes) * 5

        return total_distance
    def min_assignment(self,cost_matrix):
        n=len(cost_matrix)
        memo={}
        def dp(i,mask):
            if i==n:
                return 0
            if (i,mask) in memo:
                return memo[(i,mask)]
            best=float("inf")
            for j in range(len(cost_matrix[0])):
                if not (mask &(1 << j)):
                    cost = cost_matrix[i][j] + dp(i+1, mask | (1 << j))
                    best=min(best,cost)  
            memo[(i,mask)]= best
            return best
        return dp(0,0)

    def is_deadlock(self,state: State,row,col):
        if State.goals[row][col] == state.boxes[row][col]:
            return False

        up=State.walls[row-1][col]
        down=State.walls[row+1][col]
        left=State.walls[row][col-1]
        right=State.walls[row][col+1]  

        # Corner deadlock
        if (up and left) or (up and right) or (down and left) or (down and right):
            return True

        # Wall deadlock (horizontal)
        if up or down:
            goal_in_row = False
            for c in range(len(State.goals[row])):
                if State.goals[row][c] == state.boxes[row][col]:
                    goal_in_row = True
                    break
            if not goal_in_row:
                return True

        # Wall deadlock (vertical)
        if left or right:
            goal_in_col = False
            for r in range(len(State.goals)):
                if State.goals[r][col] == state.boxes[row][col]:
                    goal_in_col = True
                    break
            if not goal_in_col:
                return True

        return False

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

"""
Signature of h(state) method:

h(self, state: State) -> int
Breakdown:

Method name: h
Parameters:
    self - the heuristic object instance
    state: State - the state to evaluate
    Return type: int - estimated cost from state to goal
Current implementation: Raises NotImplementedError
"""

"""
HeuristicAstar class returns state.g  + self.h(state) in its f(state) method, which is the standard A* evaluation function combining the cost to reach the current state (g) and the heuristic estimate to reach the goal (h).
i.e., f(n) = g(n) + h(n)
g(n) and h(n) are exploration and exploitation respectively.

HeuristicGreedy class returns self.h(state) in its f(state) method, which is the standard greedy evaluation function that relies solely on the heuristic estimate to reach the goal, ignoring the cost to reach the current state.
i.e., f(n) = h(n)

HeuristicWeightedAStar class returns state.g + self.w * self.h(state) in its f(state) method, which is a weighted A* evaluation function that combines the cost to reach the current state (g) with a weighted heuristic estimate to reach the goal (w * h). The weight w allows for tuning the influence of the heuristic in the search process.
i.e., f(n) = g(n) + w * h(n)
"""

"""
All heuristics share:
    h(state) method (you implement the heuristic logic once)
    __init__ for preprocessing
But differ in:
    How f(state) combines g and h
    Search behavior and guarantees
"""

"""
__repr__ method is a dunder method.
It is used to define the string representation of an object, which is what you see when you print the object or inspect it in a debugger.
Ex:
Class Person(...):
    def __repr__(self):
        return f"Person(name={self.name}, age={self.age})"

person = Person()
without __repr__, printing person would show something like <__main__.Person object at 0x7f8b9c2d0>, which is not informative.
With __repr__, it would show Person(name=Alice, age=30), which is much more useful for debugging and understanding the object's state.
"""