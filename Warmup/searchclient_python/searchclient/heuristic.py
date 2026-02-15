from abc import ABC, abstractmethod
from collections import deque
import sys

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
    '''
    def h(self, state: State) -> int:
        has_boxes = any("A" <= state.boxes[row][col] <="Z" for row in range(len(state.boxes)) for col in range(len(state.boxes[row])))
        #Adding this line for Ex7 and modifying if statement
        has_multiple_agents = len(state.agent_rows) > 1
        if has_boxes and has_multiple_agents:
            return self.full_hospital_h(state)
        elif has_boxes:
            return self.sokoban_h(state)
        else:
            return self.multiagent_h(state)
    '''
    def h(self, state: State) -> int:
        has_boxes = any(
            "A" <= state.boxes[row][col] <= "Z" 
            for row in range(len(state.boxes)) 
            for col in range(len(state.boxes[row]))
        )
        
        has_multiple_agents = len(state.agent_rows) > 1
        
        # DEBUG: Print which heuristic is used (only once)
        if not hasattr(self, '_debug_printed'):
            print(f"[DEBUG] has_boxes={has_boxes}, has_multiple_agents={has_multiple_agents}", file=sys.stderr, flush=True)
            self._debug_printed = True
        
        if has_boxes and has_multiple_agents:
            return self.full_hospital_h(state)
        elif has_boxes:
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
    
    
    '''Adding the below heuristic for Ex7, which combines the multiagent and sokoban heuristics. 
    It calculates the distance from each agent to the nearest box and adds a small penalty for each box to encourage faster solutions.'''
    def full_hospital_h(self, state: State) -> int:
        
        #Identifying the Goals and Boxes in the state and storing them in separate lists for easier access
        agent_goals_dict ={}
        box_goals = []
        boxes_not_at_goals = []
        
        for row in range(len(State.goals)):
            for col in range(len(State.goals[row])):
                goal = State.goals[row][col]
                if "0" <= goal <= "9":
                    agent_id = ord(goal) - ord("0")
                    agent_goals_dict[agent_id] = (row, col)
                elif "A" <= goal <= "Z":
                    box_goals.append((row, col, goal))
        
        for row in range(len(state.boxes)):
            for col in range(len(state.boxes[row])):
                box = state.boxes[row][col]
                if "A" <= box <= "Z":
                    if State.goals[row][col] != box:
                        boxes_not_at_goals.append((row, col, box))
        
        
        #Next, we are checking for deadlocks and unreachable goals, which are critical for Sokoban puzzles. 
        #If any box is in a deadlock position or if any box cannot reach its goal, we return a very high heuristic value to indicate that this state is undesirable.
        for (row, col, box) in boxes_not_at_goals:
            if self.is_deadlock(state, row, col):
                return 10**9

            reachable = False
            for (goal_row, goal_col, goal_box) in box_goals:
                if box == goal_box:
                    if (row, col) in self.distances and (goal_row, goal_col) in self.distances[(row, col)]:
                        reachable = True
                        break
            if not reachable:
                return 10**9
        
        #Now, we commpute Agent to Goal distances using the precomputed distances from the BFS. 
        #We sum up the distances from each agent to its respective goal, and also add the distances from each box to its nearest goal. 
        #Finally, we add a small penalty for each box to encourage faster solutions.
        
        agent_cost = 0
        agents_with_goals = []
        agents_without_goals = []
        for agent_id in range(len(state.agent_rows)):
            agent_pos = (state.agent_rows[agent_id], state.agent_cols[agent_id])
            if agent_id in agent_goals_dict:
                goal_pos = agent_goals_dict[agent_id]
                if agent_pos in self.distances and goal_pos in self.distances[agent_pos]:
                    dist = self.distances[agent_pos][goal_pos]
                else:
                    dist = abs(agent_pos[0] - goal_pos[0]) + abs(agent_pos[1] - goal_pos[1])
                agent_cost += dist
                agents_with_goals.append((agent_id, agent_pos))
            else:
                agents_without_goals.append((agent_id, agent_pos))
                
        # Computing box to goal distances
        box_cost = 0
        if not boxes_not_at_goals:
            return agent_cost
        #We use the Hungarian algorithm for optimal assignment if there are 6 or fewer boxes, otherwise we can use a greedy approach
        if len(boxes_not_at_goals) <= 6:
            cost_matrix = []
            for (box_row, box_col, box_letter) in boxes_not_at_goals:
                row_costs = []
                for(goal_row, goal_col, goal_letter) in box_goals:
                    if box_letter == goal_letter:
                        box_pos = (box_row, box_col)
                        goal_pos = (goal_row, goal_col)
                        if box_pos in self.distances and goal_pos in self.distances[box_pos]:
                            dist = self.distances[box_pos][goal_pos]
                        else:
                            dist = abs(box_row - goal_row) + abs(box_col - goal_col)
                        row_costs.append(dist)
                    else:
                        row_costs.append(10000)  # Large cost for non-matching goals
                cost_matrix.append(row_costs)
            box_cost = self.min_assignment(cost_matrix)
        else:
            #Greedy assignment for many boxes
            used_goals = set()
            for (box_row, box_col, box_letter) in boxes_not_at_goals:
                best_dist = float("inf")
                best_goal = None
                for (goal_row, goal_col, goal_letter) in box_goals:
                    if (goal_row, goal_col) in used_goals:
                        continue
                    if box_letter == goal_letter:
                        box_pos = (box_row, box_col)
                        goal_pos = (goal_row, goal_col)
                        if box_pos in self.distances and goal_pos in self.distances[box_pos]:
                            dist = self.distances[box_pos][goal_pos]
                        else:
                            dist = abs(box_row - goal_row) + abs(box_col - goal_col)
                        if dist < best_dist:
                            best_dist = dist
                            best_goal = (goal_row, goal_col)
                if best_goal:
                    used_goals.add(best_goal)
                    box_cost += best_dist

        #Agent box interaction(coordination) cost
        coordination_cost = 0
        if boxes_not_at_goals and agents_without_goals:
            min_dist = float("inf")
            for (agent_id, agent_pos) in agents_without_goals:
                for (box_row, box_col, box_letter) in boxes_not_at_goals:
                    box_pos = (box_row, box_col)
                    if agent_pos in self.distances and box_pos in self.distances[agent_pos]:
                        dist = self.distances[agent_pos][box_pos]
                    else:
                        dist = abs(agent_pos[0] - box_row) + abs(agent_pos[1] - box_col)
                    min_dist = min(min_dist, dist)
            coordination_cost = min_dist
        elif boxes_not_at_goals:
            if agents_with_goals:
                coordination_cost = min(
                    abs(agent_pos[0] - box_row) + abs(agent_pos[1] - box_col)
                    for (agent_id, agent_pos) in agents_with_goals
                    for (box_row, box_col, box_letter) in boxes_not_at_goals
                )
            else:
                coordination_cost = len(boxes_not_at_goals) * 2  # Penalty if no agents have goals
        
        #Now combining the costs
        total_cost = agent_cost + box_cost + coordination_cost
        # Small penalty for each box to encourage faster solutions
        total_cost += len(boxes_not_at_goals) * 3
        return total_cost
    
      
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