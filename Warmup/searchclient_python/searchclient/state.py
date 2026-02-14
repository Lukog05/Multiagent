import random
from typing import ClassVar #classVar is used to indicate that the variable is a class variable, meaning it is shared among all instances of the class. In this code, agent_colors, walls, box_colors, and goals are defined as class variables, which means they are shared across all instances of the State class. This is useful for storing information that is common to all states, such as the layout of the level (walls and goals) and the colors of agents and boxes.

from searchclient.action import Action, ActionType
from searchclient.color import Color


class State:
    _RNG = random.Random(1)

    agent_colors: ClassVar[list[Color | None]] #List of colors for each agent. Indexed by agent number. None if agent has no color.
    walls: ClassVar[list[list[bool]]] #2D list of booleans. True if there's a wall at (row, col).
    box_colors: ClassVar[list[Color | None]] #list of colors for each box. Indexed by (row, col). None if no box at (row, col).
    goals: ClassVar[list[list[str]]] #list of goals. Indexed by (row, col). Empty string if no goal at (row, col). Otherwise, the goal is represented as a single character string: "A"-"Z" for box goals and "0"-"9" for agent goals.

    def __init__(self, agent_rows: list[int], agent_cols: list[int], boxes: list[list[str]]) -> None:
        """
        Constructs an initial state.
        Arguments are not copied, and therefore should not be modified after being passed in.

        The lists walls, boxes, and goals are indexed from top-left of the level, row-major order (row, col).
               Col 0  Col 1  Col 2  Col 3
        Row 0: (0,0)  (0,1)  (0,2)  (0,3)  ...
        Row 1: (1,0)  (1,1)  (1,2)  (1,3)  ...
        Row 2: (2,0)  (2,1)  (2,2)  (2,3)  ...
        ...

        For example, State.walls[2] is a list of booleans for the third row.
        State.walls[row][col] is True if there's a wall at (row, col).

        The agent rows and columns are indexed by the agent number.
        For example, State.agent_rows[0] is the row location of agent '0'.

        Note: The state should be considered immutable after it has been hashed, e.g. added to a dictionary or set.
        """
        self.agent_rows = agent_rows
        self.agent_cols = agent_cols
        self.boxes = boxes
        self.parent: State | None = None
        self.joint_action: list[Action] | None = None
        self.g = 0
        self._hash: int | None = None

    def result(self, joint_action: list[Action]) -> "State":
        """
        Returns the state resulting from applying joint_action in this state.
        Precondition: Joint action must be applicable and non-conflicting in this state.
        """

        # Copy this state.
        copy_agent_rows = self.agent_rows[:]
        copy_agent_cols = self.agent_cols[:]
        copy_boxes = [row[:] for row in self.boxes]

        # Apply each action.
        for agent, action in enumerate(joint_action):
            agent_row=self.agent_rows[agent]
            agent_col= self.agent_cols[agent]
            if action.type is ActionType.NoOp:
                pass 

            elif action.type is ActionType.Move:
                copy_agent_rows[agent] += action.agent_row_delta
                copy_agent_cols[agent] += action.agent_col_delta
            elif action.type is ActionType.Push:
                box_row=agent_row + action.agent_row_delta
                box_col=agent_col +action.agent_col_delta

                new_box_row=box_row +action.box_row_delta
                new_box_col=box_col+action.box_col_delta
                copy_boxes[new_box_row][new_box_col]=copy_boxes[box_row][box_col]
                copy_boxes[box_row][box_col]=""

                copy_agent_rows[agent] += action.agent_row_delta
                copy_agent_cols[agent] += action.agent_col_delta
            elif action.type is ActionType.Pull:
                new_agent_row= agent_row + action.agent_row_delta
                new_agent_col= agent_col + action.agent_col_delta
                box_row=agent_row - action.agent_row_delta
                box_col=agent_col - action.agent_col_delta
                copy_boxes[agent_row][agent_col]=copy_boxes[box_row][box_col]
                copy_boxes[box_row][box_col]=""
                copy_agent_rows[agent] = new_agent_row
                copy_agent_cols[agent] = new_agent_col
                
        copy_state = State(copy_agent_rows, copy_agent_cols, copy_boxes)

        copy_state.parent = self
        copy_state.joint_action = joint_action.copy()
        copy_state.g = self.g + 1

        return copy_state

    def is_goal_state(self) -> bool:
        for row in range(len(State.goals)):
            for col in range(len(State.goals[row])):
                goal = State.goals[row][col]

                if "A" <= goal <= "Z" and self.boxes[row][col] != goal:
                    return False
                if "0" <= goal <= "9" and not (
                    self.agent_rows[ord(goal) - ord("0")] == row and self.agent_cols[ord(goal) - ord("0")] == col
                ):
                    return False
        return True

    def get_expanded_states(self) -> list["State"]:
        num_agents = len(self.agent_rows)

        # Determine list of applicable action for each individual agent.
        applicable_actions = [
            [action for action in Action if self.is_applicable(agent, action)] for agent in range(num_agents)
        ]

        # Iterate over joint actions, check conflict and generate child states.
        joint_action = [Action.NoOp for _ in range(num_agents)]
        actions_permutation = [0 for _ in range(num_agents)]
        expanded_states = []
        while True:
            for agent in range(num_agents):
                joint_action[agent] = applicable_actions[agent][actions_permutation[agent]]

            if not self.is_conflicting(joint_action):
                expanded_states.append(self.result(joint_action))

            # Advance permutation.
            done = False
            for agent in range(num_agents):
                if actions_permutation[agent] < len(applicable_actions[agent]) - 1:
                    actions_permutation[agent] += 1
                    break
                else:  # noqa: RET508
                    actions_permutation[agent] = 0
                    if agent == num_agents - 1:
                        done = True

            # Last permutation?
            if done:
                break

        State._RNG.shuffle(expanded_states)
        return expanded_states

    def is_applicable(self, agent: int, action: Action) -> bool:
        agent_row = self.agent_rows[agent]
        agent_col = self.agent_cols[agent]
        agent_color = State.agent_colors[agent]

        if action.type is ActionType.NoOp:
            return True

        if action.type is ActionType.Move:
            destination_row = agent_row + action.agent_row_delta
            destination_col = agent_col + action.agent_col_delta
            return self.is_free(destination_row, destination_col)
        if action.type is ActionType.Push:
            box_row=agent_row+action.agent_row_delta 
            box_col=agent_col+action.agent_col_delta
            box_letter=self.boxes[box_row][box_col]
            if not box_letter:
                return False
            if State.box_colors[ord(box_letter)-ord("A")] != agent_color:
                return False
            new_box_row=box_row+ action.box_row_delta
            new_box_col= box_col+action.box_col_delta

            return self.is_free(new_box_row,new_box_col)
        if action.type is ActionType.Pull:
             destination_row = agent_row + action.agent_row_delta
             destination_col = agent_col + action.agent_col_delta
             if not self.is_free(destination_row, destination_col):
                 return False
             box_row=agent_row - action.agent_row_delta
             box_col=agent_col - action.agent_col_delta
             box_letter=self.boxes[box_row][box_col]
             if not box_letter:
                return False
             if State.box_colors[ord(box_letter)-ord("A")] != agent_color:
                return False
             return True
        assert False, f"Not implemented for action type {action.type}."

    def is_conflicting(self, joint_action: list[Action]) -> bool:
        num_agents = len(self.agent_rows)

        destination_rows = [-1 for _ in range(num_agents)]  # row of new cell to become occupied by action
        destination_cols = [-1 for _ in range(num_agents)]  # column of new cell to become occupied by action
        box_rows = [-1 for _ in range(num_agents)]  # current row of box moved by action
        box_cols = [-1 for _ in range(num_agents)]  # current column of box moved by action

        # Collect cells to be occupied and boxes to be moved.
        for agent in range(num_agents):
            action = joint_action[agent]
            agent_row = self.agent_rows[agent]
            agent_col = self.agent_cols[agent]

            if action.type is ActionType.NoOp:
                pass

            elif action.type is ActionType.Move:
                destination_rows[agent] = agent_row + action.agent_row_delta
                destination_cols[agent] = agent_col + action.agent_col_delta
                box_rows[agent] = agent_row  # Distinct dummy value.
                box_cols[agent] = agent_col  # Distinct dummy value.
            elif action.type is ActionType.Push:
                destination_rows[agent] = agent_row + action.agent_row_delta
                destination_cols[agent] = agent_col + action.agent_col_delta
                box_rows[agent] = destination_rows[agent]
                box_cols[agent] = destination_cols[agent]
            elif action.type is ActionType.Pull:
                destination_rows[agent] = agent_row + action.agent_row_delta
                destination_cols[agent] = agent_col + action.agent_col_delta
                box_rows[agent] =agent_row - action.agent_row_delta
                box_cols[agent] =agent_col - action.agent_col_delta

        for a1 in range(num_agents):
            if joint_action[a1] is Action.NoOp:
                continue

            for a2 in range(a1 + 1, num_agents):
                if joint_action[a2] is Action.NoOp:
                    continue

                # Moving into same cell?
                if destination_rows[a1] == destination_rows[a2] and destination_cols[a1] == destination_cols[a2]:
                    return True

        return False

    def is_free(self, row: int, col: int) -> bool:
        return not State.walls[row][col] and not self.boxes[row][col] and self.agent_at(row, col) is None

    def agent_at(self, row: int, col: int) -> str | None:
        for agent in range(len(self.agent_rows)):
            if self.agent_rows[agent] == row and self.agent_cols[agent] == col:
                return chr(agent + ord("0"))
        return None

    def extract_plan(self) -> list[list[Action]]:
        plan = []
        state: State | None = self
        while state is not None and state.joint_action is not None:
            plan.append(state.joint_action)
            state = state.parent
        plan.reverse()
        return plan

    def __hash__(self) -> int:
        if self._hash is None:
            prime = 31
            h = 1
            h = h * prime + hash(tuple(self.agent_rows))
            h = h * prime + hash(tuple(self.agent_cols))
            h = h * prime + hash(tuple(State.agent_colors))
            h = h * prime + hash(tuple(tuple(row) for row in self.boxes))
            h = h * prime + hash(tuple(State.box_colors))
            h = h * prime + hash(tuple(tuple(row) for row in State.goals))
            h = h * prime + hash(tuple(tuple(row) for row in State.walls))
            self._hash = h
        return self._hash

    def __eq__(self, other: object) -> bool:
        if self is other:
            return True
        if not isinstance(other, State):
            return False
        if self.agent_rows != other.agent_rows:
            return False
        if self.agent_cols != other.agent_cols:
            return False
        if State.agent_colors != other.agent_colors:
            return False
        if State.walls != other.walls:
            return False
        if self.boxes != other.boxes:
            return False
        if State.box_colors != other.box_colors:
            return False
        return State.goals == other.goals

    def __repr__(self) -> str:
        lines = []
        for row in range(len(self.boxes)):
            line = []
            for col in range(len(self.boxes[row])):
                if self.boxes[row][col]:
                    line.append(self.boxes[row][col])
                elif State.walls[row][col] is not None:
                    line.append("+")
                elif (agent := self.agent_at(row, col)) is not None:
                    line.append(agent)
                else:
                    line.append(" ")
            lines.append("".join(line))
        return "\n".join(lines)


"""
Initial state is repsented as an instance of State class.
i) Agent position:
    Under __init__ method, agent_rows and agent_cols give agent positions.
    # Lists indexed by agent number
    agent_rows = [1, 3, 5]  # Agent 0 at row 1, Agent 1 at row 3, Agent 2 at row 5
    agent_cols = [2, 4, 6]  # Agent 0 at col 2, Agent 1 at col 4, Agent 2 at col 6

ii) Agent color: (class variable, shared across all states)
    agent_colors: ClassVar[list[Color | None]]
    # Shared across ALL states (never changes)
    # agent_colors = [Color.Blue, Color.Red, None]
    # Agent 0 is blue, Agent 1 is red, Agent 2 has no color

iii) Box position:
    self.boxes = boxes
    # 2D grid, same dimensions as the level
    # boxes[row][col] = letter if box present, empty string "" if not

    boxes = [
        ["", "", "", ""],      # Row 0: no boxes
        ["", "A", "", "B"],    # Row 1: Box A at (1,1), Box B at (1,3)
        ["", "", "C", ""],     # Row 2: Box C at (2,2)
    ]

iv) Box color: (class variable, shared across all states)
    box_colors: ClassVar[list[Color | None]]
    # Shared across ALL states
    # Indexed by box letter: 'A' = index 0, 'B' = index 1, etc.
    # box_colors[0] = Color.Blue  (Box type A is blue)
    # box_colors[1] = Color.Red   (Box type B is red)

v) Wall position: (class variable, shared across all states)
    walls: ClassVar[list[list[bool]]]
    # 2D grid, same dimensions as the level
    # walls[row][col] = True if wall at (row, col), False otherwise

    walls = [
        [False, False, False, False],  # Row 0: no walls
        [False, True, False, False],   # Row 1: wall at (1,1)
        [False, False, False, False],  # Row 2: no walls
    ]
    
vi) Goal position: (class variable, shared across all states)
    goals: ClassVar[list[list[str]]]
    # 2D grid, same dimensions as the level
    # goals[row][col] = "" if no goal at (row, col)
    # goals[row][col] = "A"-"Z" for box goals (box of type A should go here)
    # goals[row][col] = "0"-"9" for agent goals (agent of type 0 should go here)

    goals = [
        ["", "", "", ""],      # Row 0: no goals
        ["", "A", "", "0"],   # Row 1: Box goal A at (1,1), Agent goal 0 at (1,3)
        ["", "", "B", ""],    # Row 2: Box goal B at (2,2)
    ]
    
"""

"""
is_applicable method checks if an action can be applied by an agent in the current state. For example, for a Move action, it checks if the destination cell is free (not a wall, not occupied by another box or agent).
is_free method checks if a cell is free, meaning it has no wall, no box, and no agent. This is used by is_applicable to determine if an agent can move into a cell.
is_conflicting method checks if a joint action (actions for all agents) has any conflicts, such as two agents trying to move into the same cell. This is used in get_expanded_states to filter out joint actions that would result in conflicts.
result method applies a joint action to the current state and returns the resulting state. It creates a copy of the current state, applies the actions of all agents, and returns the new state.
"""

"""
| Component       | Type     | Storage                      | Example                 |
| --------------- | -------- | ---------------------------- | ----------------------- |
| Agent positions | Instance | agent_rows[i], agent_cols[i] | [1, 3], [2, 4]          |
| Agent colors    | Class    | agent_colors[i]              | [Color.Blue, Color.Red] |
| Box positions   | Instance | boxes[row][col]              | [["", "A"], ["B", ""]]  |
| Box colors      | Class    | box_colors[letter_index]     | [Color.Blue, Color.Red] |
| Walls           | Class    | walls[row][col]              | [[True, False], ...]    |
| Goals           | Class    | goals[row][col]              | [["", "A"], ["0", ""]]  |
"""
