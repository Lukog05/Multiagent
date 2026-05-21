import random
import sys
from collections import deque
from typing import ClassVar

from searchclient.action import Action, ActionType
from searchclient.color import Color

ENABLE_SIMPLE_DEADLOCK = True
ENABLE_2BOX_DEADLOCK   = True

ENABLE_ZOBRIST = True

class State:
    _RNG = random.Random(1)

    _simple_deadlock_count: ClassVar[int] = 0
    _2box_deadlock_count: ClassVar[int]   = 0
    _states_kept: ClassVar[int]           = 0
    _states_pruned: ClassVar[int]         = 0

    @classmethod
    def reset_diagnostics(cls) -> None:
        cls._simple_deadlock_count = 0
        cls._2box_deadlock_count   = 0
        cls._states_kept           = 0
        cls._states_pruned         = 0

    @classmethod
    def print_diagnostics(cls) -> None:
        total = cls._states_kept + cls._states_pruned
        ratio = (cls._states_pruned / total * 100) if total else 0
        print("[diagnostics] Pruning stats:", file=sys.stderr, flush=True)
        print(f"  Simple deadlocks pruned : {cls._simple_deadlock_count}", file=sys.stderr, flush=True)
        print(f"  2-box deadlocks pruned  : {cls._2box_deadlock_count}", file=sys.stderr, flush=True)
        print(f"  Total states kept       : {cls._states_kept}", file=sys.stderr, flush=True)
        print(f"  Total states pruned     : {cls._states_pruned}", file=sys.stderr, flush=True)
        print(f"  Pruning ratio           : {ratio:.1f}%", file=sys.stderr, flush=True)

    agent_colors: ClassVar[list[Color | None]]
    walls: ClassVar[list[list[bool]]]
    box_colors: ClassVar[list[Color | None]]
    goals: ClassVar[list[list[str]]]
    _static_hash: ClassVar[int]
    _dead_cells: ClassVar[dict[str, frozenset[tuple[int, int]]]]
    _zobrist_agents: ClassVar[list[list[int]]]
    _zobrist_boxes: ClassVar[list[list[int]]]

    def __init__(self, agent_rows: list[int], agent_cols: list[int], boxes: list[list[str]], _precomputed_hash: int | None = None) -> None:
        self.agent_rows = agent_rows
        self.agent_cols = agent_cols
        self.boxes = boxes
        self.parent: State | None = None
        self.joint_action: list[Action] | None = None
        self.g = 0
        self._hash: int | None = _precomputed_hash
        self._moved_box_positions: list[tuple[int, int]] = []

    def result(self, joint_action: list[Action]) -> "State":

        copy_agent_rows = self.agent_rows[:]
        copy_agent_cols = self.agent_cols[:]
        copy_boxes = [row[:] for row in self.boxes]

        for agent, action in enumerate(joint_action):
            if action.type is ActionType.NoOp:
                pass

            elif action.type is ActionType.Move:
                copy_agent_rows[agent] += action.agent_row_delta
                copy_agent_cols[agent] += action.agent_col_delta

            elif action.type is ActionType.Push:
                box_row = self.agent_rows[agent] + action.agent_row_delta
                box_col = self.agent_cols[agent] + action.agent_col_delta
                copy_boxes[box_row + action.box_row_delta][box_col + action.box_col_delta] = copy_boxes[box_row][box_col]
                copy_boxes[box_row][box_col] = ""
                copy_agent_rows[agent] += action.agent_row_delta
                copy_agent_cols[agent] += action.agent_col_delta

            elif action.type is ActionType.Pull:
                box_row = self.agent_rows[agent] - action.box_row_delta
                box_col = self.agent_cols[agent] - action.box_col_delta
                copy_boxes[self.agent_rows[agent]][self.agent_cols[agent]] = copy_boxes[box_row][box_col]
                copy_boxes[box_row][box_col] = ""
                copy_agent_rows[agent] += action.agent_row_delta
                copy_agent_cols[agent] += action.agent_col_delta

        if ENABLE_ZOBRIST:
            cols = len(State.walls[0]) if State.walls else 1
            new_hash = self.__hash__()
            for agent, action in enumerate(joint_action):
                if action.type is ActionType.NoOp:
                    continue
                old_r, old_c = self.agent_rows[agent], self.agent_cols[agent]
                new_r, new_c = copy_agent_rows[agent], copy_agent_cols[agent]
                new_hash ^= State._zobrist_agents[agent][old_r * cols + old_c]
                new_hash ^= State._zobrist_agents[agent][new_r * cols + new_c]
                if action.type is ActionType.Push:
                    box_r = self.agent_rows[agent] + action.agent_row_delta
                    box_c = self.agent_cols[agent] + action.agent_col_delta
                    new_box_r = box_r + action.box_row_delta
                    new_box_c = box_c + action.box_col_delta
                    letter_idx = ord(self.boxes[box_r][box_c]) - ord('A')
                    new_hash ^= State._zobrist_boxes[letter_idx][box_r * cols + box_c]
                    new_hash ^= State._zobrist_boxes[letter_idx][new_box_r * cols + new_box_c]
                elif action.type is ActionType.Pull:
                    box_r = self.agent_rows[agent] - action.box_row_delta
                    box_c = self.agent_cols[agent] - action.box_col_delta
                    new_box_r = self.agent_rows[agent]
                    new_box_c = self.agent_cols[agent]
                    letter_idx = ord(self.boxes[box_r][box_c]) - ord('A')
                    new_hash ^= State._zobrist_boxes[letter_idx][box_r * cols + box_c]
                    new_hash ^= State._zobrist_boxes[letter_idx][new_box_r * cols + new_box_c]
            new_hash &= 0xFFFFFFFFFFFFFFFF
            copy_state = State(copy_agent_rows, copy_agent_cols, copy_boxes, _precomputed_hash=new_hash)
        else:
            copy_state = State(copy_agent_rows, copy_agent_cols, copy_boxes)

        copy_state._moved_box_positions = []
        for agent, action in enumerate(joint_action):
            if action.type is ActionType.Push:
                box_row = self.agent_rows[agent] + action.agent_row_delta
                box_col = self.agent_cols[agent] + action.agent_col_delta
                new_box_row = box_row + action.box_row_delta
                new_box_col = box_col + action.box_col_delta
                copy_state._moved_box_positions.append((new_box_row, new_box_col))
            elif action.type is ActionType.Pull:
                new_box_row = self.agent_rows[agent]
                new_box_col = self.agent_cols[agent]
                copy_state._moved_box_positions.append((new_box_row, new_box_col))

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

        has_boxes = any(self.boxes[r][c] for r in range(len(self.boxes)) for c in range(len(self.boxes[r])))

        if not has_boxes:
            expanded_states: list[State] = []
            move_actions = [a for a in Action if a.type is ActionType.Move]

            for agent in range(num_agents):
                for action in move_actions:
                    if not self.is_applicable(agent, action):
                        continue
                    joint_action = [Action.NoOp for _ in range(num_agents)]
                    joint_action[agent] = action
                    if not self.is_conflicting(joint_action):
                        expanded_states.append(self.result(joint_action))

            State._RNG.shuffle(expanded_states)
            return expanded_states

        action_set = list(Action)
        applicable_actions = [
            [action for action in action_set if self.is_applicable(agent, action)] for agent in range(num_agents)
        ]

        joint_action = [Action.NoOp for _ in range(num_agents)]
        actions_permutation = [0 for _ in range(num_agents)]
        expanded_states = []
        while True:
            for agent in range(num_agents):
                joint_action[agent] = applicable_actions[agent][actions_permutation[agent]]

            if not self.is_conflicting(joint_action):
                child = self.result(joint_action)
                pruned = False
                if ENABLE_SIMPLE_DEADLOCK and child.has_simple_deadlock():
                    State._simple_deadlock_count += 1
                    State._states_pruned += 1
                    pruned = True
                elif ENABLE_2BOX_DEADLOCK and child.has_2box_deadlock(child._moved_box_positions):
                    State._2box_deadlock_count += 1
                    State._states_pruned += 1
                    pruned = True
                if not pruned:
                    State._states_kept += 1
                    expanded_states.append(child)

            done = False
            for agent in range(num_agents):
                if actions_permutation[agent] < len(applicable_actions[agent]) - 1:
                    actions_permutation[agent] += 1
                    break
                else:
                    actions_permutation[agent] = 0
                    if agent == num_agents - 1:
                        done = True

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
            box_row = agent_row + action.agent_row_delta
            box_col = agent_col + action.agent_col_delta
            box = self.boxes[box_row][box_col]
            if not box:
                return False
            if State.box_colors[ord(box) - ord("A")] != agent_color:
                return False
            box_dest_row = box_row + action.box_row_delta
            box_dest_col = box_col + action.box_col_delta
            return self.is_free(box_dest_row, box_dest_col)

        if action.type is ActionType.Pull:
            agent_dest_row = agent_row + action.agent_row_delta
            agent_dest_col = agent_col + action.agent_col_delta
            if not self.is_free(agent_dest_row, agent_dest_col):
                return False
            box_row = agent_row - action.box_row_delta
            box_col = agent_col - action.box_col_delta
            box = self.boxes[box_row][box_col]
            if not box:
                return False
            return State.box_colors[ord(box) - ord("A")] == agent_color

        assert False, f"Not implemented for action type {action.type}."

    def is_conflicting(self, joint_action: list[Action]) -> bool:
        num_agents = len(self.agent_rows)

        dest_rows = [-1] * num_agents
        dest_cols = [-1] * num_agents
        box_dest_rows = [-1] * num_agents
        box_dest_cols = [-1] * num_agents

        for agent in range(num_agents):
            action = joint_action[agent]
            agent_row = self.agent_rows[agent]
            agent_col = self.agent_cols[agent]

            if action.type is ActionType.NoOp:
                pass

            elif action.type is ActionType.Move:
                dest_rows[agent] = agent_row + action.agent_row_delta
                dest_cols[agent] = agent_col + action.agent_col_delta

            elif action.type is ActionType.Push:
                dest_rows[agent] = agent_row + action.agent_row_delta
                dest_cols[agent] = agent_col + action.agent_col_delta
                box_dest_rows[agent] = dest_rows[agent] + action.box_row_delta
                box_dest_cols[agent] = dest_cols[agent] + action.box_col_delta

            elif action.type is ActionType.Pull:
                dest_rows[agent] = agent_row + action.agent_row_delta
                dest_cols[agent] = agent_col + action.agent_col_delta
                box_dest_rows[agent] = agent_row
                box_dest_cols[agent] = agent_col

        for a1 in range(num_agents):
            if joint_action[a1].type is ActionType.NoOp:
                continue

            for a2 in range(a1 + 1, num_agents):
                if joint_action[a2].type is ActionType.NoOp:
                    continue

                if dest_rows[a1] == dest_rows[a2] and dest_cols[a1] == dest_cols[a2]:
                    return True

                if box_dest_rows[a2] != -1:
                    if dest_rows[a1] == box_dest_rows[a2] and dest_cols[a1] == box_dest_cols[a2]:
                        return True

                if box_dest_rows[a1] != -1:
                    if dest_rows[a2] == box_dest_rows[a1] and dest_cols[a2] == box_dest_cols[a1]:
                        return True

                if box_dest_rows[a1] != -1 and box_dest_rows[a2] != -1:
                    if box_dest_rows[a1] == box_dest_rows[a2] and box_dest_cols[a1] == box_dest_cols[a2]:
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

    @classmethod
    def _compute_dead_cells(cls) -> dict[str, frozenset[tuple[int, int]]]:
        rows = len(cls.walls)
        cols = len(cls.walls[0]) if rows > 0 else 0

        def free(r: int, c: int) -> bool:
            return 0 <= r < rows and 0 <= c < cols and not cls.walls[r][c]

        MOVE_DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        PERP: dict[tuple[int, int], list[tuple[int, int]]] = {
            (-1, 0): [(0, -1), (0, 1)],
            (1,  0): [(0, -1), (0, 1)],
            (0, -1): [(-1, 0), (1, 0)],
            (0,  1): [(-1, 0), (1, 0)],
        }

        goal_positions: dict[str, list[tuple[int, int]]] = {}
        for r in range(rows):
            for c in range(cols):
                g = cls.goals[r][c]
                if "A" <= g <= "Z":
                    goal_positions.setdefault(g, []).append((r, c))

        dead: dict[str, frozenset[tuple[int, int]]] = {}
        for letter, gpos in goal_positions.items():
            reachable: set[tuple[int, int]] = set(gpos)
            queue: deque[tuple[int, int]] = deque(gpos)
            while queue:
                b_r, b_c = queue.popleft()
                for dr, dc in MOVE_DIRS:
                    a_r, a_c = b_r - dr, b_c - dc
                    if not free(a_r, a_c) or (a_r, a_c) in reachable:
                        continue
                    push_ok = free(a_r - dr, a_c - dc)
                    pull_straight_ok = free(b_r + dr, b_c + dc)
                    pull_perp_ok = any(free(b_r + pr, b_c + pc) for pr, pc in PERP[(dr, dc)])
                    if push_ok or pull_straight_ok or pull_perp_ok:
                        reachable.add((a_r, a_c))
                        queue.append((a_r, a_c))
            dead[letter] = frozenset(
                (r, c)
                for r in range(rows)
                for c in range(cols)
                if not cls.walls[r][c] and (r, c) not in reachable
            )
        return dead

    @classmethod
    def _compute_dead_pairs(cls) -> frozenset:
        return frozenset()

    @classmethod
    def _init_zobrist(cls, num_rows: int, num_cols: int) -> None:
        import random as _rng_mod
        rng = _rng_mod.Random(42)
        cls._zobrist_agents = [[rng.getrandbits(64) for _ in range(num_rows * num_cols)] for _ in range(10)]
        cls._zobrist_boxes = [[rng.getrandbits(64) for _ in range(num_rows * num_cols)] for _ in range(26)]

    def has_simple_deadlock(self) -> bool:
        dead = State._dead_cells
        for r in range(len(self.boxes)):
            for c in range(len(self.boxes[r])):
                box = self.boxes[r][c]
                if box and box in dead and (r, c) in dead[box]:
                    return True
        return False

    def has_2box_deadlock(self, moved_box_positions: list[tuple[int, int]]) -> bool:
        return False

    def __hash__(self) -> int:
        if self._hash is None:
            cols = len(State.walls[0]) if State.walls else 1
            h = State._static_hash
            for i, (r, c) in enumerate(zip(self.agent_rows, self.agent_cols)):
                h ^= State._zobrist_agents[i][r * cols + c]
            for r in range(len(self.boxes)):
                for c in range(len(self.boxes[r])):
                    b = self.boxes[r][c]
                    if b:
                        h ^= State._zobrist_boxes[ord(b) - ord('A')][r * cols + c]
            self._hash = h & 0xFFFFFFFFFFFFFFFF
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
