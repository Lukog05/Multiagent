from enum import Enum, unique
from typing import Literal #Literal is used to specify that a variable can only take on specific literal values, in this case -1, 0, or 1. 
#This is useful for type checking and ensuring that the values passed to the Action class are valid.


@unique # Decorator to ensure all enum values are unique
class ActionType(Enum):
    NoOp = 0
    Move = 1
#Push =2
#Pull = 3

@unique
class Action(Enum):
    #   List of possible actions. Each action has the following parameters,
    #    taken in order from left to right:
    #    1. The name of the action as a string. This is the string sent to the server
    #    when the action is executed. Note that for Pull and Push actions the syntax is
    #    "Push(X,Y)" and "Pull(X,Y)" with no spaces.
    #    2. Action type: NoOp, Move, Push or Pull (only NoOp and Move initially supported)
    #    3. agentRowDelta: the vertical displacement of the agent (-1,0,+1) #-1 means agent moves up (N), +1 means agent moves down (S)
    #    4. agentColDelta: the horisontal displacement of the agent (-1,0,+1) #-1 means agent moves left (W), +1 means agent moves right (E)
    #    5. boxRowDelta: the vertical displacement of the box (-1,0,+1) #-1 means box moves up (N), +1 means box moves down (S)
    #    6. boxColDelta: the horisontal discplacement of the box (-1,0,+1) #-1 means box moves left (W), +1 means box moves right (E)
    #    Note: Origo (0,0) is in the upper left corner. So +1 in the vertical direction is down (S)
    #    and +1 in the horisontal direction is right (E).
    NoOp = ("NoOp", ActionType.NoOp, 0, 0, 0, 0)

    MoveN = ("Move(N)", ActionType.Move, -1, 0, 0, 0)
    MoveS = ("Move(S)", ActionType.Move, 1, 0, 0, 0)
    MoveE = ("Move(E)", ActionType.Move, 0, 1, 0, 0)
    MoveW = ("Move(W)", ActionType.Move, 0, -1, 0, 0)
    
    #PushN, PushS, PushE, PushW
    #PullN, PullS, PullE, PullW


    def __init__(
        self, #self is Action class itself, and the parameters are the values defined in the enum members above.
        name: str, #name is the string representation of the action, such as "Move(N)" for moving north. This is the string that will be sent to the server when the action is executed.
        type: ActionType, #type is the type of action, which can be either NoOp or Move (and later Push or Pull). This is used to categorize the action and determine how it should be executed.
        ard: Literal[-1, 0, 1], #ard stands for agentRowDelta, which is the vertical displacement of the agent. It can take on values of -1, 0, or 1, where -1 means the agent moves up (north), 0 means no vertical movement, and 1 means the agent moves down (south).
        acd: Literal[-1, 0, 1],
        brd: Literal[-1, 0, 1],
        bcd: Literal[-1, 0, 1],
    ) -> None:
        self.name_ = name
        self.type = type
        self.agent_row_delta = ard  # horisontal displacement agent (-1,0,+1)
        self.agent_col_delta = acd  # vertical displacement agent (-1,0,+1)
        self.box_row_delta = brd  # horisontal displacement box (-1,0,+1)
        self.box_col_delta = bcd  # vertical displacement box (-1,0,+1)

#Somewhere in other codefiles this from action import Action is called
#Imports in this code as imported
#class ActionType(enum) is created. So when you want to for instance call NoOp, you can call Action.NoOp and it will return the tuple ("NoOp", ActionType.NoOp, 0, 0, 0, 0) which is the name of the action, its type, and the deltas for the agent and box movements.
#Now, class Action(enum) is created
#First, enum memebers are defined say for NoOp
#Then, the __init__ method is defined which takes in the parameters for each action and assigns them to instance variables.
"""
# Automatically called:
Action.NoOp.__init__(
    "NoOp",           # name
    ActionType.NoOp,  # type
    0,                # ard (agent_row_delta)
    0,                # acd (agent_col_delta)
    0,                # brd (box_row_delta)
    0                 # bcd (box_col_delta)
)
"""
#Inside __init__ method, the attributes are set.
"""
self.name_ = "NoOp"
self.type = ActionType.NoOp
self.agent_row_delta = 0
self.agent_col_delta = 0
self.box_row_delta = 0
self.box_col_delta = 0
"""
#And it is stored in the memory.
#Remaining move actions also created now one by one.
"""
# In any other file that imports:
from action import Action, ActionType

# You can now use:
Action.NoOp          # Access the enum member
Action.MoveN         # Access the enum member
Action.MoveN.name_   # "Move(N)"
Action.MoveN.type    # ActionType.Move
Action.MoveN.agent_row_delta  # -1
"""