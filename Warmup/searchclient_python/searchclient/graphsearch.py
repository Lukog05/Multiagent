import sys
import time

from searchclient import memory
from searchclient.action import Action
from searchclient.frontier import Frontier
from searchclient.state import State

start_time = time.perf_counter()


def search(initial_state: State, frontier: Frontier) -> list[list[Action]] | None:
    output_fixed_solution = False

    if output_fixed_solution:
        # Part 1:
        # The agents will perform the sequence of actions returned by this method.
        # Try to solve a few levels by hand, enter the found solutions below, and run them:
        return [
            [Action.MoveS], #Goes south from 1,1 to 2,1
            [Action.MoveS],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveS],
            [Action.MoveS]
        ]

    # Part 2:
    # Now try to implement the Graph-Search algorithm from R&N figure 3.7
    # In the case of "failure to find a solution" you should return None.
    # Some useful methods on the state class which you will need to use are:
    # state.is_goal_state() - Returns true if the state is a goal state.
    # state.extract_plan() - Returns the list of actions used to reach this state.
    # state.get_expanded_states() - Returns a list containing the states reachable from the current state.
    # You should also take a look at frontier.py to see which methods the Frontier interface exposes
    #
    # print_search_status(expanded, frontier): As you can see below, the code will print out status
    # (#expanded states, size of the frontier, #generated states, total time used) for every 1000th node
    # generated.
    # You should also make sure to print out these stats when a solution has been found, so you can keep
    # track of the exact total number of states generated!!

    iterations = 0

    frontier.add(initial_state)
    explored: set[State] = set()

    while True:
        iterations += 1
        if iterations % 1000 == 0:
            print_search_status(explored, frontier)

        if memory.get_usage() > memory.max_usage:
            print_search_status(explored, frontier)
            print("Maximum memory usage exceeded.", file=sys.stderr, flush=True)
            return None

        # Your code here...
        #Belw code added for Ex2, point3
###############################################################################        
        #Checks if frontier is empty, if it is then there is no solution and we return None
        if frontier.is_empty():
            print_search_status(explored, frontier)
            print("Frontier is empty. No solution found.", file=sys.stderr, flush=True)
            return None
        
        #Pop next state from frontier to explore
        state = frontier.pop()
        
        #Goal test - if the state is a goal state, we return the plan to reach it
        if state.is_goal_state():
            print_search_status(explored, frontier)
            print("Solution found.", file=sys.stderr, flush=True)
            return state.extract_plan()
        
        
        #Add the state to the explored set
        explored.add(state)
        
        #Expand the state and add the new states to the frontier if they haven't been explored or aren't already in the frontier
        for child_state in state.get_expanded_states():
            if child_state not in explored and not frontier.contains(child_state):
                frontier.add(child_state)
##############################################################################

def print_search_status(explored: set[State], frontier: Frontier) -> None:
    elapsed_time = time.perf_counter() - start_time
    print(
        f"#Expanded: {len(explored):8,}, #Frontier: {frontier.size():8,}, "
        f"#Generated: {len(explored) + frontier.size():8,}, Time: {elapsed_time:3.3f} s\n"
        f"[Alloc: {memory.get_usage():4.2f} MB, MaxAlloc: {memory.max_usage:4.2f} MB]",
        file=sys.stderr,
        flush=True,
    )
    print(f'#Explored: {len(explored)}', flush=True)
    print(f"#Generated: {len(explored) + frontier.size()}", flush=True)
    print(f"#Alloc: {memory.get_usage():.2f} MB", flush=True)
    
    
"""
main search loop is in the while True: loop in the search function. You should implement the graph search algorithm there, using the Frontier and State classes. The Frontier class will manage the states that are yet to be explored, while the State class will represent the current state of the problem and provide methods to check if it's a goal state, extract the plan to reach it, and get its expanded states.
Currently returns a HARDCODED solution:
    A list of joint actions (one per timestep)
    Each joint action is a list of actions (one per agent)
    For single agent: each inner list has 1 action
    This is just a dummy/placeholder - doesn't actually solve anything!
the hardcoded solution can be changed under if output_fixed_solution: block in the search function.

Need to implement:
i) turn off the hardcoded solution by setting output_fixed_solution to False
ii) complete the graph search algorithm in the while True: loop in the search function, using the Frontier and State classes.

"""

"""
Solution for hardcode MAPF00.lvl:
Move(S)  ← Step 1
Move(S)  ← Step 2
Move(S)  ← Step 3
Move(S)  ← Step 4
Move(E)  ← Step 5
Move(E)  ← Step 6
Move(E)  ← Step 7
Move(E)  ← Step 8
Move(E)  ← Step 9
Move(E)  ← Step 10
Move(E)  ← Step 11
Move(E)  ← Step 12
Move(E)  ← Step 13
Move(E)  ← Step 14

"""