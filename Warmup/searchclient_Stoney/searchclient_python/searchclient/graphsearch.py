import sys
import time

from searchclient import memory
from searchclient.action import Action
from searchclient.frontier import Frontier
from searchclient.state import State

start_time = time.perf_counter()


def search(initial_state: State, frontier: Frontier) -> tuple[list[list[Action]] | None, dict]:
    output_fixed_solution = False

    if output_fixed_solution:
        # Part 1:
        # The agents will perform the sequence of actions returned by this method.
        # Try to solve a few levels by hand, enter the found solutions below, and run them:

        return [
            [Action.MoveS],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveS],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveS],
            [Action.MoveS],


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
            return None, {}

        # Your code here...



        # Check if frontier is empty (failure)
        if frontier.is_empty():
            print("No solution found.", file=sys.stderr, flush=True)
            return None, {}

        # Pop a state from the frontier
        state = frontier.pop()

        # Check if this is a goal state
        if state.is_goal_state():
            # Print final statistics
            print_search_status(explored, frontier)
            # Extract and return the plan with metrics
            metrics = {
                'explored': len(explored),
                'generated': len(explored) + frontier.size(),
                'alloc': memory.get_usage()
            }
            return state.extract_plan(), metrics

        # Add state to explored set
        explored.add(state)

        # Expand the state (get all successors)
        for child_state in state.get_expanded_states():
            # Only add child if not already explored and not in frontier
            if child_state not in explored and not frontier.contains(child_state):
                frontier.add(child_state)



def print_search_status(explored: set[State], frontier: Frontier) -> None:
    elapsed_time = time.perf_counter() - start_time
    print(
        f"#Expanded: {len(explored):8,}, #Frontier: {frontier.size():8,}, "
        f"#Generated: {len(explored) + frontier.size():8,}, Time: {elapsed_time:3.3f} s\n"
        f"[Alloc: {memory.get_usage():4.2f} MB, MaxAlloc: {memory.max_usage:4.2f} MB]",
        file=sys.stderr,
        flush=True,
    )
