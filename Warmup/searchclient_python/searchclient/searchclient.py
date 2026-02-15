import argparse
import sys
import time
from typing import TextIO

from searchclient import memory
from searchclient.color import Color
from searchclient.frontier import Frontier, FrontierBestFirst, FrontierBFS, FrontierDFS
from searchclient.graphsearch import search
from searchclient.heuristic import HeuristicAStar, HeuristicGreedy, HeuristicWeightedAStar
from searchclient.state import State


class SearchClient:
    @staticmethod
    def parse_level(server_messages: TextIO) -> State:
        # We can assume that the level file is conforming to specification, since the server verifies this.
        # Read domain.
        server_messages.readline()  # #domain
        server_messages.readline()  # hospital

        # Read Level name.
        server_messages.readline()  # #levelname
        server_messages.readline()  # <name>

        # Read colors.
        server_messages.readline()  # #colors
        agent_colors: list[Color | None] = [None for _ in range(10)]
        box_colors: list[Color | None] = [None for _ in range(26)]
        line = server_messages.readline()
        while not line.startswith("#"):
            split = line.split(":")
            color = Color.from_string(split[0].strip())
            entities = [e.strip() for e in split[1].split(",")]
            for e in entities:
                if "0" <= e <= "9":
                    agent_colors[ord(e) - ord("0")] = color
                elif "A" <= e <= "Z":
                    box_colors[ord(e) - ord("A")] = color
            line = server_messages.readline()

        # Read initial state.
        # line is currently "#initial".
        num_rows = 0
        num_cols = 0
        level_lines: list[str] = []
        line = server_messages.readline()
        while not line.startswith("#"):
            level_lines.append(line)
            num_cols = max(num_cols, len(line))
            num_rows += 1
            line = server_messages.readline()

        num_agents = 0
        agent_rows: list[int] = [-1 for _ in range(10)]
        agent_cols: list[int] = [-1 for _ in range(10)]
        walls = [[False for _ in range(num_cols)] for _ in range(num_rows)]
        boxes = [["" for _ in range(num_cols)] for _ in range(num_rows)]
        for row, line in enumerate(level_lines):
            for col, c in enumerate(line):
                if "0" <= c <= "9":
                    agent_rows[ord(c) - ord("0")] = row
                    agent_cols[ord(c) - ord("0")] = col
                    num_agents += 1
                elif "A" <= c <= "Z":
                    boxes[row][col] = c
                elif c == "+":
                    walls[row][col] = True
        del agent_rows[num_agents:]
        del agent_cols[num_agents:]

        # Read goal state.
        # line is currently "#goal".
        goals = [["" for _ in range(num_cols)] for _ in range(num_rows)]
        line = server_messages.readline()
        row = 0
        while not line.startswith("#"):
            for col, c in enumerate(line):
                if "0" <= c <= "9" or "A" <= c <= "Z":
                    goals[row][col] = c

            row += 1
            line = server_messages.readline()

        # End.
        # line is currently "#end".

        State.agent_colors = agent_colors
        State.walls = walls
        State.box_colors = box_colors
        State.goals = goals
        return State(agent_rows, agent_cols, boxes)

    @staticmethod
    def print_search_status(start_time: int, explored: set[State], frontier: Frontier) -> None:
        elapsed_time = time.perf_counter() - start_time
        print(
            f"#Expanded: {len(explored):8,}, #Frontier: {frontier.size():8,}, "
            f"#Generated: {len(explored) + frontier.size():8,}, Time: {elapsed_time:3.3f} s\n"
            f"[Alloc: {memory.get_usage():4.2f} MB, MaxAlloc: {memory.max_usage:4.2f} MB]",
            file=sys.stderr,
            flush=True,
        )

    @staticmethod
    def main(args: argparse.Namespace) -> None:
        # Use stderr to print to the console.
        print(
            "SearchClient initializing. I am sending this using the error output stream.", file=sys.stderr, flush=True
        )

        # Send client name to server.
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="ASCII")
        print("SearchClient", flush=True)

        # We can also print comments to stdout by prefixing with a #.
        print("#This is a comment.", flush=True)

        # Parse the level.
        server_messages = sys.stdin
        if hasattr(server_messages, "reconfigure"):
            server_messages.reconfigure(encoding="ASCII")
        initial_state = SearchClient.parse_level(server_messages)

        # Select search strategy.
        frontier: Frontier
        if args.bfs:
            frontier = FrontierBFS()
        elif args.dfs:
            frontier = FrontierDFS()
        elif args.astar:
            frontier = FrontierBestFirst(HeuristicAStar(initial_state))
        elif args.wastar is not False:
            frontier = FrontierBestFirst(HeuristicWeightedAStar(initial_state, args.wastar))
        elif args.greedy:
            frontier = FrontierBestFirst(HeuristicGreedy(initial_state))
        else:
            # Default to BFS search.
            frontier = FrontierBFS()
            print(
                "Defaulting to BFS search. Use arguments -bfs, -dfs, -astar, -wastar, or -greedy to set the search"
                " strategy.",
                file=sys.stderr,
                flush=True,
            )

        # Search for a plan.
        print(f"Starting {frontier.get_name()}.", file=sys.stderr, flush=True)
        plan = search(initial_state, frontier)

        # Print plan to server.
        if plan is None:
            print("Unable to solve level.", file=sys.stderr, flush=True)
            sys.exit(0)
        else:
            print(f"Found solution of length {len(plan)}.", file=sys.stderr, flush=True)

            for joint_action in plan:
                print("|".join(a.name_ for a in joint_action), flush=True)
                # We must read the server's response to not fill up the stdin buffer and block the server.
                _response = server_messages.readline()


if __name__ == "__main__":
    # Program arguments.
    parser = argparse.ArgumentParser(description="Simple client based on state-space graph search.")
    parser.add_argument(
        "--max-memory",
        metavar="<MB>",
        type=float,
        default=2048.0,
        help="The maximum memory usage allowed in MB (soft limit, default 2048).",
    )

    strategy_group = parser.add_mutually_exclusive_group()
    strategy_group.add_argument("-bfs", action="store_true", dest="bfs", help="Use the BFS strategy.")
    strategy_group.add_argument("-dfs", action="store_true", dest="dfs", help="Use the DFS strategy.")
    strategy_group.add_argument("-astar", action="store_true", dest="astar", help="Use the A* strategy.")
    strategy_group.add_argument(
        "-wastar",
        action="store",
        dest="wastar",
        nargs="?",
        type=int,
        default=False,
        const=5,
        help="Use the WA* strategy.",
    )
    strategy_group.add_argument("-greedy", action="store_true", dest="greedy", help="Use the Greedy strategy.")

    args = parser.parse_args()

    # Set max memory usage allowed (soft limit).
    memory.max_usage = args.max_memory

    # Run client.
    SearchClient.main(args)

"""
It reads the level from the server, selects a search strategy based on the program arguments, and then searches for a plan using that strategy. Finally, it sends the plan to the server.
It reads the level from the server through standard input (stdin)
  server_messages = sys.stdin  # ← Read from stdin (server sends level here)
The method reads the level line by line in parse_level method which processes the domain, level name, colors, initial state, and goal state until it encounters the "#end" line. The parsed information is used to create an initial State object that represents the starting configuration of the level.

Initial state is constructed in 2 parts:
    i) Class variables (shared across all states) like agent colors, walls, box colors, and goals are set directly on the State class.
    ii) Instance variables (specific to each state) like agent positions and box positions are set in the State constructor when creating the initial state object.

Invoking the search startegy:
 Select search strategy based on command-line arguments and then call the search function with the initial state and the selected frontier (which implements the search strategy). The search function will return a plan (sequence of joint actions) if a solution is found, or None if no solution exists.

It sends actions back to the server by printing them to standard output (stdout) in the required format. Each joint action is printed as a line where individual agent actions are separated by "|". After sending each joint action, it reads the server's response to ensure synchronization and prevent filling up the stdin buffer. 
"""
