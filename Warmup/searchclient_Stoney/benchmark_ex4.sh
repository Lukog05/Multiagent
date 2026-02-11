#!/bin/bash
# Benchmark script for Exercise 4
# Tests BFS, A*, and Greedy on required levels

LEVELS=(
    "MAPF00.lvl"
    "MAPF01.lvl"
    "MAPF02.lvl"
    "MAPF02C.lvl"
    "MAPF03.lvl"
    "MAPF03C.lvl"
    "MAPFslidingpuzzle.lvl"
    "MAPFreorder2.lvl"
)

ALGORITHMS=("bfs" "astar" "greedy")

cd /Users/hoskuldurthorsteinsson/Documents/GitHub/AI_and_MAS/searchclient/searchclient_python

echo "========================================="
echo "Exercise 4: Benchmark with Goal Count Heuristic"
echo "========================================="
echo ""

for level in "${LEVELS[@]}"; do
    echo "----------------------------------------"
    echo "Level: $level"
    echo "----------------------------------------"
    
    for algo in "${ALGORITHMS[@]}"; do
        echo ""
        echo "Algorithm: $algo"
        
        # Run the search client with a timeout
        output=$(timeout 180 java -jar ../server.jar -l ../levels/$level -c "python3 -m searchclient.searchclient -$algo" -g -s 300 -t 180 2>&1)
        
        if [ $? -eq 124 ]; then
            echo "TIMEOUT (180s)"
        else
            # Extract the final expanded count
            expanded=$(echo "$output" | grep "Expanded:" | tail -1 | awk -F'[: ,]+' '{for(i=1;i<=NF;i++) if($i=="Expanded") print $(i+1)}')
            generated=$(echo "$output" | grep "Generated:" | tail -1 | awk -F'[: ,]+' '{for(i=1;i<=NF;i++) if($i=="Generated") print $(i+1)}')
            actions=$(echo "$output" | grep "Actions used:" | awk '{print $4}')
            time=$(echo "$output" | grep "Time to solve:" | awk '{print $4}')
            solved=$(echo "$output" | grep "Level solved:" | awk '{print $4}')
            
            echo "  Solved: $solved"
            echo "  Expanded: $expanded"
            echo "  Generated: $generated"
            echo "  Actions: $actions"
            echo "  Time: ${time}s"
        fi
        echo ""
    done
    echo ""
done
