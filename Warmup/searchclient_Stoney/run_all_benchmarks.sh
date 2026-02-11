#!/bin/bash
# Complete benchmark script for Exercise 4 - Goal Count Heuristic

cd /Users/hoskuldurthorsteinsson/Documents/GitHub/AI_and_MAS/searchclient/searchclient_python

LEVELS=("MAPF00.lvl" "MAPF01.lvl" "MAPF02.lvl" "MAPF02C.lvl" "MAPF03.lvl" "MAPF03C.lvl" "MAPFslidingpuzzle.lvl" "MAPFreorder2.lvl")
TIMEOUT=180

echo "========================================"
echo "Exercise 4: Goal Count Heuristic Benchmarks"
echo "========================================"
echo ""
echo "| Level | Algorithm | Expanded | Generated | Actions | Time (s) | Status |"
echo "|-------|-----------|----------|-----------|---------|----------|--------|"

for level in "${LEVELS[@]}"; do
    for algo in "astar" "greedy"; do
        # Run with timeout
        output=$(timeout $TIMEOUT java -jar ../server.jar -l ../levels/$level -c "python3 -m searchclient.searchclient -$algo" -g -s 300 -t $TIMEOUT 2>&1)
        exit_code=$?
        
        if [ $exit_code -eq 124 ]; then
            # Timeout
            echo "| $level | $algo | - | - | - | >180 | TIMEOUT |"
        else
            # Parse results
            expanded=$(echo "$output" | grep -o "Expanded:[[:space:]]*[0-9,]*" | tail -1 | grep -o "[0-9,]*" | tr -d ',')
            generated=$(echo "$output" | grep -o "Generated:[[:space:]]*[0-9,]*" | tail -1 | grep -o "[0-9,]*" | tr -d ',')
            actions=$(echo "$output" | grep "Actions used:" | grep -o "[0-9]*")
            time=$(echo "$output" | grep "Time to solve:" | grep -o "[0-9.]*" | head -1)
            solved=$(echo "$output" | grep "Level solved:" | grep -o "Yes\|No")
            
            if [ -z "$solved" ]; then
                solved="ERROR"
            fi
            
            # Format with defaults if empty
            expanded=${expanded:-"-"}
            generated=${generated:-"-"}
            actions=${actions:-"-"}
            time=${time:-"-"}
            
            echo "| $level | $algo | $expanded | $generated | $actions | $time | $solved |"
        fi
    done
done

echo ""
echo "========================================"
echo "Benchmark Complete!"
echo "========================================"
