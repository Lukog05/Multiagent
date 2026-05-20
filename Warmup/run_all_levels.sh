#!/bin/bash
# Run all levels one at a time and print a summary table.
# Usage:  bash run_all_levels.sh [strategy]
# Strategy options: (blank) | -astar | -greedy | -wastar 5 | -bfs | -dfs
# Example: bash run_all_levels.sh -astar

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LEVELS_DIR="$SCRIPT_DIR/levels"
SERVER="$SCRIPT_DIR/server.jar"
CLIENT_DIR="$SCRIPT_DIR/searchclient_python"
PYTHON="/opt/homebrew/bin/python3"
STRATEGY="${*}"          # pass any flags through, e.g. -astar or -wastar 10
TIMEOUT=180              # seconds per level (matches competition 3-min limit)

SOLVED=0
FAILED=0
TOTAL=0

# Header
printf "\n%-40s %-8s %-8s %s\n" "Level" "Result" "Actions" "Time"
printf "%s\n" "$(printf '─%.0s' {1..75})"

for LVL_FILE in "$LEVELS_DIR"/*.lvl; do
    LEVEL=$(basename "$LVL_FILE" .lvl)
    TOTAL=$((TOTAL + 1))

    OUTPUT=$(
        cd "$CLIENT_DIR" && \
        java -jar "$SERVER" \
            -l "$LVL_FILE" \
            -c "$PYTHON -m searchclient.searchclient $STRATEGY" \
            -t "$TIMEOUT" \
            2>&1
    )

    SOLVED_LINE=$(echo "$OUTPUT" | grep "Level solved:")
    ACTIONS=$(echo "$OUTPUT" | grep "Actions used:" | sed 's/.*Actions used: //' | tr -d '.')
    TIME=$(echo "$OUTPUT" | grep -o 'Time to solve: [0-9.]*' | awk '{printf "%.3fs", $NF}')

    if echo "$SOLVED_LINE" | grep -q "Yes"; then
        SOLVED=$((SOLVED + 1))
        TAG="✓ SOLVED"
    else
        FAILED=$((FAILED + 1))
        TAG="✗ FAILED"
    fi

    printf "%-40s %-8s %-8s %s\n" "$LEVEL" "$TAG" "$ACTIONS" "$TIME"
done

# Footer
printf "%s\n" "$(printf '─%.0s' {1..75})"
printf "\nTotal: %d levels  |  Solved: %d  |  Failed: %d\n\n" \
    "$TOTAL" "$SOLVED" "$FAILED"
