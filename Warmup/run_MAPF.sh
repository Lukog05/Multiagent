#!/bin/bash
# Run all Multi-Agent Path Finding (MAPF*) levels.
# Usage:  bash run_MAPF.sh [strategy]

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LEVELS_DIR="/Users/akhilkarnati/Desktop/Multiagent/Warmup/levels"
SERVER="/Users/akhilkarnati/Desktop/Multiagent/Warmup/server.jar/server.jar"
CLIENT_DIR="/Users/akhilkarnati/Desktop/Multiagent/Warmup/searchclient_python/searchclient/searchclient.py"
PYTHON="/opt/homebrew/bin/python3"
STRATEGY="${*}"
TIMEOUT=180

SOLVED=0
FAILED=0
TOTAL=0

printf "\n%-40s %-10s %-10s %s\n" "Level" "Result" "Actions" "Time"
printf "%s\n" "$(printf '─%.0s' {1..75})"

for LVL_FILE in "$LEVELS_DIR"/MAPF*.lvl; do
    [ -f "$LVL_FILE" ] || continue
    LEVEL=$(basename "$LVL_FILE" .lvl)
    TOTAL=$((TOTAL + 1))

    OUTPUT=$(
        cd "$(dirname "$CLIENT_DIR")" && \
        java -jar "$SERVER" \
            -l "$LVL_FILE" \
            -c "$PYTHON -m searchclient.searchclient $STRATEGY" \
            -t "$TIMEOUT" \
            2>&1
    )

    ACTIONS=$(echo "$OUTPUT" | grep "Actions used:" | sed 's/.*Actions used: //' | tr -d '.')
    TIME=$(echo "$OUTPUT" | grep -o 'Time to solve: [0-9.]*' | awk '{printf "%.3fs", $NF}')

    if echo "$OUTPUT" | grep -q "Level solved: Yes"; then
        SOLVED=$((SOLVED + 1))
        TAG="✓ SOLVED"
    else
        FAILED=$((FAILED + 1))
        TAG="✗ FAILED"
    fi

    printf "%-40s %-10s %-10s %s\n" "$LEVEL" "$TAG" "$ACTIONS" "$TIME"
done

printf "%s\n" "$(printf '─%.0s' {1..75})"
printf "\nMAPF: %d levels  |  Solved: %d  |  Failed: %d\n\n" \
    "$TOTAL" "$SOLVED" "$FAILED"
