#!/bin/bash
# Run all Multi-Agent (MA*) levels, excluding MAPF.
# Usage:  bash run_MA.sh [strategy]

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LEVELS_DIR="$SCRIPT_DIR/levels"
SERVER="$SCRIPT_DIR/server.jar"
CLIENT_DIR="$SCRIPT_DIR/searchclient_python"
PYTHON="/opt/homebrew/bin/python3"
STRATEGY="${*}"
TIMEOUT=180

SOLVED=0
FAILED=0
TOTAL=0

printf "\n%-40s %-10s %-10s %s\n" "Level" "Result" "Actions" "Time"
printf "%s\n" "$(printf '─%.0s' {1..75})"

for LVL_FILE in "$LEVELS_DIR"/MA*.lvl; do
    [ -f "$LVL_FILE" ] || continue
    LEVEL=$(basename "$LVL_FILE" .lvl)
    # Skip MAPF levels
    [[ "$LEVEL" == MAPF* ]] && continue
    TOTAL=$((TOTAL + 1))

    OUTPUT=$(
        cd "$CLIENT_DIR" && \
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
printf "\nMulti-Agent: %d levels  |  Solved: %d  |  Failed: %d\n\n" \
    "$TOTAL" "$SOLVED" "$FAILED"
