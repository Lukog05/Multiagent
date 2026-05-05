#!/bin/bash
# Generic suite runner: run_suite.sh <PATTERN> <CLIENT> <LABEL>
# e.g.: run_suite.sh "SA*.lvl" "run_current.py" "SA"
LEVELS_DIR="$(cd "$(dirname "$0")" && pwd)/levels"
SERVER="$(cd "$(dirname "$0")" && pwd)/server.jar"
CLIENT_DIR="$(cd "$(dirname "$0")" && pwd)/searchclient_python"
PYTHON="/opt/homebrew/bin/python3"
PATTERN="$1"
CLIENT="$2"
LABEL="$3"
TIMEOUT=180

SOLVED=0; FAILED=0; TOTAL=0; TOTAL_ACTIONS=0
printf "\n%-40s %-10s %-10s %s\n" "Level" "Result" "Actions" "Time"
printf "%s\n" "$(printf '─%.0s' {1..75})"

for LVL_FILE in "$LEVELS_DIR"/$PATTERN; do
    [ -f "$LVL_FILE" ] || continue
    LEVEL=$(basename "$LVL_FILE" .lvl); TOTAL=$((TOTAL + 1))
    OUTPUT=$(cd "$CLIENT_DIR" && java -jar "$SERVER" -l "$LVL_FILE" \
        -c "$PYTHON $CLIENT" -t "$TIMEOUT" 2>&1)
    ACTIONS=$(echo "$OUTPUT" | grep "Actions used:" | sed 's/.*Actions used: //' | tr -d '.' | tr -d ',')
    TIME=$(echo "$OUTPUT" | grep -o 'Time to solve: [0-9.]*' | awk '{printf "%.3fs", $NF}')
    if echo "$OUTPUT" | grep -q "Level solved: Yes"; then
        SOLVED=$((SOLVED + 1)); TAG="✓ SOLVED"
        [ -n "$ACTIONS" ] && TOTAL_ACTIONS=$((TOTAL_ACTIONS + ACTIONS))
    else
        FAILED=$((FAILED + 1)); TAG="✗ FAILED"
    fi
    printf "%-40s %-10s %-10s %s\n" "$LEVEL" "$TAG" "$ACTIONS" "$TIME"
done

printf "%s\n" "$(printf '─%.0s' {1..75})"
printf "\n%s: %d levels  |  Solved: %d  |  Failed: %d  |  Total actions: %d\n\n" \
    "$LABEL" "$TOTAL" "$SOLVED" "$FAILED" "$TOTAL_ACTIONS"
