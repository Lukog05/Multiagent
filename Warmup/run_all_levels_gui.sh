#!/bin/bash
# Run all levels one at a time with the GUI, waiting for each to finish before moving on.
# Usage:  bash run_all_levels_gui.sh [strategy]
# Example: bash run_all_levels_gui.sh -astar

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LEVELS_DIR="$SCRIPT_DIR/levels"
SERVER="$SCRIPT_DIR/server.jar"
CLIENT_DIR="$SCRIPT_DIR/searchclient_python"
PYTHON="/tmp/mas_venv/bin/python3"
STRATEGY="${*}"   # optional strategy flag, e.g. -astar or -greedy
TIMEOUT=180       # seconds per level
SPEED=150         # ms per action in the GUI

TOTAL=$(ls "$LEVELS_DIR"/*.lvl | wc -l | tr -d ' ')
CURRENT=0
SOLVED=0
FAILED=0

for LVL_FILE in "$LEVELS_DIR"/*.lvl; do
    LEVEL=$(basename "$LVL_FILE" .lvl)
    CURRENT=$((CURRENT + 1))

    echo ""
    echo "[$CURRENT/$TOTAL] Running: $LEVEL"
    echo "─────────────────────────────────────────"

    OUTPUT=$(
        cd "$CLIENT_DIR" && \
        java -jar "$SERVER" \
            -l "$LVL_FILE" \
            -c "$PYTHON -m searchclient.searchclient $STRATEGY" \
            -g \
            -s "$SPEED" \
            -t "$TIMEOUT" \
            2>&1
    )

    ACTIONS=$(echo "$OUTPUT" | grep "Actions used:"  | sed 's/.*Actions used: //' | tr -d '.')
    TIME=$(echo "$OUTPUT" | grep -o 'Time to solve: [0-9.]*' | awk '{printf "%.3fs", $NF}')

    if echo "$OUTPUT" | grep -q "Level solved: Yes"; then
        SOLVED=$((SOLVED + 1))
        echo "  Result  : ✓ SOLVED"
    else
        FAILED=$((FAILED + 1))
        echo "  Result  : ✗ FAILED"
    fi

    echo "  Actions : $ACTIONS"
    echo "  Time    : $TIME"
done

echo ""
echo "═══════════════════════════════════════════"
echo "  Done — $TOTAL levels"
echo "  Solved : $SOLVED"
echo "  Failed : $FAILED"
echo "═══════════════════════════════════════════"
