#!/bin/bash
# Run all Single-Agent (SA*) levels with GUI.
# Usage:  bash run_SA_gui.sh [strategy]

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LEVELS_DIR="$SCRIPT_DIR/levels"
SERVER="$SCRIPT_DIR/server.jar"
CLIENT_DIR="$SCRIPT_DIR/searchclient_python"
PYTHON="/opt/homebrew/bin/python3"
STRATEGY="${*}"
TIMEOUT=180
SPEED=150

SOLVED=0
FAILED=0
TOTAL=0
CURRENT=0

for LVL_FILE in "$LEVELS_DIR"/SA*.lvl; do
    [ -f "$LVL_FILE" ] || continue
    LEVEL=$(basename "$LVL_FILE" .lvl)
    TOTAL=$((TOTAL + 1))
done

CURRENT=0
for LVL_FILE in "$LEVELS_DIR"/SA*.lvl; do
    [ -f "$LVL_FILE" ] || continue
    LEVEL=$(basename "$LVL_FILE" .lvl)
    CURRENT=$((CURRENT + 1))

    echo ""
    echo "[$CURRENT/$TOTAL] Running: $LEVEL"
    echo "─────────────────────────────────────────"

    # Run the server in the background with a hard wall-clock cap of TIMEOUT+10s.
    # In GUI mode the server may keep its window open after a failed level; the
    # background killer ensures every level finishes within the time budget.
    OUTFILE=$(mktemp)
    (
        cd "$CLIENT_DIR" && \
        java -jar "$SERVER" \
            -l "$LVL_FILE" \
            -c "$PYTHON -m searchclient.searchclient $STRATEGY" \
            -g \
            -s "$SPEED" \
            -t "$TIMEOUT" \
            2>&1
    ) > "$OUTFILE" &
    JAVA_PID=$!
    # Killer: wait TIMEOUT+10 seconds then forcibly terminate if still running.
    ( sleep $((TIMEOUT + 10)); kill "$JAVA_PID" 2>/dev/null ) &
    KILLER_PID=$!
    wait "$JAVA_PID" 2>/dev/null
    kill "$KILLER_PID" 2>/dev/null
    wait "$KILLER_PID" 2>/dev/null
    OUTPUT=$(cat "$OUTFILE")
    rm -f "$OUTFILE"

    ACTIONS=$(echo "$OUTPUT" | grep "Actions used:" | sed 's/.*Actions used: //' | tr -d '.')
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
echo "  Single-Agent — $TOTAL levels"
echo "  Solved : $SOLVED"
echo "  Failed : $FAILED"
echo "═══════════════════════════════════════════"
