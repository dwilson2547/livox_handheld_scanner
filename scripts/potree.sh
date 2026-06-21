#!/usr/bin/env bash
# potree.sh — manage the Potree point cloud viewer
#
# Commands:
#   potree.sh                   — show status and list sessions
#   potree.sh start             — start viewer for the most recent session
#   potree.sh start <name>      — start viewer for a named session (fuzzy match)
#   potree.sh stop              — stop the running viewer
#   potree.sh status            — show what's running
#   potree.sh list              — list all sessions and their point cloud status
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSIONS_DIR="$(realpath "$REPO_ROOT/../..")/sessions"
VENDOR="$REPO_ROOT/vendor/PotreeConverter"
CONVERTER="$VENDOR/PotreeConverter"
PID_FILE="/tmp/potree_scanner.pid"
SESSION_FILE="/tmp/potree_scanner.session"
PORT=8087
CMD="${1:-}"

# --------------------------------------------------------------------------- #
#  helpers
# --------------------------------------------------------------------------- #

_is_running() {
    [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null
}

_stop() {
    if _is_running; then
        kill "$(cat "$PID_FILE")" 2>/dev/null || true
        sleep 0.5
        kill -9 "$(cat "$PID_FILE")" 2>/dev/null || true
        echo "Stopped."
    else
        echo "Not running."
    fi
    rm -f "$PID_FILE" "$SESSION_FILE"
}

_resolve_session() {
    local query="${1:-}"
    local best=""

    if [ -z "$query" ]; then
        # No argument: pick most recent session with a pointcloud.las
        best=$(find "$SESSIONS_DIR" -maxdepth 2 -name "pointcloud.las" \
            | sed 's|/pointcloud.las||' | sort | tail -1)
        if [ -z "$best" ]; then
            # Fall back to most recent session regardless
            best=$(ls -dt "$SESSIONS_DIR"/*/  2>/dev/null | head -1 | sed 's|/$||')
        fi
        echo "$best"
        return
    fi

    # Exact dir
    if [ -d "$query" ]; then echo "$(realpath "$query")"; return; fi
    if [ -d "$SESSIONS_DIR/$query" ]; then echo "$SESSIONS_DIR/$query"; return; fi

    # Fuzzy: match sessions whose name starts with or contains the query
    local matches
    matches=$(ls "$SESSIONS_DIR" 2>/dev/null | grep -i "$query" | sort -r || true)
    local count
    count=$(echo "$matches" | grep -c . 2>/dev/null || echo 0)

    if [ "$count" -eq 0 ]; then
        echo "ERROR: no session matching '$query'" >&2
        echo "  Run: bash scripts/potree.sh list" >&2
        exit 1
    fi
    if [ "$count" -eq 1 ]; then
        echo "$SESSIONS_DIR/$matches"
        return
    fi

    # Multiple matches — pick the most recent (last alphabetically, since names end in timestamp)
    local picked
    picked=$(echo "$matches" | tail -1)
    echo "$SESSIONS_DIR/$picked"
}

_ensure_converted() {
    local session="$1"
    local las="$session/pointcloud.las"
    local potree="$session/potree"

    if [ ! -f "$las" ]; then
        echo "No pointcloud.las for $(basename "$session")."
        echo "Process the session first, or run:"
        echo "  source ~/ros2_ws/install/setup.bash"
        echo "  python3 $REPO_ROOT/scripts/export_pointcloud.py $(basename "$session")"
        exit 1
    fi

    if [ ! -d "$potree" ]; then
        if [ ! -x "$CONVERTER" ]; then
            echo "ERROR: PotreeConverter not found. Run: bash scripts/setup_potree.sh" >&2
            exit 1
        fi
        echo "Converting to Potree format …"
        LD_LIBRARY_PATH="$VENDOR" "$CONVERTER" "$las" -o "$potree" \
            -p index --title "$(basename "$session")" 2>&1 \
            | grep -v "^WARN\|^#\|throughput\|duration\|output\|cubicAABB\|total file" \
            || true
    fi
    echo "$potree"
}

# --------------------------------------------------------------------------- #
#  commands
# --------------------------------------------------------------------------- #

case "$CMD" in

stop)
    _stop
    ;;

status)
    if _is_running; then
        session="$(cat "$SESSION_FILE" 2>/dev/null || echo "unknown")"
        echo "Running — $(basename "$session")  →  http://localhost:$PORT"
        echo "PID: $(cat "$PID_FILE")"
    else
        echo "Not running."
    fi
    ;;

list)
    echo "Sessions:"
    for d in $(ls -dt "$SESSIONS_DIR"/*/ 2>/dev/null); do
        name=$(basename "${d%/}")
        has_las=""
        has_potree=""
        [ -f "$d/pointcloud.las" ]  && has_las=" [las]"
        [ -d "$d/potree" ]          && has_potree=" [potree]"
        echo "  $name$has_las$has_potree"
    done
    ;;

start)
    session=$(_resolve_session "${2:-}")
    if [ -z "$session" ] || [ ! -d "$session" ]; then
        echo "ERROR: no sessions found in $SESSIONS_DIR" >&2
        exit 1
    fi
    echo "Session: $(basename "$session")"

    # Stop any existing server first
    if _is_running; then
        echo "Stopping existing server …"
        _stop
    fi

    potree_dir=$(_ensure_converted "$session")

    python3 -m http.server "$PORT" --directory "$potree_dir" \
        > /tmp/potree_scanner.log 2>&1 &
    echo $! > "$PID_FILE"
    echo "$session" > "$SESSION_FILE"

    sleep 1
    if ! _is_running; then
        echo "ERROR: server failed to start. Log:" >&2
        cat /tmp/potree_scanner.log >&2
        exit 1
    fi

    echo "Serving at http://localhost:$PORT  (PID $(cat "$PID_FILE"))"
    echo "To stop: bash scripts/potree.sh stop"
    ;;

"")
    # No command: show status then list
    if _is_running; then
        session="$(cat "$SESSION_FILE" 2>/dev/null || echo "unknown")"
        echo "Running — $(basename "$session")  →  http://localhost:$PORT  (PID $(cat "$PID_FILE"))"
    else
        echo "Not running."
    fi
    echo ""
    bash "$0" list
    echo ""
    echo "Usage: bash scripts/potree.sh start [session-name]"
    ;;

*)
    # Treat unknown arg as a session name shorthand for 'start'
    session=$(_resolve_session "$CMD")
    exec bash "$0" start "$session"
    ;;

esac
