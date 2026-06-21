#!/usr/bin/env bash
# serve_potree.sh — Export, convert, and serve a session's point cloud in Potree.
#
# Usage (from anywhere inside the livox_handheld_scanner project):
#   bash scripts/serve_potree.sh <session-name>            # export LAS + convert + serve
#   bash scripts/serve_potree.sh <session-name> --no-export  # skip LAS export
#   bash scripts/serve_potree.sh <session-name> --port=8088
#
# Requires: bash scripts/setup_potree.sh  (one-time)
# Requires: ROS2 sourced if doing LAS export (--no-export skips that requirement)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR="$REPO_ROOT/vendor/PotreeConverter"
CONVERTER="$VENDOR/PotreeConverter"
PORT=8087
NO_EXPORT=false

# --- arg parsing -------------------------------------------------------------
SESSION_ARG=""
for arg in "$@"; do
    case "$arg" in
        --no-export) NO_EXPORT=true ;;
        --port=*)    PORT="${arg#*=}" ;;
        *)           SESSION_ARG="$arg" ;;
    esac
done

if [ -z "$SESSION_ARG" ]; then
    echo "Usage: bash scripts/serve_potree.sh <session-name-or-path> [--no-export] [--port=8087]"
    exit 1
fi

# Resolve session: try as-is, then by name under the workspace sessions/ dir
WORKSPACE_SESSIONS="$(realpath "$REPO_ROOT/../..")/sessions"
if [ -d "$SESSION_ARG" ]; then
    SESSION_DIR="$(realpath "$SESSION_ARG")"
elif [ -d "$WORKSPACE_SESSIONS/$SESSION_ARG" ]; then
    SESSION_DIR="$(realpath "$WORKSPACE_SESSIONS/$SESSION_ARG")"
elif [ -d "$WORKSPACE_SESSIONS/$(basename "$SESSION_ARG")" ]; then
    SESSION_DIR="$(realpath "$WORKSPACE_SESSIONS/$(basename "$SESSION_ARG")")"
else
    echo "ERROR: session not found: $SESSION_ARG"
    echo "  Available sessions:"
    ls "$WORKSPACE_SESSIONS" 2>/dev/null | sed 's/^/    /' || echo "    (none)"
    exit 1
fi

SESSION_NAME="$(basename "$SESSION_DIR")"
LAS_PATH="$SESSION_DIR/pointcloud.las"
POTREE_OUT="$SESSION_DIR/potree"

# --- checks ------------------------------------------------------------------
if [ ! -x "$CONVERTER" ]; then
    echo "ERROR: PotreeConverter not found. Run: bash scripts/setup_potree.sh"
    exit 1
fi
if [ ! -d "$VENDOR/resources/page_template/libs" ]; then
    echo "ERROR: Potree viewer resources missing. Run: bash scripts/setup_potree.sh"
    exit 1
fi

# --- export LAS if needed ----------------------------------------------------
if [ "$NO_EXPORT" = false ] && [ ! -f "$LAS_PATH" ]; then
    echo "Exporting point cloud …"
    source ~/ros2_ws/install/setup.bash
    python3 "$REPO_ROOT/scripts/export_pointcloud.py" "$SESSION_DIR"
fi

if [ ! -f "$LAS_PATH" ]; then
    echo "ERROR: $LAS_PATH not found. Run without --no-export."
    exit 1
fi

# --- convert to Potree format ------------------------------------------------
if [ ! -d "$POTREE_OUT" ]; then
    echo "Converting to Potree octree format …"
    LD_LIBRARY_PATH="$VENDOR" "$CONVERTER" \
        "$LAS_PATH" \
        -o "$POTREE_OUT" \
        -p index \
        --title "$SESSION_NAME"
else
    echo "Potree data already exists at $POTREE_OUT"
fi

# --- serve -------------------------------------------------------------------
echo ""
echo "Serving at http://localhost:$PORT"
echo "Open that URL in your browser. Ctrl+C to stop."
echo ""
python3 -m http.server "$PORT" --directory "$POTREE_OUT"
