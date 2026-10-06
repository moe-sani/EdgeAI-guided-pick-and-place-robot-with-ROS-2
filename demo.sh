#!/usr/bin/env bash
# One-command demo: LLM server + simulation + scene + web UI, opened in Firefox.
# Run on the robot PC:   ~/ros2_ws/demo.sh            (Ctrl+C stops everything it started)
#
# Options:
#   --safety       start with the camera safety check ON
#   --logo         show the ABB logo on the robot (hidden by default)
#   --lan          let other machines on the network open the web UI (http://<this-PC-IP>:8080)
#   --https        serve the web UI over HTTPS (needed for the microphone from another machine)
#   --no-browser   don't open Firefox
#   --port N       web UI port (default 8080)
set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGS="$WS/log/demo"
PORT=8080
SAFETY="" LOGO="false" HOST="127.0.0.1" HTTPS="" BROWSER=1
while [ $# -gt 0 ]; do
    case "$1" in
        --safety) SAFETY="--safety" ;;
        --logo) LOGO="true" ;;
        --lan) HOST="0.0.0.0" ;;
        --https) HTTPS="--https" ;;
        --no-browser) BROWSER=0 ;;
        --port) PORT="$2"; shift ;;
        -h|--help) sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "Unknown option: $1 (see --help)"; exit 1 ;;
    esac
    shift
done

mkdir -p "$LOGS"
# shellcheck disable=SC1091
set +u  # ROS setup scripts use unset variables
source "$WS/env.sh" >/dev/null 2>&1 || { echo "Could not source $WS/env.sh (is the workspace built?)"; exit 1; }
set -u

GENIEX="$(command -v geniex || echo "$HOME/.local/bin/geniex")"
MODEL="$(sed -n 's/^ *model: *//p' "$WS/src/llm_commander/config/scene.yaml" | head -1)"
SCHEME=$([ -n "$HTTPS" ] && echo https || echo http)
URL="$SCHEME://localhost:$PORT"
STARTED=()   # process-group ids we started (stopped again on exit)
declare -A NAME=()  # process-group id -> component name
LAST_PG=""

say()  { printf '\033[1;34m▶\033[0m %s\n' "$*"; }
ok()   { printf '  \033[1;32m✓\033[0m %s\n' "$*"; }
warn() { printf '  \033[1;33m!\033[0m %s\n' "$*"; }
fail() { printf '  \033[1;31m✗\033[0m %s\n' "$*"; exit 1; }

# Start a command in its own session, so it (and everything it spawns, e.g. Gazebo) can be stopped as a group.
start_bg() {  # start_bg <name> <command...>; sets LAST_PG (don't call it inside $(...): that's a subshell)
    local name=$1; shift
    setsid "$@" >"$LOGS/$name.log" 2>&1 </dev/null &
    LAST_PG=$!
    STARTED+=("$LAST_PG")
    NAME[$LAST_PG]=$name
}

forget() {  # drop a stopped process group from STARTED
    local pg=$1 keep=() p
    for p in "${STARTED[@]}"; do [ "$p" != "$pg" ] && keep+=("$p"); done
    STARTED=("${keep[@]}")
}

stop_group() {  # SIGINT (like Ctrl+C), then SIGTERM, then SIGKILL
    local pg=$1
    kill -INT -- "-$pg" 2>/dev/null || return 0
    for _ in $(seq 1 20); do kill -0 -- "-$pg" 2>/dev/null || return 0; sleep 0.5; done
    kill -TERM -- "-$pg" 2>/dev/null
    for _ in $(seq 1 10); do kill -0 -- "-$pg" 2>/dev/null || return 0; sleep 0.5; done
    kill -KILL -- "-$pg" 2>/dev/null
}

cleanup() {
    trap - INT TERM EXIT
    [ ${#STARTED[@]} -eq 0 ] && exit 0
    echo; say "Stopping the demo..."
    local i
    for (( i=${#STARTED[@]}-1; i>=0; i-- )); do stop_group "${STARTED[$i]}"; done  # reverse start order
    ok "Stopped. Logs: $LOGS"
    exit 0
}
trap cleanup INT TERM EXIT

wait_for() {  # wait_for <seconds> <description> <test command...>
    local secs=$1 what=$2; shift 2
    for _ in $(seq 1 "$secs"); do "$@" >/dev/null 2>&1 && return 0; sleep 1; done
    return 1
}

sim_ready() { ros2 action list 2>/dev/null | grep -q '^/Robmove$' && grep -q "You can start planning" "$LOGS/sim.log"; }

echo
echo "  Robot Arm · LLM Commander demo"
echo "  ──────────────────────────────"

# 1. LLM server (GenieX) -----------------------------------------------------------------------------
say "LLM server (GenieX)"
if curl -s -m 2 http://127.0.0.1:18181/v1/models >/dev/null; then
    ok "already running"
else
    [ -x "$GENIEX" ] || fail "geniex not found"
    start_bg geniex "$GENIEX" serve --skip-update
    wait_for 60 "GenieX" curl -s -m 2 http://127.0.0.1:18181/v1/models || fail "GenieX did not start (see $LOGS/geniex.log)"
    ok "started"
fi
# Load the model now (in the background) so the first command isn't slow.
curl -s -m 120 http://127.0.0.1:18181/v1/chat/completions -H 'Content-Type: application/json' \
    -d "{\"model\": \"$MODEL\", \"max_tokens\": 1, \"messages\": [{\"role\": \"user\", \"content\": \"hi\"}]}" >/dev/null 2>&1 &

# 2. Simulation -----------------------------------------------------------------------------------------
say "Simulation (Gazebo + MoveIt)"
if pgrep -f "gz sim server" >/dev/null && ros2 action list 2>/dev/null | grep -q '^/Robmove$'; then
    ok "already running"
else
    pgrep -f "gz sim server" >/dev/null && fail "Gazebo is running but the robot isn't ready. Close Gazebo and run this again."
    for attempt in 1 2; do
        start_bg sim ros2 launch ros2srrc_launch moveit2.launch.py package:=irb120cranfield \
                 config:=irb120cranfield_2 rviz:=false logo:=$LOGO
        sim_pg=$LAST_PG
        if wait_for 150 "simulation" sim_ready; then ok "started (Gazebo window opens on this screen)"; break; fi
        # Gazebo occasionally hangs while loading the world; one automatic retry fixes it.
        stop_group "$sim_pg"; forget "$sim_pg"
        [ $attempt -eq 2 ] && fail "Simulation did not come up (see $LOGS/sim.log)"
        warn "Gazebo got stuck while starting; retrying once..."
    done
    sleep 3  # let the controllers settle
fi

# 3. Scene (trays, cubes, safety camera) --------------------------------------------------------------
say "Scene (trays, cubes, camera)"
ros2 run llm_commander spawn_scene --reset >"$LOGS/scene.log" 2>&1 && ok "ready" || warn "spawn_scene reported a problem (see $LOGS/scene.log)"

# 4. Web UI (commander) ---------------------------------------------------------------------------------
say "Web UI"
if ss -ltn 2>/dev/null | grep -q ":$PORT "; then
    warn "something is already serving port $PORT; using it (start with --port N for a fresh one)"
else
    start_bg commander ros2 run llm_commander commander --web --host "$HOST" --port "$PORT" $SAFETY $HTTPS
    wait_for 30 "web UI" bash -c "ss -ltn | grep -q ':$PORT '" || fail "the web UI did not start (see $LOGS/commander.log)"
    ok "$URL${SAFETY:+  (safety check ON)}"
fi
[ "$HOST" = "0.0.0.0" ] && ok "from other machines: $SCHEME://$(hostname -I | awk '{print $1}'):$PORT"

# 5. Browser ---------------------------------------------------------------------------------------------
if [ $BROWSER -eq 1 ]; then
    if command -v firefox >/dev/null; then (setsid firefox --new-tab "$URL" >/dev/null 2>&1 &)
    else (setsid xdg-open "$URL" >/dev/null 2>&1 &); fi
    ok "opened $URL in the browser"
fi

echo
echo "  Demo is running. Press Ctrl+C here to stop everything this script started."
echo "  Logs: $LOGS"
echo

# Stay in the foreground; stop if the commander or the simulation dies (e.g. Gazebo window closed).
while sleep 2; do
    for pg in "${STARTED[@]}"; do
        if ! kill -0 "$pg" 2>/dev/null; then
            warn "${NAME[$pg]} stopped$([ "${NAME[$pg]}" = sim ] && echo ' (Gazebo window closed?)')"
            cleanup
        fi
    done
done
