#!/usr/bin/env bash

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# One-command teleop for the Viscous Arm follower driven by the Star Arm 102 leader.
#
#   examples/viscous/teleop.sh start [extra lerobot-teleoperate args...]
#   examples/viscous/teleop.sh stop
#   examples/viscous/teleop.sh status
#
# `start` finds both serial ports, refuses to launch without both calibration files or while a
# session is already running, starts lerobot-teleoperate in the background with its log in the
# state dir, keeps the host awake while it runs, watches the log for heat and power trouble
# (desktop notifications), and waits until the follower tracks the leader. It prints one line
# either way; a failure comes with a hint. `stop` sends SIGINT and confirms both arms disconnected
# cleanly. `status` reports on the running session.
#
# The arm has no brakes: every way a session ends (`stop`, the follower's thermal stop, a crash, a
# power cut) releases torque and it drops from wherever it is. Park it folded with the leader
# before `stop`. See docs/source/viscous_teleop.mdx.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OS="$(uname -s)"

ROBOT_ID="${VISCOUS_ROBOT_ID:-viscous_01}"
TELEOP_ID="${VISCOUS_TELEOP_ID:-star_viscous}"
TELEOP_BIN="${VISCOUS_TELEOP_BIN:-$REPO/.venv/bin/lerobot-teleoperate}"
NOTIFY="${VISCOUS_NOTIFY:-1}"
START_TIMEOUT_S="${VISCOUS_START_TIMEOUT_S:-30}"
STOP_TIMEOUT_S=5
# Same resolution order as `HF_LEROBOT_CALIBRATION` in src/lerobot/utils/constants.py.
CALIBRATION_DIR="${HF_LEROBOT_CALIBRATION:-${HF_LEROBOT_HOME:-${HF_HOME:-${XDG_CACHE_HOME:-$HOME/.cache}/huggingface}/lerobot}/calibration}"
STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/lerobot/viscous_teleop"
PIDFILE="$STATE_DIR/teleop.pid"
LATEST_LOG="$STATE_DIR/latest.log"
NOTIFICATIONS_LOG="$STATE_DIR/notifications.log"

READY_LINE='synced to leader; tracking at full speed'
LEADER_DOWN='RebotArm102Leader disconnected.'
FOLLOWER_DOWN='ViscousFollower disconnected.'
N_JOINTS=7
ESC=$'\033'

usage() {
    cat <<EOF
Usage: examples/viscous/teleop.sh start [extra lerobot-teleoperate args...]
       examples/viscous/teleop.sh stop
       examples/viscous/teleop.sh status

  start   Launch teleop in the background and wait until the follower tracks the leader.
          Fold the leader to its zero pose first: the follower moves to the leader's pose.
  stop    Park the arm folded with the leader FIRST (it has no brakes), then run this.
          Sends SIGINT and checks that both arms disconnected cleanly.
  status  Running or not, uptime, loop rate, latest temperatures, warning/error counts.

Environment [default]:
  VISCOUS_ROBOT_ID         follower calibration id [viscous_01]
  VISCOUS_TELEOP_ID        leader calibration id [star_viscous]
  VISCOUS_CAN_PORT         follower CANable port [auto-detected]
  VISCOUS_STAR_PORT        Star leader port [auto-detected]
  VISCOUS_TELEOP_BIN       teleop entry point [<repo>/.venv/bin/lerobot-teleoperate]
  VISCOUS_NOTIFY           1 = desktop notifications, 0 = only log them [1]
  VISCOUS_START_TIMEOUT_S  seconds to wait for tracking [30]
  HF_LEROBOT_CALIBRATION   calibration root [~/.cache/huggingface/lerobot/calibration]
  XDG_STATE_HOME           state dir parent [~/.local/state]

State, session logs and notifications: $STATE_DIR
EOF
}

die() {
    echo "teleop.sh: $*" >&2
    exit 1
}

warn() {
    echo "teleop.sh: WARNING: $*" >&2
}

# The teleop log carries cursor-up escapes and per-tick "Teleop loop time" fragments glued onto the
# next log line with no newline in between. Strip both so log lines can be matched from the start.
clean_log() {
    sed -e "s/${ESC}\[1A//g" -e 's/Teleop loop time: [0-9.]*ms ([0-9]* Hz)//g' "$1"
}

last_exception() {
    clean_log "$1" | grep -aE '^[A-Za-z_][A-Za-z0-9_.]*(Error|Exception)(: |$)' | tail -n 1 || true
}

alive() {
    local stat
    stat="$(ps -p "$1" -o stat= 2>/dev/null)" || return 1
    stat="${stat// /}"
    [[ -n $stat && $stat != Z* ]]
}

is_viscous_teleop() {
    ps -p "$1" -o command= 2>/dev/null | grep -q -- '--robot.type=viscous_follower'
}

pidfile_pid() {
    if [[ -f $PIDFILE ]]; then sed -n 1p "$PIDFILE"; fi
}

pidfile_log() {
    if [[ -f $PIDFILE ]]; then sed -n 2p "$PIDFILE"; fi
}

# Pid of the running session: the one this script started, else one started some other way. For
# the latter only the interpreter process counts, never a shell or `uv run` wrapper whose command
# line merely mentions teleop.
running_pid() {
    local pid
    pid="$(pidfile_pid)"
    if [[ -n $pid ]] && alive "$pid" && is_viscous_teleop "$pid"; then
        echo "$pid"
        return 0
    fi
    ps -Ao pid=,command= \
        | awk '$2 ~ /(^|\/)(python[0-9.]*|lerobot-teleoperate)$/ && /--robot\.type=viscous_follower/ {print $1; exit}' \
        || true
}

# find_port <env var> <label> <glob>...: the env var if set, else exactly one glob match.
find_port() {
    local env_name=$1 label=$2
    shift 2
    local value="${!env_name:-}"
    if [[ -n $value ]]; then
        [[ -e $value ]] || die "$env_name=$value does not exist."
        echo "$value"
        return 0
    fi
    local candidates=() pattern path
    for pattern in "$@"; do
        for path in $pattern; do
            if [[ -e $path ]]; then candidates+=("$path"); fi
        done
    done
    case ${#candidates[@]} in
        1) echo "${candidates[0]}" ;;
        0) die "no $label found (looked for: ${*:-nothing on $OS}). Plug it in, or set $env_name." ;;
        *) die "more than one $label candidate: ${candidates[*]}. Set $env_name to the right one." ;;
    esac
}

detect_ports() {
    case $OS in
        Linux)
            CAN_PORT="$(find_port VISCOUS_CAN_PORT "follower CANable" '/dev/serial/by-id/usb-Openlight_Labs_CANable2*')"
            STAR_PORT="$(find_port VISCOUS_STAR_PORT "Star leader (CH340)" '/dev/serial/by-id/usb-1a86_USB_Serial*')"
            ;;
        Darwin)
            CAN_PORT="$(find_port VISCOUS_CAN_PORT "follower CANable" '/dev/cu.usbmodem*')"
            STAR_PORT="$(find_port VISCOUS_STAR_PORT "Star leader (CH340)" '/dev/cu.wchusbserial*')"
            ;;
        *)
            CAN_PORT="$(find_port VISCOUS_CAN_PORT "follower CANable")"
            STAR_PORT="$(find_port VISCOUS_STAR_PORT "Star leader")"
            ;;
    esac
}

check_calibration() {
    local follower="$CALIBRATION_DIR/robots/viscous_follower/$ROBOT_ID.json"
    local leader="$CALIBRATION_DIR/teleoperators/rebot_102_leader/$TELEOP_ID.json"
    local restore="cp -Rn $REPO/examples/calibration/. $CALIBRATION_DIR/"
    if [[ ! -f $follower ]]; then
        die "no follower calibration at $follower. Starting without it would re-zero every motor at" \
            "its current pose and destroy the arm's calibration. NEVER run lerobot-calibrate for the" \
            "follower: check VISCOUS_ROBOT_ID, or restore the committed file with: $restore"
    fi
    if [[ ! -f $leader ]]; then
        die "no leader calibration at $leader. Check VISCOUS_TELEOP_ID, or restore the committed" \
            "file with: $restore"
    fi
}

# notify <normal|critical> <title> <body>: always logged to the notifications log.
notify() {
    local urgency=$1 title=$2 body=$3
    mkdir -p "$STATE_DIR"
    printf '%s [%s] %s: %s\n' "$(date '+%F %T')" "$urgency" "$title" "$body" >>"$NOTIFICATIONS_LOG"
    [[ $NOTIFY == 1 ]] || return 0
    case $OS in
        Linux)
            command -v notify-send >/dev/null 2>&1 || return 0
            DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=/run/user/$UID/bus}" \
                notify-send -a "Viscous teleop" -u "$urgency" "$title" "$body" >/dev/null 2>&1 || true
            ;;
        Darwin)
            osascript -e 'on run argv' -e 'display notification (item 2 of argv) with title (item 1 of argv)' \
                -e 'end run' "$title" "$body" >/dev/null 2>&1 || true
            ;;
    esac
}

# Strip a log line down to its message: no level, timestamp or source location.
message_of() {
    local line
    line="$(sed -E 's/^(INFO|WARNING|ERROR) [0-9-]+ [0-9:]+ [^ ]+ //' <<<"$1")"
    echo "${line:0:250}"
}

# Follow a session's log until its process exits, turning the lines that need the operator into
# desktop notifications. Polls by byte offset rather than `tail -f`, which would outlive the
# process on a log that has stopped growing.
watch_session() {
    local pid=$1 log=$2 offset=0 size chunk line
    local last_warn=-100 last_critical=-100
    while alive "$pid"; do
        sleep 1
        size="$(wc -c <"$log" 2>/dev/null | tr -d ' ')" || continue
        [[ -n $size && $size -gt $offset ]] || continue
        chunk="$(tail -c +"$((offset + 1))" "$log" | head -c "$((size - offset))" \
            | sed -e "s/${ESC}\[1A//g" -e 's/Teleop loop time: [0-9.]*ms ([0-9]* Hz)//g')" || true
        offset=$size
        if line="$(grep -aE 'thermal stop' <<<"$chunk" | tail -n 1)" && [[ -n $line ]]; then
            notify critical "Viscous teleop: THERMAL STOP" "$(message_of "$line")"
        fi
        if line="$(grep -aF 'no CAN feedback from any motor' <<<"$chunk" | tail -n 1)" && [[ -n $line ]]; then
            notify critical "Viscous teleop: MOTOR POWER LOST" "$(message_of "$line")"
        fi
        if line="$(grep -aE 'is at [0-9.]+ C, over the .*critical limit' <<<"$chunk" | tail -n 1)" \
            && [[ -n $line ]] && ((SECONDS - last_critical >= 10)); then
            last_critical=$SECONDS
            notify critical "Viscous teleop: motor critically hot" "$(message_of "$line")"
        elif line="$(grep -aE 'is at [0-9.]+ C, over the' <<<"$chunk" | tail -n 1)" \
            && [[ -n $line ]] && ((SECONDS - last_warn >= 10)); then
            last_warn=$SECONDS
            notify normal "Viscous teleop: motor hot" "$(message_of "$line")"
        fi
    done

    local marker="$STATE_DIR/stop_requested.$pid" reason
    if [[ -e $marker ]]; then
        rm -f "$marker"
        return 0
    fi
    reason="$(last_exception "$log")"
    reason="${reason:-the process exited without an exception in its log}"
    reason="${reason%.}"
    if grep -aqF "$READY_LINE" "$log"; then
        notify critical "Viscous teleop ENDED" "$reason. Torque is off and the arm has dropped."
    else
        notify critical "Viscous teleop failed to start" "$reason"
    fi
}

explain_failure() {
    local log=$1 reason hint="" missing count first
    reason="$(last_exception "$log")"
    echo "FAILED: ${reason:-teleop exited without an exception in its log}"
    case $reason in
        *"did not respond"*)
            missing="$(grep -oE "did not respond: \[[^]]*\]" <<<"$reason" | head -n 1)" || true
            count="$(grep -o "'[a-z_]*'" <<<"$missing" | wc -l | tr -d ' ')" || true
            first="$(grep -o "'[a-z_]*'" <<<"$missing" | head -n 1 | tr -d "'")" || true
            if ((count >= N_JOINTS)); then
                hint="no motor answered: motor power is off or tripped (or the CAN adapter is not wired to the arm)."
            else
                hint="the CAN link is broken before $first: check the cable (and power) into $first."
            fi
            ;;
        *"leader is not at its zero pose"*)
            hint="fold the Star leader to its zero pose (the follower's folded pose), then start again."
            ;;
        *"Servo not found"*)
            hint="the Star leader did not answer: check its USB cable and its power."
            ;;
        *"thermal stop"*)
            hint="the motors are too hot: leave the arm folded to cool before the next session."
            ;;
        *"no CAN feedback from any motor"*)
            hint="motor power dropped: check the supply and the arm's power cable."
            ;;
    esac
    if [[ -z $hint ]] && grep -aq "Permission denied" "$log"; then
        hint="no permission on a serial port: add yourself to the dialout group (sudo usermod -aG dialout \$USER), then log in again."
    fi
    if [[ -n $hint ]]; then echo "hint: $hint"; fi
    echo "log: $log"
}

last_outcome() {
    local log reason
    log="$(readlink "$LATEST_LOG" 2>/dev/null)" || return 0
    [[ -f $log ]] || return 0
    reason="$(last_exception "$log")"
    if [[ -n $reason ]]; then
        echo "last session ended with: $reason"
    elif grep -aqF "$FOLLOWER_DOWN" "$log"; then
        echo "last session ended cleanly at $(grep -aF "$FOLLOWER_DOWN" "$log" | tail -n 1 | awk '{print $2, $3}')"
    fi
    echo "last log: $log"
}

cmd_start() {
    [[ -x $TELEOP_BIN ]] || die "$TELEOP_BIN is not executable. Install with: uv sync --extra maker --extra rebot"
    local pid log
    pid="$(running_pid)"
    [[ -z $pid ]] || die "a Viscous teleop session is already running (pid $pid). Stop it first: $0 stop"
    check_calibration
    detect_ports

    mkdir -p "$STATE_DIR"
    # The script's pid keeps two sessions started within the same second on separate logs.
    log="$STATE_DIR/teleop_$(date +%Y%m%d_%H%M%S)_$$.log"
    ln -sfn "$log" "$LATEST_LOG"

    # How the session is launched, and why (verified on hardware 2026-08-29 and 2026-09-15):
    # * A background job started by a non-interactive shell inherits SIGINT *ignored*. Python then
    #   never installs its KeyboardInterrupt handler, `kill -INT` is silently dropped, and the
    #   `finally: robot.disconnect()` in lerobot_teleoperate.py is unreachable: only SIGTERM stops
    #   it, skipping the clean shutdown. `trap - INT` in the subshell restores the default before
    #   exec, so `stop`'s SIGINT is honoured.
    # * exec all the way down, with no `uv run` or other wrapper in front, keeps `$!` the Python
    #   process itself, so the pidfile holds the pid that must receive SIGINT. A wrapper such as
    #   `systemd-inhibit cmd` ignores SIGINT and does not forward it, which is why the sleep
    #   inhibitor below is a separate process watching this pid.
    # * `setsid` (where available) gives the session its own session, so a Ctrl-C in the terminal
    #   that ran `start` cannot reach it.
    local detach=()
    if command -v setsid >/dev/null 2>&1; then detach=(setsid); fi
    (
        trap - INT
        exec ${detach[@]+"${detach[@]}"} nohup "$TELEOP_BIN" \
            --robot.type=viscous_follower --robot.port="$CAN_PORT" --robot.id="$ROBOT_ID" \
            --teleop.type=rebot_102_leader_viscous --teleop.port="$STAR_PORT" --teleop.id="$TELEOP_ID" \
            "$@" </dev/null >"$log" 2>&1
    ) &
    pid=$!
    disown "$pid" 2>/dev/null || true
    printf '%s\n%s\n' "$pid" "$log" >"$PIDFILE"

    # The host must not sleep: on 2026-08-29 an idle sleep froze the loop, the motors' 200 ms CAN
    # watchdog released torque and the arm dropped. A lid-close sleep on battery is not covered.
    case $OS in
        Linux)
            if command -v systemd-inhibit >/dev/null 2>&1; then
                nohup systemd-inhibit --what=idle:sleep --why="Viscous teleop (pid $pid)" \
                    tail --pid="$pid" -f /dev/null </dev/null >/dev/null 2>&1 &
                disown $! 2>/dev/null || true
            else
                warn "systemd-inhibit not found: nothing stops this host sleeping mid-session, which drops the arm."
            fi
            ;;
        Darwin)
            if command -v caffeinate >/dev/null 2>&1; then
                nohup caffeinate -i -w "$pid" </dev/null >/dev/null 2>&1 &
                disown $! 2>/dev/null || true
            else
                warn "caffeinate not found: nothing stops this host sleeping mid-session, which drops the arm."
            fi
            ;;
    esac

    (
        trap '' HUP
        watch_session "$pid" "$log"
    ) </dev/null >/dev/null 2>&1 &
    disown $! 2>/dev/null || true

    if [[ $OS == Linux ]]; then
        local i sigign
        for ((i = 0; i < 20; i++)); do
            if is_viscous_teleop "$pid" || ! alive "$pid"; then break; fi
            sleep 0.1
        done
        sigign="$(awk '/^SigIgn:/ {print $2}' "/proc/$pid/status" 2>/dev/null)" || true
        if [[ -n $sigign ]] && (((16#$sigign & 2) != 0)); then
            warn "pid $pid IGNORES SIGINT: '$0 stop' cannot end this session cleanly. Park the arm" \
                "and stop it with 'kill -TERM $pid' (no clean disconnect), then fix the launch."
        fi
    fi

    local deadline=$((SECONDS + START_TIMEOUT_S))
    while :; do
        if grep -aqF "$READY_LINE" "$log" 2>/dev/null; then
            echo "tracking (pid $pid, log $log)"
            return 0
        fi
        if ! alive "$pid"; then
            rm -f "$PIDFILE"
            explain_failure "$log"
            return 1
        fi
        if ((SECONDS >= deadline)); then
            echo "still starting after ${START_TIMEOUT_S} s and NOT stopped (pid $pid, log $log). Last lines:"
            clean_log "$log" | grep -av '^[[:space:]]*$' | tail -n 8 || true
            return 2
        fi
        sleep 0.2
    done
}

cmd_stop() {
    local pid log="" i
    pid="$(running_pid)"
    if [[ -z $pid ]]; then
        echo "not running."
        rm -f "$PIDFILE"
        last_outcome
        return 0
    fi
    if [[ $(pidfile_pid) == "$pid" ]]; then log="$(pidfile_log)"; fi

    mkdir -p "$STATE_DIR"
    touch "$STATE_DIR/stop_requested.$pid"
    kill -INT "$pid"
    for ((i = 0; i < STOP_TIMEOUT_S * 10; i++)); do
        if ! alive "$pid"; then break; fi
        sleep 0.1
    done

    if alive "$pid"; then
        echo "SIGINT was not honoured: pid $pid is still running after ${STOP_TIMEOUT_S} s."
        echo "NOT escalating: a forced kill skips the clean disconnect, and the arm has no brakes."
        echo "Once the arm is parked or supported, and you accept that, run: kill -TERM $pid"
        return 1
    fi
    rm -f "$PIDFILE"
    if [[ -z $log ]]; then
        echo "stopped (pid $pid was not started by this script, so there is no log to confirm a clean disconnect)."
        return 0
    fi
    if grep -aqF "$LEADER_DOWN" "$log" && grep -aqF "$FOLLOWER_DOWN" "$log"; then
        echo "stopped cleanly (log $log)"
        return 0
    fi
    echo "teleop exited, but its log does not show both arms disconnecting cleanly. Last lines:"
    clean_log "$log" | grep -av '^[[:space:]]*$' | tail -n 8 || true
    return 1
}

cmd_status() {
    local pid log="" loop temps warnings errors last_problem
    pid="$(running_pid)"
    if [[ -z $pid ]]; then
        echo "not running."
        last_outcome
        return 3
    fi
    if [[ $(pidfile_pid) == "$pid" ]]; then log="$(pidfile_log)"; fi
    echo "running (pid $pid, up $(ps -p "$pid" -o etime= | tr -d ' '))"
    if [[ -z $log || ! -f $log ]]; then
        echo "log: unknown (not started by this script)"
        return 0
    fi
    loop="$(grep -aoE 'Teleop loop time: [0-9.]+ms \([0-9]+ Hz\)' "$log" | tail -n 1)" || true
    temps="$(clean_log "$log" | grep -aF 'temperatures (C)' | tail -n 1 \
        | sed -E 's/^INFO [0-9-]+ ([0-9:]+) .*temperatures \(C\): /\1  /')" || true
    warnings="$(clean_log "$log" | grep -acE '^WARNING ')" || true
    errors="$(clean_log "$log" | grep -acE '^ERROR ')" || true
    last_problem="$(clean_log "$log" | grep -aE '^(WARNING|ERROR) ' | tail -n 1)" || true
    echo "loop: ${loop#Teleop loop time: }"
    echo "temperatures (C): ${temps:-none logged yet}"
    echo "warnings: ${warnings:-0}, errors: ${errors:-0}"
    if [[ -n $last_problem ]]; then echo "last: ${last_problem:0:250}"; fi
    echo "log: $log"
}

case "${1:-}" in
    start)
        shift
        cmd_start "$@"
        ;;
    stop) cmd_stop ;;
    status) cmd_status ;;
    -h | --help | help) usage ;;
    *)
        usage >&2
        exit 64
        ;;
esac
