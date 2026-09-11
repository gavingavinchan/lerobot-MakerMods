#!/usr/bin/env python

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

"""Bench the Star Arm 102-HD lock toggle on the leader alone -- no follower, no teleop loop.

The 102-HD (purple) leader can hold its own pose: servo stop-on-control mode 0x11 ("hold") on
every joint, 0x10 ("release") to let go again. This script drives that toggle and prints the
per-joint current so you can see the holding effort, which is the only feedback the arm gives
that the lock is doing anything.

Two ways to trigger it:

* ``--button-source=bus`` -- the arm's own lock button, a servo-protocol device at id 7. This is
  the real feature: the leader polls the button inside ``get_action`` and locks on the edge.
* ``--button-source=keyboard`` -- press SPACE (or ``l``) in this terminal. Needs no button
  hardware, so it works on a unit whose button board is silent or unwired, and it exercises the
  same ``leader.lock()`` / ``leader.release()`` calls.

Ctrl-C releases every joint and disconnects cleanly. Run it in the foreground and keep this
terminal focused (the keyboard source reads this TTY).

Keyboard source (works with no button board):

    uv run python examples/rebot_102/star_hd_lock_toggle.py \\
        --port=/dev/cu.wchusbserial110 --id=star_viscous --button-source=keyboard

Bus button source:

    uv run python examples/rebot_102/star_hd_lock_toggle.py \\
        --port=/dev/cu.wchusbserial110 --id=star_viscous --button-source=bus --button-id=7

The leader is light and this script never commands a position, so nothing here can move the arm:
locking holds where it already is, releasing lets it settle under gravity as usual. It does not
touch calibration -- it always connects with ``calibrate=False``, so a missing or mismatched
calibration file is a printed note, not an interactive prompt.
"""

import argparse
import logging
import time

from lerobot.teleoperators.rebot_102_leader import (
    RebotArm102Leader,
    RebotArm102LeaderTeleopConfig,
)
from lerobot.utils.keyboard_input import create_key_listener
from lerobot.utils.utils import init_logging

logger = logging.getLogger(__name__)

# Keys that flip the lock in `--button-source=keyboard`.
_TOGGLE_KEYS = ("space", "l")

# Delay after a lock change before re-sampling the current: the servos need a moment to build
# up (or shed) holding current, so a sample taken on the transition itself shows nothing.
_SETTLE_PRINT_SEC = 0.5


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", required=True, help="Leader serial port, e.g. /dev/cu.wchusbserial10")
    parser.add_argument("--id", default=None, help="Calibration id of the leader, e.g. star_viscous")
    parser.add_argument(
        "--button-source",
        choices=("bus", "keyboard"),
        default="keyboard",
        help="Where the toggle comes from: the arm's button board on the servo bus, or this terminal.",
    )
    parser.add_argument("--button-id", type=int, default=7, help="Servo-bus id of the button board.")
    parser.add_argument(
        "--hold-power-mw", type=int, default=0, help="Power cap for the hold command; 0 = servo default."
    )
    parser.add_argument(
        "--release-mode",
        choices=("release", "damping"),
        default="release",
        help="What 'unlocked' means: torque off (0x10) or free-but-resisting (0x12).",
    )
    parser.add_argument(
        "--damping-power-mw", type=int, default=900, help="Power cap used by --release-mode=damping."
    )
    parser.add_argument("--fps", type=float, default=30.0, help="Poll rate of the loop.")
    return parser.parse_args()


def _currents_ma(leader: RebotArm102Leader) -> dict[str, float | None]:
    """Per-joint current in mA, from one sync monitor packet (None for a silent servo)."""
    id_to_name = {v: k for k, v in leader.config.joint_ids.items()}
    monitors = leader.bus.sync_monitor(list(leader.config.joint_ids.values()))
    return {
        id_to_name[servo_id]: None if monitor is None else float(monitor.current_ma)
        for servo_id, monitor in monitors.items()
    }


def _print_status(leader: RebotArm102Leader, note: str) -> None:
    state = "LOCKED  " if leader.is_locked else "RELEASED"
    try:
        currents = _currents_ma(leader)
    except Exception as e:  # telemetry is a nicety here; never let it end the session
        print(f"[{state}] {note} (current read failed: {e})")
        return
    body = "  ".join(
        f"{name}={'--' if value is None else f'{value:.0f}'}" for name, value in currents.items()
    )
    print(f"[{state}] {note}  mA: {body}")


def main() -> None:
    args = _parse_args()
    init_logging()

    config = RebotArm102LeaderTeleopConfig(port=args.port, id=args.id)
    config.button.enabled = args.button_source == "bus"
    config.button.id = args.button_id
    config.button.hold_power_mw = args.hold_power_mw
    config.button.release_mode = args.release_mode
    config.button.damping_power_mw = args.damping_power_mw

    leader = RebotArm102Leader(config)
    if not leader.is_calibrated:
        print(
            f"No usable calibration for id={args.id!r}; connecting uncalibrated. That is fine here "
            "-- this script only reads angles and toggles the lock."
        )
    leader.connect(calibrate=False)

    toggle_requested = False

    def on_key(name: str) -> None:
        nonlocal toggle_requested
        if name in _TOGGLE_KEYS:
            toggle_requested = True

    listener = None
    if args.button_source == "keyboard":
        listener = create_key_listener(on_key, controls_help="SPACE or 'l' toggles the lock")
        if listener is None:
            raise SystemExit(
                "No keyboard backend available (not an interactive terminal?). Run this in a "
                "terminal, or use --button-source=bus."
            )
        print("Press SPACE (or 'l') to lock/release the arm. Ctrl-C to release and exit.")
    else:
        print(f"Polling the lock button at id={args.button_id}. Ctrl-C to release and exit.")

    period = 1.0 / args.fps
    was_locked = leader.is_locked
    resample_at: float | None = None
    _print_status(leader, "startup")

    try:
        while True:
            loop_start = time.perf_counter()

            # Reads the arm either way; with --button-source=bus this is also what polls the
            # button and applies the lock.
            leader.get_action()

            if toggle_requested:
                toggle_requested = False
                if leader.is_locked:
                    leader.release()
                else:
                    leader.lock()

            if leader.is_locked != was_locked:
                was_locked = leader.is_locked
                _print_status(leader, "toggled")
                resample_at = time.perf_counter() + _SETTLE_PRINT_SEC
            elif resample_at is not None and time.perf_counter() >= resample_at:
                resample_at = None
                _print_status(leader, f"settled +{_SETTLE_PRINT_SEC:.1f}s")

            elapsed = time.perf_counter() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        print("\nCtrl-C: releasing the arm.")
    finally:
        if listener is not None:
            listener.stop()
        if leader.is_connected:
            # disconnect() releases every joint on its way out; this only makes the release
            # visible in the status line before the port closes. The nested finally is the
            # point: a second Ctrl-C landing inside release() must not skip the disconnect,
            # which is what actually guarantees the joints end up free.
            try:
                leader.release()
                _print_status(leader, "released before exit")
            except Exception as e:
                logger.warning("Failed to release before disconnect: %s", e)
            finally:
                leader.disconnect()


if __name__ == "__main__":
    main()
