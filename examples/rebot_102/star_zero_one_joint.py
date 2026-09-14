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

"""Re-zero ONE Star leader servo at its current pose (e.g. after reassembling that joint).

`lerobot-calibrate` for the leader sets the origin of every servo at once; this does the same
unlock -> set_origin_point sequence for a single joint and leaves the other six untouched. It
never talks to the follower. The calibration JSON is not involved: the leader stores its zero in
the servo, and the file only carries homing_offset=0 and the joint ranges.

Modes:
  (default)  print the joint's raw angle once; changes nothing.
  --watch    free the joint and print its raw angle live (and how far it is from --target) while
             you turn it by hand. Ctrl-C to stop. Commands no position.
  --apply    set the joint's current pose as its zero. Hold it still at the target first.

The zero pose is the follower's reference pose, matched by eye joint for joint (docs/source/
viscous.mdx step 4) -- NOT a point derived from the end stops. The captured Star data holds only
travel, so the stops say nothing about where zero sits between them; on 2026-09-14 a stop-derived
zero for shoulder_pan landed ~27 deg away from the visually matched pose.
"""

import argparse
import time

from motorbridge_smart_servo import FashionStarServo

from lerobot.teleoperators.rebot_102_leader.config_rebot_102_leader_viscous import (
    RebotArm102LeaderViscousConfig,
)
from lerobot.teleoperators.rebot_102_leader.rebot_102_leader import _SETTLE_SEC

p = argparse.ArgumentParser()
p.add_argument("--port", required=True)
p.add_argument("--joint", required=True)
p.add_argument("--target", type=float, default=0.0, help="raw angle the zero pose should be at")
mode = p.add_mutually_exclusive_group()
mode.add_argument("--watch", action="store_true")
mode.add_argument("--apply", action="store_true")
a = p.parse_args()

cfg = RebotArm102LeaderViscousConfig(port=a.port)
if a.joint not in cfg.joint_ids:
    raise SystemExit(f"unknown joint {a.joint!r}; choose from {list(cfg.joint_ids)}")
servo_id = cfg.joint_ids[a.joint]
factor = cfg.joint_directions[a.joint]
bus = FashionStarServo(a.port, baudrate=cfg.baudrate)


def raw_angle() -> float:
    mon = bus.sync_monitor([servo_id]).get(servo_id)
    if mon is None:
        raise RuntimeError(f"servo {a.joint} (id={servo_id}) did not respond")
    return mon.angle_deg


try:
    if not bus.ping(servo_id):
        raise SystemExit(f"servo {a.joint} (id={servo_id}) not found on {a.port}")

    if a.watch:
        bus.unlock(servo_id)  # free the joint so it turns by hand
        print(f"watching {a.joint} (id={servo_id}); target raw {a.target:.1f} deg. Ctrl-C to stop.")
        try:
            while True:
                raw = raw_angle()
                off = raw - a.target
                hint = "HOLD HERE" if abs(off) <= 1.0 else ("turn +" if off < 0 else "turn -")
                print(f"\rraw={raw:8.1f}  off by {off:+7.1f} deg  {hint:<10}", end="", flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print()
        raise SystemExit

    before = raw_angle()
    print(
        f"{a.joint} (id={servo_id}) raw={before:.1f} deg (off target by {before - a.target:+.1f}) "
        f"-> follower target ~{before * factor:.1f} deg"
    )
    if not a.apply:
        print("(read-only; pass --apply to set this pose as the joint's zero)")
        raise SystemExit
    bus.unlock(servo_id)
    time.sleep(_SETTLE_SEC)
    bus.set_origin_point(servo_id)
    time.sleep(_SETTLE_SEC)
    bus.reset_multi_turn(servo_id)
    time.sleep(_SETTLE_SEC)
    # The open connection keeps reporting the pre-origin angle; only a fresh one reads the new zero.
    bus.close()
    bus = FashionStarServo(a.port, baudrate=cfg.baudrate)
    after = raw_angle()
    print(f"origin set: {a.joint} raw {before:.1f} -> {after:.1f} deg (fresh connection)")
finally:
    bus.close()
