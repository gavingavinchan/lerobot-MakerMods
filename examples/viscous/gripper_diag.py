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

"""Viscous follower gripper diagnostic.

Phase 1 (always): torque stays OFF on every motor. Reads position / torque / MOS temperature
of all seven motors and prints them. Never touches calibration.

Phase 2 (--actuate): enables torque on the GRIPPER ONLY, steps it `--travel` degrees toward
open (negative) in 2 deg steps at 20 Hz, prints position and torque each step, steps back,
then disables the gripper's torque. The arm must be folded at rest with hands clear.
"""

import argparse
import time

from lerobot.robots.viscous_follower import ViscousFollower, ViscousFollowerConfig

p = argparse.ArgumentParser()
p.add_argument("--port", required=True)
p.add_argument("--id", default="viscous_01")
p.add_argument("--actuate", action="store_true")
p.add_argument("--travel", type=float, default=20.0)
p.add_argument("--kp", type=float, default=20.0)
p.add_argument("--kd", type=float, default=0.5)
a = p.parse_args()

f = ViscousFollower(ViscousFollowerConfig(port=a.port, id=a.id))
assert f.is_calibrated, "calibration file missing -- refusing to continue"
bus = f.bus
bus.connect()  # handshake only; no torque enable
motors = list(f.config.joint_limits)


def snap(m):
    return (
        float(bus.read("Present_Position", m)),
        float(bus.read("Present_Torque", m)),
        float(bus.read("Temperature_MOS", m)),
    )


try:
    print("== torque OFF snapshot ==")
    for m in motors:
        pos, tq, tmp = snap(m)
        lo, hi = f.config.joint_limits[m]
        print(
            f"{m:<14} pos={pos:8.1f} deg  torque={tq:6.2f} Nm  temp={tmp:5.1f} C  limits=({lo:.1f},{hi:.1f})"
        )
    g0, _, _ = snap("gripper")
    lo, hi = f.config.joint_limits["gripper"]
    off = 0.0
    if g0 > hi + 180:
        off = -360.0
    if g0 < lo - 180:
        off = 360.0
    print(
        f"gripper raw={g0:.1f}; session turn offset={off:+.0f}; in-limits pos={g0 + off:.1f} (limits {lo},{hi})"
    )
    if not a.actuate:
        print("(read-only run; pass --actuate for the gripper step test)")
        raise SystemExit
    print(f"\n== GRIPPER ONLY actuation: kp={a.kp} kd={a.kd}, {a.travel:.0f} deg toward open and back ==")
    bus.write("Kp", "gripper", a.kp)
    bus.write("Kd", "gripper", a.kd)
    bus.enable_torque("gripper")
    start = g0

    def step_to(goal_raw):
        bus.write("Goal_Position", "gripper", goal_raw)
        time.sleep(0.05)
        pos, tq, tmp = snap("gripper")
        print(
            f"  goal={goal_raw + off:7.1f}  pos={pos + off:7.1f}  err={goal_raw - pos:6.1f}  torque={tq:6.2f} Nm  temp={tmp:4.1f}",
            flush=True,
        )

    path = [start - d for d in range(0, int(a.travel) + 1, 2)]
    for g in path + path[::-1]:
        step_to(g)
    for _ in range(10):
        step_to(start)
finally:
    try:
        bus.disable_torque("gripper")
    finally:
        bus.disconnect(disable_torque=False)  # leave the other (already torque-free) motors alone
    print("gripper torque off; bus closed")
