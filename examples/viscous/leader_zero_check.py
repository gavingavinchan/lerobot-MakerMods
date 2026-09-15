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

"""Pre-teleop check: is the Star leader sitting in its zero pose?

Read-only. Never commands a position, never unlocks, never touches calibration or the follower.
On connect the follower drives to the leader's pose, so a leader left away from zero becomes the
follower's first motion. The leader's connect now enforces that itself through
`max_start_offset_deg` (30 deg on the Viscous preset); this script remains for a tighter,
standalone per-joint read-out.

Every joint's zero is raw 0 deg. `shoulder_pan` lost its origin when that joint of the leader was
reassembled; on 2026-09-14 its servo origin was re-set with `examples/rebot_102/star_zero_one_joint.py`
at the pose matched by eye to the follower's reference pose, so it reads 0 again. (A zero derived
from the end stops was tried first and landed ~27 deg off: the stops do not locate zero.) If the
pan is reassembled and not re-zeroed, pass its zero as a raw angle with --pan-zero.

Exits 0 when every joint is within --tol of zero, 1 otherwise.
"""

import argparse
import sys
import time

from motorbridge_smart_servo import FashionStarServo

from lerobot.teleoperators.rebot_102_leader.config_rebot_102_leader_viscous import (
    RebotArm102LeaderViscousConfig,
)

p = argparse.ArgumentParser()
p.add_argument("--port", required=True)
p.add_argument("--tol", type=float, default=10.0, help="allowed deviation from zero, leader raw deg")
p.add_argument("--pan-zero", type=float, default=0.0, help="shoulder_pan zero, leader raw deg")
p.add_argument("--samples", type=int, default=5)
a = p.parse_args()

cfg = RebotArm102LeaderViscousConfig(port=a.port)
zero = dict.fromkeys(cfg.joint_ids, 0.0)
zero["shoulder_pan"] = a.pan_zero
names = {v: k for k, v in cfg.joint_ids.items()}
ids = list(names)

bus = FashionStarServo(a.port, baudrate=cfg.baudrate)
try:
    missing = [names[i] for i in ids if not bus.ping(i)]
    if missing:
        print(f"NOT RESPONDING: {', '.join(missing)}")
        sys.exit(1)
    sums = dict.fromkeys(ids, 0.0)
    counts = dict.fromkeys(ids, 0)
    for _ in range(a.samples):
        for i, mon in bus.sync_monitor(ids).items():
            if mon is not None:
                sums[i] += mon.angle_deg
                counts[i] += 1
        time.sleep(0.05)
finally:
    bus.close()

ok = True
print(f"{'joint':<14} {'raw':>8} {'zero':>8} {'off by':>8}")
for i in ids:
    n = names[i]
    if counts[i] == 0:
        print(f"{n:<14} {'--':>8} {zero[n]:8.1f} {'--':>8}  NO READING")
        ok = False
        continue
    raw = sums[i] / counts[i]
    off = raw - zero[n]
    good = abs(off) <= a.tol
    ok &= good
    print(f"{n:<14} {raw:8.1f} {zero[n]:8.1f} {off:+8.1f}  {'ok' if good else 'MOVE TOWARD ZERO'}")
print(f"LEADER {'AT ZERO' if ok else 'NOT AT ZERO'} (tolerance {a.tol:.0f} deg)")
sys.exit(0 if ok else 1)
