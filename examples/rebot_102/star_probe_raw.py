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

"""Read-only probe: print the Star leader's raw servo angles and the mapped Viscous gripper target.

Never commands a position or touches calibration. Ctrl-C to stop.
"""

import sys
import time

from motorbridge_smart_servo import FashionStarServo

from lerobot.teleoperators.rebot_102_leader.config_rebot_102_leader_viscous import (
    RebotArm102LeaderViscousConfig,
)
from lerobot.teleoperators.rebot_102_leader.rebot_102_leader import RebotArm102Leader

port = sys.argv[1]
cfg = RebotArm102LeaderViscousConfig(port=port)
bus = FashionStarServo(port, baudrate=cfg.baudrate)
ids = list(cfg.joint_ids.values())
names = {v: k for k, v in cfg.joint_ids.items()}
for i in ids:
    print(f"id {i} {names[i]:<14} ping={bus.ping(i)}")
lo, hi = cfg.joint_ranges["gripper"]
d = cfg.joint_directions["gripper"]
s = 1.0 if d >= 0 else -1.0
print(
    f"gripper mapping: raw in [{lo * s / d:.1f},{hi * s / d:.1f}] -> target [{lo},{hi}] (factor {d})  -- move the gripper now"
)
mn = dict.fromkeys(names.values(), 1000000000.0)
mx = dict.fromkeys(names.values(), -1000000000.0)
try:
    while True:
        res = bus.sync_monitor(ids)
        raw = {names[i]: (res[i].angle_deg if res.get(i) else float("nan")) for i in ids}
        for n, v in raw.items():
            mn[n] = min(mn[n], v)
            mx[n] = max(mx[n], v)
        g = raw["gripper"]
        unwrapped, _ = RebotArm102Leader._round_to_valid_range(g, lo * s, hi * s)
        target = max(lo, min(hi, unwrapped * d))
        print(
            " ".join(f"{n}={v:7.1f}" for n, v in raw.items())
            + f" | gripper unwrapped={unwrapped:7.1f} target={target:7.1f}",
            flush=True,
        )
        time.sleep(0.2)
except KeyboardInterrupt:
    pass
finally:
    print("\nobserved raw min/max:")
    for n in names.values():
        print(f"  {n:<14} {mn[n]:8.1f} .. {mx[n]:8.1f}  (travel {mx[n] - mn[n]:.1f})")
    bus.close()
