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

from dataclasses import dataclass, field

from ..config import RobotConfig
from ..maker_follower import MakerFollowerConfigBase


@dataclass
class ViscousFollowerConfigBase(MakerFollowerConfigBase):
    """
    Configuration for the Viscous Arm v1 follower: same topology, joint names, CAN ids and
    driver stack as the Maker Arm, so everything not listed here is inherited from
    `MakerFollowerConfigBase`. Two things differ, both hardware facts about this arm:

    * all seven joints are RS00 (RobStride model "O0"), where Maker carries RS02 at
      `shoulder_lift` / `elbow_flex` — see `ViscousFollower.motor_models`;
    * its own soft limits and follow gains, below.

    Kept separate from the registered `ViscousFollowerConfig` for the same reason the Maker
    base is: so a bimanual config could embed per-arm configs without making the draccus CLI
    parser tree self-referential.
    """

    # Soft joint limits (degrees), clipped against on every action. Zero is the calibration pose
    # (arm folded in its resting position, gripper open). Captured on this arm on 2026-08-28 with
    # the maker-arm SDK `capture_maker_limits.py` at a 0.05 rad backoff from the mechanical stops,
    # then re-expressed against that zero (`joint-limits-zeroed-frame.json`). `wrist_yaw` was
    # recaptured on its own with `--motor-id 5` after the first sweep only reached 36 deg of its
    # 176 deg travel; the rewritten value agrees with the Maker arm to 0.1%. Narrow them to fence
    # off workspace.
    joint_limits: dict[str, tuple[float, float]] = field(
        default_factory=lambda: {
            "shoulder_pan": (-185.4, 127.6),
            "shoulder_lift": (-176.4, -2.4),
            "elbow_flex": (2.9, 251.1),
            "wrist_flex": (-120.8, 92.0),
            "wrist_yaw": (-89.2, 86.2),
            "wrist_roll": (-151.3, 153.6),
            "gripper": (-120.0, -2.8),
        }
    )

    # Per-motor MIT follow gains {name: (kp, kd)}, applied at connect().
    #
    # UNTUNED for `shoulder_lift` and `elbow_flex`: these are the deliberately soft placeholders
    # from the SDK profile (`viscous_arm_v1.yaml`), NOT bench-tuned values. Maker's kp 150 / 90
    # there are RS02 numbers and must not be copied onto this arm's RS00 motors, which hold a
    # comparable mass against gravity on a dry bench with none of the buoyancy the arm is
    # designed around. Placeholdered soft so the failure mode is sag rather than a slam: start
    # here, walk kp up +10 at a time against soft tracking, kd +0.3 against jitter, and keep
    # early bench sessions short (an RS00 holding a pose it cannot comfortably hold runs hot).
    # The other five joints match Maker's bench-tuned values; the gripper runs a deliberately low
    # kp so grip force (kp times position error) stays compliant.
    gains: dict[str, tuple[float, float]] = field(
        default_factory=lambda: {
            "shoulder_pan": (60.0, 4.0),
            "shoulder_lift": (30.0, 2.0),  # untuned placeholder
            "elbow_flex": (30.0, 2.0),  # untuned placeholder
            "wrist_flex": (30.0, 2.0),
            "wrist_yaw": (30.0, 2.0),
            "wrist_roll": (30.0, 2.0),
            "gripper": (20.0, 0.5),
        }
    )


@RobotConfig.register_subclass("viscous_follower")
@dataclass
class ViscousFollowerConfig(RobotConfig, ViscousFollowerConfigBase):
    """Registered single-arm Viscous follower config (adds `id` / `calibration_dir`)."""
