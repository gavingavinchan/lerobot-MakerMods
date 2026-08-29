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

import math
from dataclasses import dataclass, field

from lerobot.robots.viscous_follower.config_viscous_follower import ViscousFollowerConfigBase

from ..config import TeleoperatorConfig
from .config_rebot_102_leader import RebotArm102LeaderConfig


def _viscous_joint_ranges() -> dict[str, list[int]]:
    """`ViscousFollowerConfigBase.joint_limits`, rounded inward to the int bounds the leader uses.

    Derived rather than duplicated so the follower's captured limits stay the single source of
    truth; `test_viscous_ranges_match_follower_limits` enforces that they still agree.
    Truncating toward zero-width (ceil the low, floor the high) keeps the leader strictly inside
    the follower's envelope, so it can never emit a target the follower would have to clip.
    """
    limits = ViscousFollowerConfigBase().joint_limits
    return {joint: [math.ceil(lo), math.floor(hi)] for joint, (lo, hi) in limits.items()}


@dataclass
class RebotArm102LeaderViscousConfig(RebotArm102LeaderConfig):
    """The reBot Arm 102 / Star Arm 102 leader mapped onto the Viscous Arm follower.

    Same physical leader as `RebotArm102LeaderConfig`, same servo bus, same `joint_ids`. Only the
    joint mapping differs. The parent's defaults describe the reBot B601 follower; pointed at a
    Viscous arm unchanged, most joints run the wrong way or saturate against the follower's soft
    limits and stop moving while teleop keeps reporting a healthy loop.

    `joint_ranges` is derived from `ViscousFollowerConfigBase.joint_limits`, so the leader can
    never emit a target outside the follower's envelope.
    `test_viscous_ranges_match_follower_limits` enforces that identity. Update the follower's
    limits and these follow.
    """

    # Sign flips travel; magnitude rescales a leader joint onto a follower joint whose travel
    # differs. Captured for this arm: Star leader travel from the maker-arm SDK
    # `capture_star_ranges.py`, 2026-08-29 (purple Star Arm 102, `/dev/cu.wchusbserial10`),
    # against the follower limits captured 2026-08-28 that `joint_ranges` below is derived from.
    # factor = sign * (follower travel / Star travel), per joint; the signs are carried from the
    # Maker mapping, since the two arms share a topology and joint map and neither is mirrored.
    # The mapping is linear, so a single factor per joint is exact. The preset only scales and
    # flips -- it adds no offset, so the leader's zero must coincide with the follower's zero.
    joint_directions: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": 1.439742,
            "shoulder_lift": -0.868697,
            "elbow_flex": -0.915867,
            "wrist_flex": -1.082952,
            "wrist_yaw": -1.260963,
            "wrist_roll": 0.945426,
            "gripper": -1.778452,
        }
    )

    # Mirrors ViscousFollowerConfigBase.joint_limits so leader output is bounded by the follower's
    # own soft limits before it ever reaches the bus. These bounds do more than clip:
    # `_round_to_valid_range` centres the multi-turn unwrap window on (min+max)/2, so a range
    # borrowed from the wrong follower can land a joint on the wrong 360 deg branch.
    joint_ranges: dict[str, list[int]] = field(default_factory=_viscous_joint_ranges)


@TeleoperatorConfig.register_subclass("rebot_102_leader_viscous")
@dataclass
class RebotArm102LeaderViscousTeleopConfig(TeleoperatorConfig, RebotArm102LeaderViscousConfig):
    """Registered configuration for the reBot Arm 102 leader driving a Viscous arm follower."""

    pass
