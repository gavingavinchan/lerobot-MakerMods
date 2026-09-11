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

from ..config import TeleoperatorConfig


@dataclass
class RebotArm102LeaderButtonConfig:
    """Lock-button (pose-hold) settings for the Star Arm 102-HD leader.

    The HD leader ships with a button board that the vendor exposes as an extra device on the
    servo bus (default id 7, "slave mode"): polled like a servo it answers ``unlocked_angle``
    while the arm is free and ``locked_angle`` once the button has been pressed. Crossing
    ``trigger_threshold`` puts every non-excluded joint into servo stop-on-control mode "hold"
    (0x11), so the leader holds the pose it is standing in; crossing back releases it again.

    Off by default: the read-only 102-LD (green) arm has no button board, and nothing here is
    reached while ``enabled`` is False -- no extra bus traffic, no stop-mode writes.
    """

    # Master switch. False leaves get_action byte-for-byte as it was before this feature.
    enabled: bool = False

    # Bus id of the button board. Must not collide with `joint_ids`.
    id: int = 7

    # Angle in degrees at or above which the button reads as "locked". The board reports three
    # states (vendor `stararm102_ro.py`, `Button.key_state_*`): 0 = unlocked, 180 = lock
    # requested, 90 = unlock requested. 135 sits between the two non-zero states so 90 is
    # never mistaken for a lock (the vendor's LeRobot plugin uses 90 and would be).
    trigger_threshold: float = 135.0

    # While locked, keep emitting the last action recorded before the lock, so a follower
    # freezes with the leader instead of chasing the small drift of a held-but-nudged arm.
    # Excluded joints are still reported live: they were never locked, so they still move.
    freeze_action_on_lock: bool = True

    # Lock immediately after connect(), before the first get_action.
    lock_on_connect: bool = False

    # Joints never locked -- they stay free while the rest of the arm holds. Empty by default
    # (the whole arm locks); the vendor excludes "gripper" so the gripper keeps working as a
    # trigger while the pose is frozen.
    excluded_lock_joints: list[str] = field(default_factory=list)

    # Power cap in mW for the hold (0x11) command. 0 means "servo firmware default", which is
    # what the vendor sends; the RP8-U45H-M then holds against its own power-protection
    # threshold (config id 42) rather than a per-command cap.
    hold_power_mw: int = 0

    # What "unlocked" means: "release" is stop mode 0x10 (torque off, the arm is free and
    # backdrivable, the pre-existing behaviour) and "damping" is stop mode 0x12, which leaves
    # the joints free but resisting, at `damping_power_mw`.
    release_mode: str = "release"

    # Power cap in mW for the damping (0x12) command; only read when release_mode="damping".
    # 900 mW is the vendor's value.
    damping_power_mw: int = 900

    # Stop polling the button after this many consecutive failed reads, keeping the current
    # lock state (an unanswered id costs a full bus timeout every cycle, which alone would
    # break a 60 Hz teleop loop). 0 polls forever.
    max_consecutive_read_errors: int = 20


@dataclass
class RebotArm102LeaderConfig:
    """Base configuration class for the Seeed Studio StarArm102 / reBot Arm 102 leader.

    The reBot Arm 102 is a 7-joint (incl. gripper) leader arm driven by FashionStar
    UART smart servos. Servo communication goes through ``motorbridge-smart-servo``.
    """

    # USB-to-UART device the leader arm is connected to (e.g. "/dev/ttyUSB0").
    port: str

    baudrate: int = 1_000_000

    # Servo id of each joint on the UART bus.
    joint_ids: dict[str, int] = field(
        default_factory=lambda: {
            "shoulder_pan": 0,
            "shoulder_lift": 1,
            "elbow_flex": 2,
            "wrist_flex": 3,
            "wrist_yaw": 4,
            "wrist_roll": 5,
            "gripper": 6,
        }
    )

    # Per-joint signed scale applied to raw servo angles so the leader matches the follower
    # convention: output = raw * direction. The sign flips direction; the magnitude rescales
    # a leader joint's travel onto a follower joint with a different range (e.g. the gripper
    # uses -6 to widen onto the reBot B601 gripper; a follower joint with a smaller range than
    # the leader uses a fractional magnitude like -0.667). Float so non-integer scales work.
    joint_directions: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": -1.0,
            "shoulder_lift": -1.0,
            "elbow_flex": 1.0,
            "wrist_flex": 1.0,
            "wrist_yaw": 1.0,
            "wrist_roll": -1.0,
            "gripper": -6.0,
        }
    )

    # Per-joint [min, max] output range in degrees. Matches the reBot B601 follower
    # joint limits so leader actions can drive the follower key-for-key.
    joint_ranges: dict[str, list[int]] = field(
        default_factory=lambda: {
            "shoulder_pan": [-150, 150],
            "shoulder_lift": [-200, 1],
            "elbow_flex": [-200, 1],
            "wrist_flex": [-80, 90],
            "wrist_yaw": [-90, 90],
            "wrist_roll": [-90, 90],
            "gripper": [-270, 0],
        }
    )

    # Per-joint additive offset in degrees, applied after `joint_directions` and before the
    # `joint_ranges` clamp: output = raw * direction + offset. Joints not listed get 0. For when
    # a follower joint's zero does not coincide with the leader's, e.g. a gripper whose closed
    # stop sits well inside the follower's range (see the Viscous preset).
    joint_offsets: dict[str, float] = field(default_factory=dict)

    # Optional lock-button support (102-HD only). Nested so every preset inherits it and the
    # CLI reaches it as `--teleop.button.enabled=true`, `--teleop.button.id=7`, ...
    button: RebotArm102LeaderButtonConfig = field(default_factory=RebotArm102LeaderButtonConfig)


@TeleoperatorConfig.register_subclass("rebot_102_leader")
@dataclass
class RebotArm102LeaderTeleopConfig(TeleoperatorConfig, RebotArm102LeaderConfig):
    """Registered configuration for the reBot Arm 102 leader teleoperator."""

    pass
