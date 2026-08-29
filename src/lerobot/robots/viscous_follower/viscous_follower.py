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

from ..maker_follower import MakerFollower
from .config_viscous_follower import ViscousFollowerConfig

# All seven joints are RS00, model "O0" in `motors/robstride/tables.py`. The Viscous Arm runs
# neutrally buoyant underwater, where the gravity load Maker's RS02 shoulder_lift / elbow_flex
# fight on a bench largely disappears, so the whole arm is built from one motor tier.
MOTOR_MODELS = dict.fromkeys(
    (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_yaw",
        "wrist_roll",
        "gripper",
    ),
    "O0",
)


class ViscousFollower(MakerFollower):
    """
    Viscous Arm v1 follower: 6 joints + a permanent gripper, all RS00 RobStride motors over
    classic CAN via the stock `RobstrideMotorsBus` in MIT position control.

    Identical in topology and driver stack to the Maker Arm, so the whole implementation is
    inherited; only the motor fit (all RS00) and the config defaults (this arm's own soft limits
    and gains) differ.
    """

    config_class = ViscousFollowerConfig
    name = "viscous_follower"
    motor_models = MOTOR_MODELS
    display_name = "Viscous"
