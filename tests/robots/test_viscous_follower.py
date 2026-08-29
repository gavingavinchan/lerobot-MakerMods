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

from unittest.mock import MagicMock, patch

import pytest

from lerobot.robots.config import RobotConfig
from lerobot.robots.maker_follower import MakerFollowerConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.robots.viscous_follower import ViscousFollower, ViscousFollowerConfig

JOINTS = list(ViscousFollowerConfig(port="x").motor_can_ids)

# The follower is a subclass of MakerFollower, so the shared behaviour (soft-limit clipping,
# startup sync, full-turn detection, stale zero, missed replies) is covered once in
# test_maker_follower.py. What is tested here is only what the Viscous Arm changes.


@pytest.fixture
def follower(tmp_path):
    r = ViscousFollower(
        ViscousFollowerConfig(
            port="/dev/fake",
            calibration_dir=tmp_path,
            startup_sync_speed_deg=None,
        )
    )
    r.bus = MagicMock()
    r.bus.read.side_effect = lambda _data_name, motor: 0.0
    r.bus.is_connected = True
    return r


def test_every_joint_is_an_rs00():
    """The whole point of the type: no RS02 anywhere, unlike the Maker arm."""
    with patch("lerobot.robots.maker_follower.maker_follower.RobstrideMotorsBus") as bus_cls:
        ViscousFollower(ViscousFollowerConfig(port="/dev/fake"))
    motors = bus_cls.call_args.kwargs["motors"]
    assert list(motors) == JOINTS
    for motor in motors.values():
        assert motor.model == "O0"
        assert motor.motor_type_str == "O0"
        assert motor.recv_id == motor.id
    assert set(ViscousFollower.motor_models.values()) == {"O0"}


def test_defaults():
    cfg = ViscousFollowerConfig(port="/dev/fake")
    assert cfg.can_interface == "slcan"
    assert set(cfg.joint_limits) == set(cfg.gains) == set(JOINTS)
    for lo, hi in cfg.joint_limits.values():
        assert lo < hi
    for kp, kd in cfg.gains.values():
        assert 0 < kp <= 500 and 0 < kd <= 5
    # These are this arm's own captures / gains, not the Maker arm's.
    maker = MakerFollowerConfig(port="/dev/fake")
    assert cfg.joint_limits != maker.joint_limits
    # The RS02 gains must never be copied onto RS00 motors, see the Viscous docs.
    for joint in ("shoulder_lift", "elbow_flex"):
        assert cfg.gains[joint][0] < maker.gains[joint][0]
    # Everything not specific to this arm is inherited from the Maker base.
    assert cfg.motor_can_ids == maker.motor_can_ids
    assert cfg.can_bitrate == maker.can_bitrate
    assert cfg.disable_torque_on_disconnect == maker.disable_torque_on_disconnect


def test_send_action_clips_to_the_viscous_soft_limits(follower):
    lo, hi = follower.config.joint_limits["elbow_flex"]
    out = follower.send_action({"elbow_flex.pos": hi + 50.0, "shoulder_pan.pos": -1000.0})
    assert out["elbow_flex.pos"] == hi
    assert out["shoulder_pan.pos"] == follower.config.joint_limits["shoulder_pan"][0]


def test_connect_pushes_the_viscous_gains(follower):
    follower.calibration = {"fake": object()}
    follower.bus.is_connected = False

    def _connect():
        follower.bus.is_connected = True

    follower.bus.connect.side_effect = _connect
    follower.connect(calibrate=False)
    follower.bus.sync_write.assert_any_call("Kp", {j: kp for j, (kp, _) in follower.config.gains.items()})


def test_registered_type_resolves():
    assert RobotConfig.get_choice_class("viscous_follower") is ViscousFollowerConfig
    with patch("lerobot.robots.maker_follower.maker_follower.RobstrideMotorsBus"):
        robot = make_robot_from_config(ViscousFollowerConfig(port="/dev/fake"))
    assert isinstance(robot, ViscousFollower)
    assert robot.name == "viscous_follower"
