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
            telemetry_in_state=True,
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


def test_the_thermal_guard_is_inherited(follower):
    """RS00 on J2/J3 fighting gravity on a dry bench is what cooked this arm on 2026-08-29."""
    cfg = follower.config
    assert (cfg.temp_warn_c, cfg.temp_critical_c, cfg.temp_stop_c) == (60.0, 75.0, 85.0)
    # Well under the RS00's own protection: firmware warns at 135 C and faults at 145 C.
    assert cfg.temp_stop_c < 135.0

    assert "shoulder_lift.temp" in follower.observation_features

    temperatures = {**dict.fromkeys(JOINTS, 30.0), "shoulder_lift": 78.0}
    follower.bus.read.side_effect = lambda data_name, motor: (
        temperatures[motor] if data_name == "Temperature_MOS" else 0.0
    )
    obs = follower.get_observation()
    assert obs["shoulder_lift.temp"] == 78.0
    # Over the critical limit but under the stop the follower shouts and keeps driving: stopping
    # releases torque and drops the arm.
    follower.get_observation()
    follower.get_observation()
    follower.send_action({"shoulder_lift.pos": -90.0})
    follower.bus.disable_torque.assert_not_called()


def test_the_thermal_stop_ends_the_session_at_85_c(follower):
    temperatures = {**dict.fromkeys(JOINTS, 30.0), "elbow_flex": 85.0}
    follower.bus.read.side_effect = lambda data_name, motor: (
        temperatures[motor] if data_name == "Temperature_MOS" else 0.0
    )
    follower.get_observation()
    follower.get_observation()
    with pytest.raises(RuntimeError, match="thermal stop"):
        follower.get_observation()


def test_a_dead_bus_ends_the_session(follower):
    """The 2026-08-29 cutout left `lerobot-teleoperate` spinning for 24 minutes on warnings."""
    follower.config.max_silent_cycles = 4
    follower.get_observation()
    follower.bus.read.side_effect = ConnectionError("No response received from motor")
    for _ in range(3):
        follower.get_observation()
    with pytest.raises(RuntimeError, match="check arm power / CAN adapter"):
        follower.get_observation()


@pytest.mark.parametrize("telemetry_in_state,width", [(False, 7), (True, 21)])
def test_policy_state_width_is_explicit(telemetry_in_state, width, tmp_path):
    from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features

    robot = ViscousFollower(
        ViscousFollowerConfig(
            port="/dev/fake", calibration_dir=tmp_path, telemetry_in_state=telemetry_in_state
        )
    )
    features = hw_to_dataset_features(robot.observation_features, "observation")
    actions = hw_to_dataset_features(robot.action_features, "action")
    values = {
        **{f"{joint}.pos": 1.0 for joint in JOINTS},
        **{f"{joint}.temp": 35.0 for joint in JOINTS},
        **{f"{joint}.torque": 0.1 for joint in JOINTS},
    }
    frame = build_dataset_frame(features, values, "observation")
    assert features["observation.state"]["shape"] == (width,)
    assert frame["observation.state"].shape == (width,)
    assert actions["action"]["shape"] == (7,)
    assert robot.config.temp_stop_c == 85.0
