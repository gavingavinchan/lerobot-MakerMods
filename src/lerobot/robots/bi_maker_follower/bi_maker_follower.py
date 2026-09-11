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

import logging
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from functools import cached_property
from typing import Any

from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.utils.bimanual import BimanualMixin
from lerobot.utils.decorators import check_if_not_connected

from ..maker_follower import MakerFollower, MakerFollowerConfig, MakerFollowerConfigBase
from ..robot import Robot
from .config_bi_maker_follower import BiMakerFollowerConfig

logger = logging.getLogger(__name__)


class BiMakerFollower(BimanualMixin, Robot):
    """Bimanual Maker follower with side-prefixed motor and camera keys."""

    config_class = BiMakerFollowerConfig
    name = "bi_maker_follower"

    def __init__(self, config: BiMakerFollowerConfig):
        super().__init__(config)
        self.config = config

        self._top_level_cam_keys = set(config.cameras)
        collisions = self._top_level_cam_keys & (
            set(config.left_arm_config.cameras) | set(config.right_arm_config.cameras)
        )
        if collisions:
            raise ValueError(
                f"Top-level camera names collide with per-arm camera names: {sorted(collisions)}"
            )
        left_arm_cameras = {**config.left_arm_config.cameras, **config.cameras}

        def _arm_config(arm_config: MakerFollowerConfigBase, side: str) -> MakerFollowerConfig:
            return MakerFollowerConfig(
                id=f"{config.id}_{side}" if config.id else None,
                calibration_dir=config.calibration_dir,
                port=arm_config.port,
                can_interface=arm_config.can_interface,
                can_bitrate=arm_config.can_bitrate,
                motor_can_ids=arm_config.motor_can_ids,
                joint_limits=arm_config.joint_limits,
                gains=arm_config.gains,
                startup_sync_speed_deg=arm_config.startup_sync_speed_deg,
                startup_sync_tolerance_deg=arm_config.startup_sync_tolerance_deg,
                max_relative_target=arm_config.max_relative_target,
                temp_warn_c=arm_config.temp_warn_c,
                temp_critical_c=arm_config.temp_critical_c,
                max_silent_cycles=arm_config.max_silent_cycles,
                disable_torque_on_disconnect=arm_config.disable_torque_on_disconnect,
                cameras=left_arm_cameras if side == "left" else arm_config.cameras,
            )

        self.left_arm = MakerFollower(_arm_config(config.left_arm_config, "left"))
        self.right_arm = MakerFollower(_arm_config(config.right_arm_config, "right"))
        self.cameras = {**self.left_arm.cameras, **self.right_arm.cameras}

        # Independent buses release the GIL during I/O, so their waits can overlap.
        self._io_pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="bi_maker_io")

    def _run_both(self, left_fn: Callable[[], Any], right_fn: Callable[[], Any]) -> tuple[Any, Any]:
        left_future = self._io_pool.submit(left_fn)
        right_future = self._io_pool.submit(right_fn)
        left_result = left_future.result()
        right_result = right_future.result()
        return left_result, right_result

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            **{f"left_{key}": value for key, value in self.left_arm._motors_ft.items()},
            **{f"right_{key}": value for key, value in self.right_arm._motors_ft.items()},
        }

    @property
    def _telemetry_ft(self) -> dict[str, type]:
        """Per-arm `{motor}.temp` / `{motor}.torque`, see `MakerFollower._telemetry_ft`."""
        return {
            **{f"left_{key}": value for key, value in self.left_arm._telemetry_ft.items()},
            **{f"right_{key}": value for key, value in self.right_arm._telemetry_ft.items()},
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        features: dict[str, tuple] = {}
        for key, value in self.left_arm._cameras_ft.items():
            features[key if key in self._top_level_cam_keys else f"left_{key}"] = value
        for key, value in self.right_arm._cameras_ft.items():
            features[f"right_{key}"] = value
        return features

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._telemetry_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @check_if_not_connected
    def disconnect(self) -> None:
        self._io_pool.shutdown(wait=True)
        super().disconnect()

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        left_observation, right_observation = self._run_both(
            self.left_arm.get_observation, self.right_arm.get_observation
        )
        observation: RobotObservation = {}
        for key, value in left_observation.items():
            observation[key if key in self._top_level_cam_keys else f"left_{key}"] = value
        for key, value in right_observation.items():
            observation[f"right_{key}"] = value
        return observation

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        left_action = {
            key.removeprefix("left_"): value for key, value in action.items() if key.startswith("left_")
        }
        right_action = {
            key.removeprefix("right_"): value for key, value in action.items() if key.startswith("right_")
        }
        left_sent, right_sent = self._run_both(
            lambda: self.left_arm.send_action(left_action),
            lambda: self.right_arm.send_action(right_action),
        )
        return {
            **{f"left_{key}": value for key, value in left_sent.items()},
            **{f"right_{key}": value for key, value in right_sent.items()},
        }
