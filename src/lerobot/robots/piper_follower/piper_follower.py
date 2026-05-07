import logging
import time
from functools import cached_property
from typing import Any
import numpy as np

from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.damiao import DamiaoMotorsBus
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_piper_follower import (
    DEFAULT_JOINT_LIMITS,
    ARM_FACTOR,
    GRIPPER_UNIT_FACTOR,
    GRIPPER_MAX,
    GRIPPER_FACTOR,
    GRIPPER_SDK_MAX,
    GRIPPER_RESCALE,
    PiperFollowerConfig,
)

from piper_sdk import C_PiperInterface_V2

logger = logging.getLogger(__name__)

class PiperFollowerRobot(Robot):
    config_class = PiperFollowerConfig
    name = "piper_follower"

    def __init__(self, config: PiperFollowerConfig):
        super().__init__(config)
        self.config = config

        self.robot = C_PiperInterface_V2(self.config.port)

        self.cameras = make_cameras_from_configs(self.config.cameras)

    @property
    def _robot_ft(self) -> dict[str, type]:
        return {
            "joint_1": float,
            "joint_2": float,
            "joint_3": float,
            "joint_4": float,
            "joint_5": float,
            "joint_6": float,
            "gripper": float,
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        """Camera features for observation space."""
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        """Combined observation features from motors and cameras."""
        return {**self._robot_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        """Action features."""
        return self._robot_ft

    @property
    def is_connected(self):
        return self.robot.get_connect_status()

    @check_if_already_connected
    def connect(self):
        self.robot.ConnectPort()
        self.enable_robot()
        self.robot.MotionCtrl_2(
            self.config.ctrl_mode, self.config.move_mode, self.config.move_spd_rate_ctrl, self.config.is_mit_mode
            )

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()
        obs_dict: dict[str, Any] = {}

        arm_ft = self.robot.GetArmJointMsgs().joint_state
        gripper_ft = self.robot.GetArmGripperMsgs().gripper_state.grippers_angle
        gripper_ft = np.clip(gripper_ft / GRIPPER_MAX, 0.0, 1.0)

        obs_dict.update({
            "joint_1": arm_ft.joint_1 / ARM_FACTOR,
            "joint_2": arm_ft.joint_2 / ARM_FACTOR,
            "joint_3": arm_ft.joint_3 / ARM_FACTOR,
            "joint_4": arm_ft.joint_4 / ARM_FACTOR,
            "joint_5": arm_ft.joint_5 / ARM_FACTOR,
            "joint_6": arm_ft.joint_6 / ARM_FACTOR,
            "gripper": gripper_ft,
        })

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.read_latest()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} get_observation took: {dt_ms:.1f}ms")

        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        arm_action = {f"joint_{i+1}": action[f"joint_{i+1}"] for i in range(6)}

        for key, value in arm_action.items():
            arm_action[key] = np.clip(value, self.config.joint_limits[key][0], self.config.joint_limits[key][1])
            arm_action[key] = (arm_action[key] * ARM_FACTOR).astype(int).tolist()

        gripper_action = abs(action["gripper"])
        gripper_action = float(np.clip(gripper_action, 0.0, 1.0))
        gripper_action = round(gripper_action * 70 * 1000)

        self.robot.JointCtrl(**arm_action)
        self.robot.GripperCtrl(gripper_action, 1000, 0x01, 0)

        arm_action["gripper"] = gripper_action

        return arm_action

    def enable_robot(self):
        timeout = 8.0 # seconds
        start_time = time.perf_counter()

        logger.debug("Waiting for robot to enable...")
        while not self.robot.get_connect_status():
            if time.perf_counter() - start_time > timeout:
                raise TimeoutError(f"Failed to enable robot within {timeout} seconds.")

            time.sleep(0.1)
            self.robot.EnableArm(7)
            enable_ = sum(self.robot.GetArmEnableStatus()) == 6

            if enable_:
                self.robot.GripperCtrl(70 * 1000, 1000, 0x01, 0)
                logger.debug("Robot enabled successfully.")
                return

    @check_if_not_connected
    def disconnect(self):
        self.robot.DisconnectPort()
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
