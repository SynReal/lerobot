import logging
from functools import cached_property

from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..piper_follower import PiperFollowerRobot, PiperFollowerConfig
from ..robot import Robot
from .config_bi_piper import BiPiperFollowerConfig

logger = logging.getLogger(__name__)


class BiPiperFollower(Robot):
    config_class = BiPiperFollowerConfig
    name = "bi_piper_follower"

    def __init__(self, config: BiPiperFollowerConfig):
        super().__init__(config)
        self.config = config

        left_arm_config = PiperFollowerConfig(
            port=self.config.left_arm_config.port,
            cameras=self.config.left_arm_config.cameras,
            side="left",
            home_position=self.config.left_arm_config.home_position,
            joint_limits=self.config.left_arm_config.joint_limits,
            max_relative_target=self.config.left_arm_config.max_relative_target,
            ctrl_mode=self.config.left_arm_config.ctrl_mode,
            move_mode=self.config.left_arm_config.move_mode,
            move_spd_rate_ctrl=self.config.left_arm_config.move_spd_rate_ctrl,
            is_mit_mode=self.config.left_arm_config.is_mit_mode,
        )

        right_arm_config = PiperFollowerConfig(
            port=self.config.right_arm_config.port,
            cameras=self.config.right_arm_config.cameras,
            side="right",
            home_position=self.config.right_arm_config.home_position,
            joint_limits=self.config.right_arm_config.joint_limits,
            max_relative_target=self.config.right_arm_config.max_relative_target,
            ctrl_mode=self.config.right_arm_config.ctrl_mode,
            move_mode=self.config.right_arm_config.move_mode,
            move_spd_rate_ctrl=self.config.right_arm_config.move_spd_rate_ctrl,
            is_mit_mode=self.config.right_arm_config.is_mit_mode,
        )

        self.left_arm = PiperFollowerRobot(left_arm_config)
        self.right_arm = PiperFollowerRobot(right_arm_config)

        self.cameras = {**config.cameras, **self.left_arm.cameras, **self.right_arm.cameras}

    @property
    def _robot_ft(self) -> dict[str, type]:
        left_arm_ft = self.left_arm._robot_ft
        right_arm_ft = self.right_arm._robot_ft

        return {
            **{f"left_{k}": v for k, v in left_arm_ft.items()},
            **{f"right_{k}": v for k, v in right_arm_ft.items()},
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        left_arm_cameras_ft = self.left_arm._cameras_ft
        right_arm_cameras_ft = self.right_arm._cameras_ft

        return {
            **{f"left_{k}": v for k, v in left_arm_cameras_ft.items()},
            **{f"right_{k}": v for k, v in right_arm_cameras_ft.items()},
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._robot_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._robot_ft

    @property
    def is_connected(self):
        return self.left_arm.is_connected and self.right_arm.is_connected

    @check_if_already_connected
    def connect(self):
        self.left_arm.connect()
        self.right_arm.connect()

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        left_obs = self.left_arm.get_observation()
        right_obs = self.right_arm.get_observation()

        # Prefix left/right to the keys to disambiguate them
        prefixed_left_obs = {f"left_{k}": v for k, v in left_obs.items()}
        prefixed_right_obs = {f"right_{k}": v for k, v in right_obs.items()}

        return {**prefixed_left_obs, **prefixed_right_obs}

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        left_action = {
            key.removeprefix("left_"): value for key, value in action.items() if key.startswith("left_")
        }
        # Remove "right_" prefix
        right_action = {
            key.removeprefix("right_"): value for key, value in action.items() if key.startswith("right_")
        }
        sent_action_left = self.left_arm.send_action(left_action)
        sent_action_right = self.right_arm.send_action(right_action)
        return {**{f"left_{k}": v for k, v in sent_action_left.items()}, **{f"right_{k}": v for k, v in sent_action_right.items()}}

    @check_if_not_connected
    def disconnect(self):
        self.left_arm.disconnect()
        self.right_arm.disconnect()
