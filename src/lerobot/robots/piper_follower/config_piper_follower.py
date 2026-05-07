from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig

DEFAULT_JOINT_LIMITS = {
    "joint_1": (-2.687, 2.687),
    "joint_2": (0.0, 3.403),
    "joint_3": (-3.0541012, 0.0),
    "joint_4": (-1.5499, 1.5499),
    "joint_5": (-1.22, 1.22),
    "joint_6": (-1.7452, 1.7452),
    "gripper": (0.0, 1.0),
}

DEFAULT_LEFT_HOME_POSITION = {
    "joint_1": -0.1240,
    "joint_2": 0.7980,
    "joint_3": -1.1250,
    "joint_4": -0.2253,
    "joint_5": 0.9856,
    "joint_6": 0.0829,
    "gripper": 1.0,
}

DEFAULT_RIGHT_HOME_POSITION = {
    "joint_1": 0.1240,
    "joint_2": 0.7980,
    "joint_3": -1.1250,
    "joint_4": 0.2253,
    "joint_5": 0.9856,
    "joint_6":  -0.0829,
    "gripper": 1.0,
}

ARM_FACTOR = 57295.779513  # 1000 * 180 / pi, converts rad ↔ SDK 0.001° units
GRIPPER_UNIT_FACTOR = 1000.0 * 1000.0  # 0.001mm → meters
GRIPPER_MAX = 0.1  # DualPiper driver 的归一化基准 (0.1m = 100mm)
GRIPPER_FACTOR = GRIPPER_UNIT_FACTOR * GRIPPER_MAX  # = 100000
GRIPPER_SDK_MAX = 70.0 * 1000  # 真实 gripper 最大开合: 70mm = 70000 SDK units
GRIPPER_RESCALE = GRIPPER_FACTOR / GRIPPER_SDK_MAX  # ≈ 1.4286, 把 driver 的 [0,0.7] → [0,1]

@dataclass
class PiperFollowerConfig(RobotConfig):
    # CAN interface to connect to the arm (e.g., "can1")
    # Linux: "can0", "can1", etc.
    port: str

    side: str | None = None
    home_position: dict[str, float] | None = None
    joint_limits: dict[str, tuple[float, float]] | None = None

    max_relative_target: float | dict[str, float] | None = None

    """
    ctrl_mode: 控制模式 uint8
        0x00 待机模式
        0x01 CAN 指令控制模式
    move_mode: MOVE模式 uint8
        0x00 MOVE P
        0x01 MOVE J
        0x02 MOVE L
        0x03 MOVE C
        0x04 MOVE M ---基于V1.5-2版本后
        0x05 MOVE CPV ---基于V1.8-1版本后
    move_spd_rate_ctrl 运动速度百分比 uint8
        数值范围0~100
    is_mit_mode: mit模式 uint8
        0x00 位置速度模式
        0xAD MIT模式
        0xFF 无效
    """
    ctrl_mode: int = 0x01
    move_mode: int = 0x01
    move_spd_rate_ctrl: int = 100
    is_mit_mode: int = 0x00

    # Camera configurations
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self):
        if self.side is not None and self.home_position is None:
            if self.side.lower() == "left":
                self.home_position = DEFAULT_LEFT_HOME_POSITION
            elif self.side.lower() == "right":
                self.home_position = DEFAULT_RIGHT_HOME_POSITION
            else:
                raise ValueError(f"Invalid side '{self.side}'. Must be 'left', 'right', or None.")

        if self.joint_limits is None:
            self.joint_limits = DEFAULT_JOINT_LIMITS
