from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig
from ..piper import PiperConfig


@RobotConfig.register_subclass("bi_piper")
@dataclass(kw_only=True)
class BiPiperConfig(RobotConfig):
    """Configuration class for Bi Piper robots."""

    id: str | None = "bi_piper"

    left_arm_config: PiperConfig
    right_arm_config: PiperConfig

    # Top-level cameras shared across both arms.
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
