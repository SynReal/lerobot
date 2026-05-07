from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig
from ..piper_follower import PiperFollowerConfig


@RobotConfig.register_subclass("bi_piper_follower")
@dataclass(kw_only=True)
class BiPiperFollowerConfig(RobotConfig):
    """Configuration class for Bi Piper follower robots."""

    id: str | None = "bi_piper_follower"

    left_arm_config: PiperFollowerConfig
    right_arm_config: PiperFollowerConfig

    # Top-level cameras shared across both arms.
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
