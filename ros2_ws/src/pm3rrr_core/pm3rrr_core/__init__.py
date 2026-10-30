"""pm3rrr_core: ROS-independent maths for the planar 3-RRR parallel manipulator."""

from .params import DEFAULT_PARAMS, RobotParams, ServoSpec  # noqa: F401
from .kinematics import IKResult, Pm3rrrKinematics, rot2, wrap  # noqa: F401
from .dynamics import Pm3rrrDynamics  # noqa: F401

__version__ = "1.0.0"
