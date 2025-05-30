"""
Delta Robot Hybrid Python/C++ Package
"""

from .generator import PositionGenerator
from .visualizer import DeltaVisualizer

# Try to import C++ module
try:
    from .delta_robot_cpp import DeltaRobotMath, CalculationResult
    CPP_AVAILABLE = True
except ImportError:
    CPP_AVAILABLE = False

# Try to import math visualizer
try:
    from .math_visualizer import DeltaMathVisualizer
    MATH_VIZ_AVAILABLE = True
except ImportError:
    MATH_VIZ_AVAILABLE = False