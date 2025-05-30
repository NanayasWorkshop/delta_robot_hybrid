"""
Position generator for delta robot testing
"""

import numpy as np
from typing import List, Tuple, Optional
import time

# Define a simple Python implementation as fallback
class PyDeltaRobotMath:
    def __init__(self, enable_timing=False):
        self.enable_timing = enable_timing
        print("Using Python fallback implementation (limited functionality)")
    
    def calculate_joint_values(self, target_point):
        # Very simple implementation that just returns a placeholder result
        # This would be replaced with actual calculations in a real implementation
        return {
            'pitch': 0.1,
            'roll': 0.1,
            'motor_A': 5.0,
            'motor_B': 5.0,
            'motor_C': 5.0,
            'prismatic_length': 20.0,
            'fermat_point': [0, 0, 15],
            'within_limits': True,
            'original_target': target_point,
            'corrected_target': target_point,
            'workspace_corrected': False
        }

# Import C++ module (will be available after building)
try:
    from .delta_robot_cpp import DeltaRobotMath, constants
    CPP_AVAILABLE = True
except ImportError:
    print("Warning: C++ module not found, using Python implementation")
    CPP_AVAILABLE = False

class PositionGenerator:
    """Generates random positions for delta robot testing"""
    
    def __init__(self, 
                 robot_radius: float = None,
                 min_height: float = None,
                 working_height: float = None,
                 motor_limit: float = None):
        """Initialize with robot parameters (uses constants if not provided)"""
        
        # Use constants from C++ module if available, otherwise use defaults
        if CPP_AVAILABLE:
            self.robot_radius = robot_radius or constants.ROBOT_RADIUS
            self.min_height = min_height or constants.MIN_HEIGHT
            self.working_height = working_height or constants.WORKING_HEIGHT
            self.motor_limit = motor_limit or constants.MOTOR_LIMIT
            self.workspace_cone_angle_rad = constants.WORKSPACE_CONE_ANGLE_RAD
        else:
            # Fallback defaults (should match the constants)
            self.robot_radius = robot_radius or 24.8
            self.min_height = min_height or 101.0
            self.working_height = working_height or 11.5
            self.motor_limit = motor_limit or 11.0
            self.workspace_cone_angle_rad = 0.5236  # 30 degrees
        
        self.resting_position = 2 * self.motor_limit + self.min_height
        
        # Initialize the math engine
        if CPP_AVAILABLE:
            self.math_engine = DeltaRobotMath(
                self.robot_radius,
                self.min_height,
                self.working_height,
                self.motor_limit,
                self.resting_position,
                self.workspace_cone_angle_rad
            )
            self.using_cpp = True
        else:
            self.math_engine = PyDeltaRobotMath(enable_timing=True)
            self.using_cpp = False
    
    def generate_random_positions(self, num_points: int = 100) -> List[np.ndarray]:
        """Generate random positions within the expected workspace"""
        np.random.seed(42)  # For reproducible results
        
        positions = []
        for _ in range(num_points):
            # Generate random points more likely to be within workspace
            # Using a cone-like distribution
            
            # Random radius with square distribution (more points near center)
            r = np.random.random() ** 0.5 * 25  # Max radius of ~25mm
            
            # Random angle
            theta = np.random.random() * 2 * np.pi
            
            # Convert to cartesian
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            
            # Height biased toward reasonable workspace heights
            z = np.random.uniform(12, 30)
            
            positions.append(np.array([x, y, z]))
            
        return positions
    
    def process_positions(self, positions: List[np.ndarray]) -> Tuple[List[Optional[dict]], float]:
        """Process a list of positions and time the performance"""
        start_time = time.time()
        results = []
        
        for pos in positions:
            if self.using_cpp:
                # C++ version returns a list of values
                cpp_result = self.math_engine.calculate_joint_values(pos.tolist())
                
                if cpp_result is not None:
                    # Convert C++ result to dict format
                    result = {
                        'pitch': cpp_result[0],
                        'roll': cpp_result[1],
                        'motor_A': cpp_result[2],
                        'motor_B': cpp_result[3],
                        'motor_C': cpp_result[4],
                        'prismatic_length': cpp_result[5],
                        'fermat_point': [cpp_result[6], cpp_result[7], cpp_result[8]],
                        'within_limits': bool(cpp_result[9]),
                        'original_target': pos,
                        'corrected_target': [cpp_result[13], cpp_result[14], cpp_result[15]],
                        'workspace_corrected': bool(cpp_result[16])
                    }
                else:
                    result = None
            else:
                # Python version already returns a dict
                result = self.math_engine.calculate_joint_values(pos)
                if result is not None:
                    # Ensure original_target is set correctly
                    result['original_target'] = pos
                
            results.append(result)
            
        elapsed_time = time.time() - start_time
        return results, elapsed_time
    
    def compare_implementations(self, num_points: int = 100):
        """Compare Python and C++ implementations if both are available"""
        if not CPP_AVAILABLE:
            print("C++ module not available, can't compare implementations")
            return
            
        # Store the current engine settings
        original_engine = self.math_engine
        original_using_cpp = self.using_cpp
        
        # Generate test points
        test_points = self.generate_random_positions(num_points)
        
        # Test C++ implementation
        self.math_engine = DeltaRobotMath(
            self.robot_radius,
            self.min_height,
            self.working_height,
            self.motor_limit,
            self.resting_position,
            self.workspace_cone_angle_rad
        )
        self.using_cpp = True
        
        cpp_start = time.time()
        cpp_results, _ = self.process_positions(test_points)
        cpp_time = time.time() - cpp_start
        
        # Test Python implementation
        self.math_engine = PyDeltaRobotMath(enable_timing=True)
        self.using_cpp = False
        
        py_start = time.time()
        py_results, _ = self.process_positions(test_points)
        py_time = time.time() - py_start
        
        # Compare results
        matching_results = 0
        for cpp_res, py_res in zip(cpp_results, py_results):
            if cpp_res is None and py_res is None:
                matching_results += 1
            elif cpp_res is not None and py_res is not None:
                # Check if results are close enough (note: Python implementation is just a placeholder)
                # In a real implementation, you'd want actual Python calculations here
                if (abs(cpp_res['pitch'] - py_res['pitch']) < 1e-5 and
                    abs(cpp_res['roll'] - py_res['roll']) < 1e-5 and
                    abs(cpp_res['motor_A'] - py_res['motor_A']) < 1e-5 and
                    abs(cpp_res['motor_B'] - py_res['motor_B']) < 1e-5 and
                    abs(cpp_res['motor_C'] - py_res['motor_C']) < 1e-5):
                    matching_results += 1
        
        # Restore the original engine
        self.math_engine = original_engine
        self.using_cpp = original_using_cpp
        
        # Print comparison results
        print(f"=== Implementation Comparison ({num_points} points) ===")
        print(f"C++ Implementation: {cpp_time:.6f} seconds")
        print(f"Python Implementation: {py_time:.6f} seconds")
        print(f"Speed Improvement: {py_time/cpp_time:.2f}x faster")
        print(f"Matching Results: {matching_results}/{num_points}")
        print("Note: Python implementation is a placeholder with dummy values")
        
        return {
            'cpp_time': cpp_time,
            'py_time': py_time,
            'speedup': py_time/cpp_time,
            'matching_results': matching_results,
            'total_points': num_points
        }
