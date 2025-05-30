"""
Position generator for delta robot testing
"""

import numpy as np
from typing import List, Tuple, Optional
import time

# Import C++ module (will be available after building)
try:
    from .delta_robot_cpp import DeltaRobotMath, constants
    CPP_AVAILABLE = True
except ImportError:
    print("Warning: C++ module not found")
    CPP_AVAILABLE = False

class PositionGenerator:
    """Generates random positions for delta robot testing"""
    
    def __init__(self, 
                 robot_radius: float = None,
                 min_height: float = None,
                 working_height: float = None,
                 motor_limit: float = None):
        """Initialize with robot parameters (uses constants if not provided)"""
        
        if not CPP_AVAILABLE:
            raise ImportError("C++ module is required for PositionGenerator")
        
        # Use constants from C++ module
        self.robot_radius = robot_radius or constants.ROBOT_RADIUS
        self.min_height = min_height or constants.MIN_HEIGHT
        self.working_height = working_height or constants.WORKING_HEIGHT
        self.motor_limit = motor_limit or constants.MOTOR_LIMIT
        self.workspace_cone_angle_rad = constants.WORKSPACE_CONE_ANGLE_RAD
        
        self.resting_position = 2 * self.motor_limit + self.min_height
        
        # Initialize the math engine
        self.math_engine = DeltaRobotMath(
            self.robot_radius,
            self.min_height,
            self.working_height,
            self.motor_limit,
            self.resting_position,
            self.workspace_cone_angle_rad
        )
    
    def generate_random_positions(self, num_points: int = 100) -> List[np.ndarray]:
        """Generate random positions within the expected workspace"""
        np.random.seed(42)  # For reproducible results
        
        positions = []
        for _ in range(num_points):
            # Generate random points more likely to be within workspace
            # Using a cone-like distribution
            
            # Random radius with square distribution (more points near center)
            r = np.random.random() ** 0.5 * 50  # Max radius of ~25mm
            
            # Random angle
            theta = np.random.random() * 2 * np.pi
            
            # Convert to cartesian
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            
            # Height biased toward reasonable workspace heights
            z = np.random.uniform(50, 120)
            
            positions.append(np.array([x, y, z]))
            
        return positions
    
    def process_positions(self, positions: List[np.ndarray]) -> Tuple[List[Optional[dict]], float]:
        """Process a list of positions and time the performance"""
        start_time = time.time()
        results = []
        
        for pos in positions:
            # C++ version returns a structured result
            cpp_result = self.math_engine.calculate_joint_values(pos.tolist())
            
            if cpp_result is not None:
                # Convert C++ CalculationResult to dict format
                result = {
                    'pitch': cpp_result.pitch,
                    'roll': cpp_result.roll,
                    'motor_A': cpp_result.motor_positions[0],
                    'motor_B': cpp_result.motor_positions[1],
                    'motor_C': cpp_result.motor_positions[2],
                    'prismatic_length': cpp_result.prismatic_length,
                    'fermat_point': list(cpp_result.fermat_point),
                    'within_limits': cpp_result.within_limits,
                    'original_target': list(cpp_result.original_target),
                    'corrected_target': list(cpp_result.corrected_target),
                    'workspace_corrected': cpp_result.workspace_corrected
                }
            else:
                result = None
                
            results.append(result)
            
        elapsed_time = time.time() - start_time
        return results, elapsed_time
    
    def process_positions_legacy(self, positions: List[np.ndarray]) -> Tuple[List[Optional[dict]], float]:
        """Process positions using legacy vector format (for backward compatibility testing)"""
        start_time = time.time()
        results = []
        
        for pos in positions:
            # Use legacy vector format
            cpp_result = self.math_engine.calculate_joint_values_legacy(pos.tolist())
            
            if cpp_result is not None:
                # Convert legacy vector format to dict (old way with magic indices)
                result = {
                    'pitch': cpp_result[0],
                    'roll': cpp_result[1],
                    'motor_A': cpp_result[2],
                    'motor_B': cpp_result[3],
                    'motor_C': cpp_result[4],
                    'prismatic_length': cpp_result[5],
                    'fermat_point': [cpp_result[6], cpp_result[7], cpp_result[8]],
                    'within_limits': bool(cpp_result[9]),
                    'original_target': [cpp_result[10], cpp_result[11], cpp_result[12]],
                    'corrected_target': [cpp_result[13], cpp_result[14], cpp_result[15]],
                    'workspace_corrected': bool(cpp_result[16])
                }
            else:
                result = None
                
            results.append(result)
            
        elapsed_time = time.time() - start_time
        return results, elapsed_time
    
    def compare_formats(self, num_points: int = 100):
        """Compare structured vs legacy format performance and accuracy"""
        if not CPP_AVAILABLE:
            print("C++ module not available, can't compare formats")
            return
            
        # Generate test points
        test_points = self.generate_random_positions(num_points)
        
        # Test structured format
        structured_start = time.time()
        structured_results, _ = self.process_positions(test_points)
        structured_time = time.time() - structured_start
        
        # Test legacy format
        legacy_start = time.time()
        legacy_results, _ = self.process_positions_legacy(test_points)
        legacy_time = time.time() - legacy_start
        
        # Compare structured vs legacy results
        matching_formats = 0
        for struct_res, legacy_res in zip(structured_results, legacy_results):
            if struct_res is None and legacy_res is None:
                matching_formats += 1
            elif struct_res is not None and legacy_res is not None:
                # Check if structured and legacy formats produce same results
                if (abs(struct_res['pitch'] - legacy_res['pitch']) < 1e-10 and
                    abs(struct_res['roll'] - legacy_res['roll']) < 1e-10 and
                    abs(struct_res['motor_A'] - legacy_res['motor_A']) < 1e-10 and
                    abs(struct_res['motor_B'] - legacy_res['motor_B']) < 1e-10 and
                    abs(struct_res['motor_C'] - legacy_res['motor_C']) < 1e-10):
                    matching_formats += 1
        
        # Print comparison results
        print(f"=== Format Comparison ({num_points} points) ===")
        print(f"Structured Format: {structured_time:.6f} seconds")
        print(f"Legacy Format: {legacy_time:.6f} seconds")
        if legacy_time > 0:
            print(f"Legacy overhead: {((structured_time/legacy_time)-1)*100:.1f}% slower")
        print(f"Matching results: {matching_formats}/{num_points}")
        
        return {
            'structured_time': structured_time,
            'legacy_time': legacy_time,
            'overhead_percent': ((structured_time/legacy_time)-1)*100 if legacy_time > 0 else 0,
            'matching_results': matching_formats,
            'total_points': num_points
        }