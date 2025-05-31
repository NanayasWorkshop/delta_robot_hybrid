"""
Delta Robot Math Visualizer - Hooks into existing C++ calculations
Save as: python/math_visualizer.py
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import Dict, List, Optional, Tuple
import sys
import os

# Import your existing delta robot module
try:
    from .delta_robot_cpp import DeltaRobotMath, constants
    CPP_AVAILABLE = True
except ImportError:
    try:
        # Fallback for direct execution
        from delta_robot_cpp import DeltaRobotMath, constants
        CPP_AVAILABLE = True
    except ImportError:
        print("Warning: C++ module not available")
        CPP_AVAILABLE = False

class DeltaMathVisualizer:
    """Visualizes intermediate calculations from existing C++ delta robot math"""
    
    def __init__(self):
        if not CPP_AVAILABLE:
            raise ImportError("C++ module required for visualization")
        
        # Use your existing robot setup
        self.robot_radius = constants.ROBOT_RADIUS
        self.working_height = constants.WORKING_HEIGHT
        self.workspace_cone_angle_rad = constants.WORKSPACE_CONE_ANGLE_RAD
        
        # Initialize your existing math engine
        resting_position = 2 * constants.MOTOR_LIMIT + constants.MIN_HEIGHT
        self.math_engine = DeltaRobotMath(
            constants.ROBOT_RADIUS,
            constants.MIN_HEIGHT, 
            constants.WORKING_HEIGHT,
            constants.MOTOR_LIMIT,
            resting_position,
            constants.WORKSPACE_CONE_ANGLE_RAD
        )
        
        # Base positions from constants
        self.base_positions = [
            [self.robot_radius * np.cos(constants.BASE_A_ANGLE), 
             self.robot_radius * np.sin(constants.BASE_A_ANGLE)],
            [self.robot_radius * np.cos(constants.BASE_B_ANGLE), 
             self.robot_radius * np.sin(constants.BASE_B_ANGLE)],
            [self.robot_radius * np.cos(constants.BASE_C_ANGLE), 
             self.robot_radius * np.sin(constants.BASE_C_ANGLE)]
        ]
    
    def extract_step2_data(self, target_point: List[float]) -> Dict:
        """
        Extract Step 2 intermediate calculations by mimicking C++ logic
        This reverse-engineers what your C++ code does internally
        """
        target_x, target_y, target_z = target_point
        
        # Step 2a: Direction vector (from your C++ code)
        direction_vector = np.array([target_x, target_y, target_z])
        direction_length = np.linalg.norm(direction_vector)
        
        if direction_length < constants.EPSILON:
            plane_normal = np.array([0, 0, 1])
        else:
            plane_normal = direction_vector / direction_length
        
        # Step 2b: Plane center (from your C++ calculateTopPositions)
        plane_center = direction_vector / 2
        
        # Step 2c: Mirror working height point (from your C++ logic)
        H = np.array([0, 0, self.working_height])
        H_to_center = H - plane_center
        projection_length = np.dot(H_to_center, plane_normal)
        G = H - 2 * projection_length * plane_normal
        
        # Step 2d: Calculate u vector and heights (from your C++)
        u = G - H
        u_x, u_y, u_z = u
        
        # Extract actuator heights using your formula
        actuator_heights = []
        if abs(u_z) > constants.EPSILON:
            for base_pos in self.base_positions:
                height = -((u_x * base_pos[0] + u_y * base_pos[1]) / u_z)
                actuator_heights.append(height)
        else:
            actuator_heights = [constants.MIN_HEIGHT] * 3
        
        # Create 3D actuator positions
        actuator_3d_positions = [
            [self.base_positions[i][0], self.base_positions[i][1], actuator_heights[i]]
            for i in range(3)
        ]
        
        return {
            'target_point': target_point,
            'direction_vector': direction_vector,
            'direction_length': direction_length,
            'plane_normal': plane_normal,
            'plane_center': plane_center,
            'H': H,
            'G': G,
            'u_vector': u,
            'actuator_heights': actuator_heights,
            'actuator_3d_positions': actuator_3d_positions
        }
    
    def extract_step3_data(self, actuator_3d_positions: List[List[float]]) -> Dict:
        """
        Extract Step 3 Fermat point calculations by mimicking C++ logic
        """
        A_Point = np.array(actuator_3d_positions[0])
        B_Point = np.array(actuator_3d_positions[1])
        C_Point = np.array(actuator_3d_positions[2])
        
        # Step 3a: Calculate vectors and side lengths (from your C++ calculateFermatFromPoints)
        AB = B_Point - A_Point
        BC = C_Point - B_Point  
        CA = A_Point - C_Point
        
        a = np.linalg.norm(BC)  # Length opposite to A
        b = np.linalg.norm(CA)  # Length opposite to B
        c = np.linalg.norm(AB)  # Length opposite to C
        
        # Step 3b: Calculate angles (from your C++ logic)
        def safe_acos(x):
            return np.arccos(np.clip(x, constants.TRIG_CLAMP_MIN, constants.TRIG_CLAMP_MAX))
        
        # Your C++ angle calculations
        CA_dot_AB = -np.dot(CA, AB)
        AB_dot_BC = -np.dot(AB, BC)
        BC_dot_CA = -np.dot(BC, CA)
        
        CA_norm = np.linalg.norm(CA)
        AB_norm = np.linalg.norm(AB)
        BC_norm = np.linalg.norm(BC)
        
        Alpha = safe_acos(CA_dot_AB / (CA_norm * AB_norm))
        Beta = safe_acos(AB_dot_BC / (AB_norm * BC_norm))
        Gamma = safe_acos(BC_dot_CA / (BC_norm * CA_norm))
        
        # Step 3c: Calculate lambdas (from your C++ constants)
        def safe_sin_divide(numerator, angle):
            denominator = np.sin(angle + constants.FERMAT_ANGLE_OFFSET)
            return numerator / max(denominator, constants.FERMAT_MIN_DENOMINATOR)
        
        LambdaA = safe_sin_divide(a, Alpha)
        LambdaB = safe_sin_divide(b, Beta) 
        LambdaC = safe_sin_divide(c, Gamma)
        
        # Calculate Fermat point (from your C++)
        total_lambda = LambdaA + LambdaB + LambdaC
        Fermat_point = (LambdaA * A_Point + LambdaB * B_Point + LambdaC * C_Point) / total_lambda
        
        # Calculate distances from Fermat point
        dist_A = np.linalg.norm(Fermat_point - A_Point)
        dist_B = np.linalg.norm(Fermat_point - B_Point)
        dist_C = np.linalg.norm(Fermat_point - C_Point)
        
        return {
            'A_Point': A_Point,
            'B_Point': B_Point,
            'C_Point': C_Point,
            'side_vectors': {'AB': AB, 'BC': BC, 'CA': CA},
            'side_lengths': {'a': a, 'b': b, 'c': c},
            'angles': {'Alpha': Alpha, 'Beta': Beta, 'Gamma': Gamma},
            'lambdas': {'LambdaA': LambdaA, 'LambdaB': LambdaB, 'LambdaC': LambdaC},
            'fermat_point': Fermat_point,
            'distances': {'dist_A': dist_A, 'dist_B': dist_B, 'dist_C': dist_C}
        }
    
    def visualize_complete_calculation(self, target_point: List[float], 
                                     save_path: Optional[str] = None) -> Dict:
        """
        Complete visualization using YOUR existing C++ calculations + intermediate data
        """
        print(f"Analyzing target point: {target_point}")
        
        # Get the official result from YOUR C++ code
        cpp_result = self.math_engine.calculate_joint_values(target_point)
        if cpp_result is None:
            print("C++ calculation failed!")
            return {}
        
        print("✓ C++ calculation successful")
        
        # Extract intermediate data by mimicking your C++ logic
        step2_data = self.extract_step2_data(target_point)
        step3_data = self.extract_step3_data(step2_data['actuator_3d_positions'])
        
        # Create 2-column layout: large 3D plot left, summary + small 3D plot right
        fig = plt.figure(figsize=(20, 10))
        fig.suptitle(f'Delta Robot Math Visualization - Target: {target_point}', fontsize=16)
        
        # Left column: Step 2 - Top Position Calculation (3D) - Large
        ax1 = plt.subplot2grid((2, 2), (0, 0), rowspan=2, projection='3d')
        self._plot_step2_3d(ax1, step2_data)
        
        # Right top: Final Results
        ax2 = plt.subplot2grid((2, 2), (0, 1))
        self._plot_final_results(ax2, cpp_result, step2_data, step3_data)
        
        # Right bottom: Step 3 - Fermat Point Calculation (3D) - Smaller
        ax3 = plt.subplot2grid((2, 2), (1, 1), projection='3d')
        self._plot_step3_3d(ax3, step3_data)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Visualization saved to {save_path}")
        
        # Return all data for further analysis
        return {
            'cpp_result': cpp_result,
            'step2_data': step2_data,
            'step3_data': step3_data,
            'target_point': target_point
        }
    
    def _plot_step2_3d(self, ax, data):
        """3D visualization of Step 2 calculations"""
        # Robot base circle
        theta = np.linspace(0, 2*np.pi, 100)
        base_x = self.robot_radius * np.cos(theta)
        base_y = self.robot_radius * np.sin(theta)
        base_z = np.zeros_like(theta)
        ax.plot(base_x, base_y, base_z, 'k-', alpha=0.5)
        
        # Base positions
        colors = ['red', 'green', 'blue']
        for i, (base_pos, color) in enumerate(zip(self.base_positions, colors)):
            ax.scatter(base_pos[0], base_pos[1], 0, color=color, s=100, 
                      label=f'Base {["A", "B", "C"][i]}')
        
        # Key points from your calculations
        ax.scatter(*data['target_point'], color='orange', s=150, marker='*', label='Target')
        ax.scatter(*data['H'], color='purple', s=100, marker='s', label='H (working height)')
        ax.scatter(*data['G'], color='red', s=100, marker='^', label='G (mirrored)')
        ax.scatter(*data['plane_center'], color='cyan', s=100, marker='o', label='Plane center')
        
        # Actuator positions from your calculations
        for i, (pos, color) in enumerate(zip(data['actuator_3d_positions'], colors)):
            ax.scatter(*pos, color=color, s=120, marker='D', alpha=0.8)
            # Connect base to actuator
            base_pos = self.base_positions[i]
            ax.plot([base_pos[0], pos[0]], [base_pos[1], pos[1]], [0, pos[2]], 
                   color=color, linewidth=3, alpha=0.7)
        
        # Vectors and lines from your calculations
        # Direction vector as dotted yellow line
        ax.plot([0, data['direction_vector'][0]], [0, data['direction_vector'][1]], 
               [0, data['direction_vector'][2]], 'y--', linewidth=3, label='Direction vector')
        
        # Connect origin to H with purple line
        ax.plot([0, data['H'][0]], [0, data['H'][1]], [0, data['H'][2]], 
               'purple', linewidth=3, label='Origin to H')
        
        # Connect G to Target with colored line
        ax.plot([data['G'][0], data['target_point'][0]], 
               [data['G'][1], data['target_point'][1]], 
               [data['G'][2], data['target_point'][2]], 
               'orange', linewidth=3, label='G to Target')
        
        ax.plot([data['H'][0], data['G'][0]], [data['H'][1], data['G'][1]], 
               [data['H'][2], data['G'][2]], 'r--', linewidth=3, label='H-G line')
        
        ax.set_title('Step 2: Top Position Calculation')
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    def _plot_step3_3d(self, ax, data):
        """3D visualization of Fermat point calculation"""
        A, B, C = data['A_Point'], data['B_Point'], data['C_Point']
        fermat = data['fermat_point']
        
        # Triangle vertices
        colors = ['red', 'green', 'blue']
        points = [A, B, C]
        labels = ['A', 'B', 'C']
        
        for point, color, label in zip(points, colors, labels):
            ax.scatter(*point, color=color, s=150, label=f'Actuator {label}')
        
        # Fermat point
        ax.scatter(*fermat, color='magenta', s=200, marker='*', label='Fermat Point')
        
        # Triangle edges
        ax.plot([A[0], B[0]], [A[1], B[1]], [A[2], B[2]], 'k-', linewidth=2, alpha=0.7)
        ax.plot([B[0], C[0]], [B[1], C[1]], [B[2], C[2]], 'k-', linewidth=2, alpha=0.7)
        ax.plot([C[0], A[0]], [C[1], A[1]], [C[2], A[2]], 'k-', linewidth=2, alpha=0.7)
        
        # Lines from Fermat to vertices
        for point, color in zip(points, colors):
            ax.plot([fermat[0], point[0]], [fermat[1], point[1]], [fermat[2], point[2]], 
                   color=color, linewidth=2, linestyle='--', alpha=0.7)
        
        # Distance labels
        distances = [data['distances']['dist_A'], data['distances']['dist_B'], data['distances']['dist_C']]
        for i, (point, dist) in enumerate(zip(points, distances)):
            mid = (fermat + point) / 2
            ax.text(*mid, f'{dist:.1f}', fontsize=8)
        
        ax.set_title('Step 3: Fermat Point Calculation')
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.legend()
    
    def _plot_final_results(self, ax, cpp_result, step2_data, step3_data):
        """Summary of final results"""
        ax.axis('off')
        
        # Calculate additional distances
        origin_to_H_distance = np.linalg.norm(step2_data['H'])
        G_to_target_distance = np.linalg.norm(np.array(step2_data['target_point']) - step2_data['G'])
        
        # Create summary text
        summary = f"""
FINAL RESULTS FROM YOUR C++ CALCULATION:

Target Point: {step2_data['target_point']}
Corrected Target: {list(cpp_result.corrected_target)}
Workspace Corrected: {cpp_result.workspace_corrected}

STEP 2 RESULTS:
Actuator Heights: {[f'{h:.2f}' for h in step2_data['actuator_heights']]}
Motor Positions: {[f'{m:.2f}' for m in cpp_result.motor_positions]}

STEP 3 RESULTS:
Fermat Point: {[f'{f:.2f}' for f in step3_data['fermat_point']]}
Your C++ Fermat: {[f'{f:.2f}' for f in cpp_result.fermat_point]}

KEY DISTANCES:
Origin to H: {origin_to_H_distance:.2f} mm
G to Target: {G_to_target_distance:.2f} mm

FINAL OUTPUT:
Pitch: {cpp_result.pitch:.4f} rad ({np.degrees(cpp_result.pitch):.2f}°)
Roll: {cpp_result.roll:.4f} rad ({np.degrees(cpp_result.roll):.2f}°)
Prismatic Length: {cpp_result.prismatic_length:.2f} mm
Within Limits: {cpp_result.within_limits}

PERFORMANCE:
"""
        
        # Add timing if available
        try:
            stats = self.math_engine.get_last_operation_stats()
            summary += f"Total Time: {stats.total_ms:.3f} ms\n"
            summary += f"Step 2 (Top Positions): {stats.calculate_top_positions_ms:.3f} ms\n"
            summary += f"Step 3 (Fermat): {stats.calculate_fermat_ms:.3f} ms"
        except:
            summary += "Timing data not available"
        
        ax.text(0.05, 0.95, summary, transform=ax.transAxes, va='top', ha='left',
               fontfamily='monospace', fontsize=9,
               bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        ax.set_title('Summary: Your C++ Results')

# Example usage function
def demo_visualization():
    """Demo function showing how to use the visualizer"""
    if not CPP_AVAILABLE:
        print("C++ module not available!")
        return
    
    # Create visualizer
    viz = DeltaMathVisualizer()
    
    # Test points
    test_points = [
        [10.0, 5.0, 20.0],   # Good point
        [0.0, 0.0, 15.0],    # Center point  
        [25.0, 25.0, 30.0],  # Edge case
    ]
    
    for i, point in enumerate(test_points):
        print(f"\n=== Visualizing Point {i+1}: {point} ===")
        
        # Create visualization 
        data = viz.visualize_complete_calculation(
            point, 
            save_path=f"delta_math_visualization_{i+1}.png"
        )
        
        if data:
            print("✓ Visualization complete!")
        
        # Show the plot
        plt.show()

if __name__ == "__main__":
    demo_visualization()