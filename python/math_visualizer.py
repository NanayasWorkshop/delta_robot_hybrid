"""
Delta Robot Math Visualizer - Professional Version
Uses REAL C++ calculation data instead of reverse-engineering
Professional styling without emoji decorations
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
    """Visualizes intermediate calculations from C++ delta robot math using REAL data"""
    
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
    
    def get_real_calculation_data(self, target_point: List[float]) -> Tuple[Dict, Dict]:
        """
        Get REAL C++ calculation data - no more reverse engineering!
        Returns actual intermediate data from C++ calculations
        """
        print(f"Getting real C++ calculation data for: {target_point}")
        
        # Get the official result from C++
        cpp_result = self.math_engine.calculate_joint_values(target_point)
        if cpp_result is None:
            print("C++ calculation failed!")
            return {}, {}
        
        print("C++ calculation successful")
        
        # Get REAL intermediate data from C++ kinematics module
        kinematics = self.math_engine.get_kinematics()
        kinematics_data = kinematics.get_last_calculation_data()
        
        print("Real C++ intermediate data accessed")
        
        # Extract Step 2 data (REAL from C++)
        step2_data = {
            'target_point': target_point,
            'direction_vector': list(kinematics_data.direction_vector),
            'plane_normal': list(kinematics_data.plane_normal),
            'plane_center': list(kinematics_data.plane_center),
            'H_point': list(kinematics_data.H_point),
            'G_point': list(kinematics_data.G_point),
            'u_vector': list(kinematics_data.u_vector),
            
            # Calculate actuator heights and positions from real C++ data
            'actuator_heights': self._calculate_actuator_heights_from_u_vector(kinematics_data.u_vector),
            'actuator_3d_positions': None  # Will be calculated below
        }
        
        # Calculate 3D actuator positions using real heights
        step2_data['actuator_3d_positions'] = [
            [self.base_positions[i][0], self.base_positions[i][1], step2_data['actuator_heights'][i]]
            for i in range(3)
        ]
        
        # Extract Step 3 data (REAL from C++)
        step3_data = {
            'A_Point': np.array(step2_data['actuator_3d_positions'][0]),
            'B_Point': np.array(step2_data['actuator_3d_positions'][1]),
            'C_Point': np.array(step2_data['actuator_3d_positions'][2]),
            'triangle_sides': {
                'a': kinematics_data.triangle_sides[0], 
                'b': kinematics_data.triangle_sides[1], 
                'c': kinematics_data.triangle_sides[2]
            },
            'triangle_angles': {
                'Alpha': kinematics_data.triangle_angles[0],
                'Beta': kinematics_data.triangle_angles[1], 
                'Gamma': kinematics_data.triangle_angles[2]
            },
            'lambda_weights': {
                'LambdaA': kinematics_data.lambda_weights[0],
                'LambdaB': kinematics_data.lambda_weights[1],
                'LambdaC': kinematics_data.lambda_weights[2]
            },
            'fermat_point': list(cpp_result.fermat_point),  # Use C++ calculated Fermat point
            
            # Calculate distances from C++ Fermat point
            'distances': self._calculate_distances_from_fermat(
                step2_data['actuator_3d_positions'], 
                list(cpp_result.fermat_point)
            )
        }
        
        return step2_data, step3_data
    
    def _calculate_actuator_heights_from_u_vector(self, u_vector: List[float]) -> List[float]:
        """Calculate actuator heights from real C++ u_vector"""
        u_x, u_y, u_z = u_vector
        
        # Use the same calculation as C++ (avoiding division by zero)
        if abs(u_z) < constants.EPSILON:
            return [constants.MIN_HEIGHT] * 3
        
        actuator_heights = []
        for base_pos in self.base_positions:
            height = -((u_x * base_pos[0] + u_y * base_pos[1]) / u_z)
            actuator_heights.append(height)
        
        return actuator_heights
    
    def _calculate_distances_from_fermat(self, actuator_positions: List[List[float]], 
                                        fermat_point: List[float]) -> Dict:
        """Calculate distances from Fermat point to actuators"""
        fermat = np.array(fermat_point)
        distances = {}
        
        for i, pos in enumerate(actuator_positions):
            point = np.array(pos)
            dist = np.linalg.norm(fermat - point)
            distances[f'dist_{["A", "B", "C"][i]}'] = dist
            
        return distances
    
    def visualize_complete_calculation(self, target_point: List[float], 
                                     save_path: Optional[str] = None) -> Dict:
        """
        Complete visualization using REAL C++ calculations (no more reverse engineering!)
        """
        print(f"Analyzing target point: {target_point}")
        
        # Get REAL C++ calculation data
        cpp_result = self.math_engine.calculate_joint_values(target_point)
        if cpp_result is None:
            print("C++ calculation failed!")
            return {}
        
        print("C++ calculation successful")
        
        # Get REAL intermediate data from C++
        step2_data, step3_data = self.get_real_calculation_data(target_point)
        
        # Verify we got real data
        print(f"Real Step 2 data - Direction vector: {step2_data['direction_vector'][:2]}...")
        print(f"Real Step 3 data - Triangle sides: {[f'{v:.2f}' for v in step3_data['triangle_sides'].values()]}")
        
        # Create 2-column layout: large 3D plot left, summary + small 3D plot right
        fig = plt.figure(figsize=(20, 10))
        fig.suptitle(f'Delta Robot Math Visualization - REAL C++ Data - Target: {target_point}', fontsize=16)
        
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
        
        # Return all REAL data for further analysis
        return {
            'cpp_result': cpp_result,
            'step2_data': step2_data,
            'step3_data': step3_data,
            'target_point': target_point,
            'data_source': 'REAL_CPP_DATA'  # Mark this as real data!
        }
    
    def _plot_step2_3d(self, ax, data):
        """3D visualization of Step 2 calculations using REAL C++ data"""
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
        
        # Key points from REAL C++ calculations
        ax.scatter(*data['target_point'], color='orange', s=150, marker='*', label='Target')
        ax.scatter(*data['H_point'], color='purple', s=100, marker='s', label='H (working height)')
        ax.scatter(*data['G_point'], color='red', s=100, marker='^', label='G (mirrored)')
        ax.scatter(*data['plane_center'], color='cyan', s=100, marker='o', label='Plane center')
        
        # Actuator positions from REAL C++ calculations
        for i, (pos, color) in enumerate(zip(data['actuator_3d_positions'], colors)):
            ax.scatter(*pos, color=color, s=120, marker='D', alpha=0.8)
            # Connect base to actuator
            base_pos = self.base_positions[i]
            ax.plot([base_pos[0], pos[0]], [base_pos[1], pos[1]], [0, pos[2]], 
                   color=color, linewidth=3, alpha=0.7)
        
        # Vectors and lines from REAL C++ calculations
        ax.plot([0, data['direction_vector'][0]], [0, data['direction_vector'][1]], 
               [0, data['direction_vector'][2]], 'y--', linewidth=3, label='Direction vector (REAL)')
        
        ax.plot([0, data['H_point'][0]], [0, data['H_point'][1]], [0, data['H_point'][2]], 
               'purple', linewidth=3, label='Origin to H')
        
        ax.plot([data['G_point'][0], data['target_point'][0]], 
               [data['G_point'][1], data['target_point'][1]], 
               [data['G_point'][2], data['target_point'][2]], 
               'orange', linewidth=3, label='G to Target')
        
        ax.plot([data['H_point'][0], data['G_point'][0]], [data['H_point'][1], data['G_point'][1]], 
               [data['H_point'][2], data['G_point'][2]], 'r--', linewidth=3, label='H-G line (REAL)')
        
        ax.set_title('Step 2: Top Position Calculation (REAL C++ DATA)')
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    def _plot_step3_3d(self, ax, data):
        """3D visualization of Fermat point calculation using REAL C++ data"""
        A, B, C = data['A_Point'], data['B_Point'], data['C_Point']
        fermat = np.array(data['fermat_point'])
        
        # Triangle vertices
        colors = ['red', 'green', 'blue']
        points = [A, B, C]
        labels = ['A', 'B', 'C']
        
        for point, color, label in zip(points, colors, labels):
            ax.scatter(*point, color=color, s=150, label=f'Actuator {label}')
        
        # Fermat point from REAL C++ calculation
        ax.scatter(*fermat, color='magenta', s=200, marker='*', label='Fermat Point (REAL)')
        
        # Triangle edges
        ax.plot([A[0], B[0]], [A[1], B[1]], [A[2], B[2]], 'k-', linewidth=2, alpha=0.7)
        ax.plot([B[0], C[0]], [B[1], C[1]], [B[2], C[2]], 'k-', linewidth=2, alpha=0.7)
        ax.plot([C[0], A[0]], [C[1], A[1]], [C[2], A[2]], 'k-', linewidth=2, alpha=0.7)
        
        # Lines from Fermat to vertices
        for point, color in zip(points, colors):
            ax.plot([fermat[0], point[0]], [fermat[1], point[1]], [fermat[2], point[2]], 
                   color=color, linewidth=2, linestyle='--', alpha=0.7)
        
        # Distance labels using REAL C++ data
        distances = [data['distances']['dist_A'], data['distances']['dist_B'], data['distances']['dist_C']]
        for i, (point, dist) in enumerate(zip(points, distances)):
            mid = (fermat + point) / 2
            ax.text(*mid, f'{dist:.1f}', fontsize=8)
        
        ax.set_title('Step 3: Fermat Point Calculation (REAL C++ DATA)')
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.legend()
    
    def _plot_final_results(self, ax, cpp_result, step2_data, step3_data):
        """Summary of final results using REAL C++ data"""
        ax.axis('off')
        
        # Calculate additional distances using real data
        origin_to_H_distance = np.linalg.norm(step2_data['H_point'])
        G_to_target_distance = np.linalg.norm(np.array(step2_data['target_point']) - np.array(step2_data['G_point']))
        
        # Create summary text with REAL data (professional styling)
        summary = f"""
FINAL RESULTS FROM REAL C++ CALCULATION:

Target Point: {step2_data['target_point']}
Corrected Target: {list(cpp_result.corrected_target)}
Workspace Corrected: {cpp_result.workspace_corrected}

STEP 2 RESULTS (REAL C++ DATA):
Actuator Heights: {[f'{h:.2f}' for h in step2_data['actuator_heights']]}
Motor Positions: {[f'{m:.2f}' for m in cpp_result.motor_positions]}
Direction Vector: {[f'{d:.2f}' for d in step2_data['direction_vector']]}
Plane Normal: {[f'{p:.3f}' for p in step2_data['plane_normal']]}

STEP 3 RESULTS (REAL C++ DATA):
Triangle Sides: a={step3_data['triangle_sides']['a']:.2f}, b={step3_data['triangle_sides']['b']:.2f}, c={step3_data['triangle_sides']['c']:.2f}
Triangle Angles: Alpha={step3_data['triangle_angles']['Alpha']:.3f}, Beta={step3_data['triangle_angles']['Beta']:.3f}, Gamma={step3_data['triangle_angles']['Gamma']:.3f}
Lambda Weights: A={step3_data['lambda_weights']['LambdaA']:.2f}, B={step3_data['lambda_weights']['LambdaB']:.2f}, C={step3_data['lambda_weights']['LambdaC']:.2f}
Fermat Point: {[f'{f:.2f}' for f in step3_data['fermat_point']]}

KEY DISTANCES:
Origin to H: {origin_to_H_distance:.2f} mm
G to Target: {G_to_target_distance:.2f} mm

FINAL OUTPUT:
Pitch: {cpp_result.pitch:.4f} rad ({np.degrees(cpp_result.pitch):.2f} degrees)
Roll: {cpp_result.roll:.4f} rad ({np.degrees(cpp_result.roll):.2f} degrees)
Prismatic Length: {cpp_result.prismatic_length:.2f} mm
Within Limits: {cpp_result.within_limits}

PERFORMANCE:
"""
        
        # Add timing if available
        try:
            stats = self.math_engine.get_last_operation_stats()
            summary += f"Total Time: {stats.total_ms:.3f} ms\n"
            summary += f"Step 2 (Top Positions): {stats.calculate_top_positions_ms:.3f} ms\n"
            summary += f"Step 3 (Fermat): {stats.calculate_fermat_ms:.3f} ms\n"
            summary += f"DATA SOURCE: REAL C++ CALCULATIONS (No Reverse Engineering)"
        except:
            summary += "Timing data not available\n"
            summary += f"DATA SOURCE: REAL C++ CALCULATIONS"
        
        ax.text(0.05, 0.95, summary, transform=ax.transAxes, va='top', ha='left',
               fontfamily='monospace', fontsize=8,
               bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        ax.set_title('Summary: REAL C++ Results (Professional)')

# Example usage function
def demo_visualization():
    """Demo function showing how to use the updated visualizer with REAL C++ data"""
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
        print(f"\n=== Visualizing Point {i+1}: {point} (REAL C++ DATA) ===")
        
        # Create visualization using REAL C++ data
        data = viz.visualize_complete_calculation(
            point, 
            save_path=f"delta_math_visualization_REAL_{i+1}.png"
        )
        
        if data and data.get('data_source') == 'REAL_CPP_DATA':
            print("Visualization complete using REAL C++ calculation data!")
            print(f"No more reverse engineering - data source: {data['data_source']}")
        
        # Force matplotlib to show
        try:
            plt.show(block=True)
        except Exception as e:
            print(f"Could not display plot: {e}")

if __name__ == "__main__":
    demo_visualization()