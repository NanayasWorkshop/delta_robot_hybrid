"""
Visualizer for Delta Robot positions and results
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Dict, Optional

class DeltaVisualizer:
    """Visualize delta robot positions and results"""
    
    def __init__(self, 
                 robot_radius: float = 24.8, 
                 min_height: float = 101.0,
                 working_height: float = 11.5):
        """Initialize with robot parameters"""
        self.robot_radius = robot_radius
        self.min_height = min_height
        self.working_height = working_height
        self.fig = None
        self.ax = None
    
    def visualize_workspace(self):
        """Visualize the robot workspace as a cone"""
        if self.fig is None:
            self.fig = plt.figure(figsize=(10, 8))
            self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Plot the base of the robot (circle)
        theta = np.linspace(0, 2*np.pi, 100)
        x = self.robot_radius * np.cos(theta)
        y = self.robot_radius * np.sin(theta)
        z = np.zeros_like(theta) + self.min_height
        self.ax.plot(x, y, z, 'b-', alpha=0.5)
        
        # Plot the workspace cone
        h = 40  # Height of cone
        cone_radius = h * np.tan(30 * np.pi / 180)  # 30 degree cone
        
        # Draw cone base
        cone_base_z = self.working_height + h
        x_cone = cone_radius * np.cos(theta)
        y_cone = cone_radius * np.sin(theta)
        z_cone = np.zeros_like(theta) + cone_base_z
        self.ax.plot(x_cone, y_cone, z_cone, 'g-', alpha=0.5)
        
        # Draw cone apex
        self.ax.plot([0], [0], [self.working_height], 'go', markersize=5)
        
        # Draw cone lines
        for i in range(0, 100, 10):
            self.ax.plot([0, x_cone[i]], [0, y_cone[i]], 
                         [self.working_height, cone_base_z], 'g-', alpha=0.2)
        
        # Plot robot base joints
        base_angles = [0, -np.pi/6, -5*np.pi/6]
        base_x = [self.robot_radius * np.cos(angle) for angle in base_angles]
        base_y = [self.robot_radius * np.sin(angle) for angle in base_angles]
        base_z = [self.min_height] * 3
        
        self.ax.scatter(base_x, base_y, base_z, color='blue', s=50, marker='o')
        
        # Set labels and limits
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        
        # Set equal aspect ratio for all axes
        self.ax.set_box_aspect([1, 1, 1])
        
        # Set limits
        limit = max(self.robot_radius, cone_radius) * 1.2
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_zlim(0, cone_base_z * 1.2)
        
    def plot_points(self, 
                    points: List[np.ndarray], 
                    results: List[Optional[Dict]] = None,
                    title: str = "Delta Robot Points"):
        """Plot points in 3D space with color coding based on results"""
        self.visualize_workspace()
        
        # Separate points based on results
        if results:
            valid_points = []
            invalid_points = []
            corrected_points = []
            
            for i, (point, result) in enumerate(zip(points, results)):
                if result is None:
                    invalid_points.append(point)
                elif result['workspace_corrected']:
                    # Store both original and corrected
                    invalid_points.append(point)
                    corrected_points.append(np.array(result['corrected_target']))
                else:
                    valid_points.append(point)
            
            # Plot points with different colors
            if valid_points:
                valid_points = np.array(valid_points)
                self.ax.scatter(valid_points[:, 0], valid_points[:, 1], valid_points[:, 2], 
                           color='green', label='Valid', alpha=0.7)
            
            if invalid_points:
                invalid_points = np.array(invalid_points)
                self.ax.scatter(invalid_points[:, 0], invalid_points[:, 1], invalid_points[:, 2], 
                           color='red', label='Invalid/Original', alpha=0.7)
            
            if corrected_points:
                corrected_points = np.array(corrected_points)
                self.ax.scatter(corrected_points[:, 0], corrected_points[:, 1], corrected_points[:, 2], 
                           color='orange', label='Corrected', alpha=0.7)
                
                # Draw lines between original and corrected points
                for i, (point, result) in enumerate(zip(points, results)):
                    if result and result['workspace_corrected']:
                        corrected = np.array(result['corrected_target'])
                        self.ax.plot([point[0], corrected[0]], 
                                [point[1], corrected[1]], 
                                [point[2], corrected[2]], 'k-', alpha=0.3)
        else:
            # Plot all points in blue if no results
            points_array = np.array(points)
            self.ax.scatter(points_array[:, 0], points_array[:, 1], points_array[:, 2], 
                       color='blue', label='Points', alpha=0.7)
        
        self.ax.legend()
        self.ax.set_title(title)
        plt.tight_layout()
        
    def show(self):
        """Show the plot"""
        plt.show()
        
    def save(self, filename: str):
        """Save the plot to a file"""
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"Saved plot to {filename}")
