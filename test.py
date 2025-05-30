#!/usr/bin/env python3
"""
Test script for Delta Robot hybrid Python/C++ implementation
Now includes mathematical step visualization
"""

import time
import numpy as np
import argparse
import os
import sys

# Add the current directory to the path so we can import the python module
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import our package
from python import PositionGenerator, DeltaVisualizer, CPP_AVAILABLE

# Try to import math visualizer
try:
    from python.math_visualizer import DeltaMathVisualizer
    MATH_VIZ_AVAILABLE = True
except ImportError:
    print("Math visualizer not available")
    MATH_VIZ_AVAILABLE = False

def main():
    """Main test function"""
    parser = argparse.ArgumentParser(description='Test Delta Robot Calculations')
    parser.add_argument('--num-points', type=int, default=1000, 
                        help='Number of random points to generate')
    parser.add_argument('--visualize', action='store_true',
                        help='Visualize the results')
    parser.add_argument('--visualize-math', action='store_true',
                        help='Visualize the mathematical steps (Step 2 & 3)')
    parser.add_argument('--math-points', type=str, default='10,5,20',
                        help='Point to visualize math steps for (format: x,y,z)')
    parser.add_argument('--compare-formats', action='store_true',
                        help='Compare structured vs legacy format performance')
    parser.add_argument('--test-legacy', action='store_true',
                        help='Test legacy vector format compatibility')
    parser.add_argument('--save-plot', type=str, default='',
                        help='Save the visualization to a file')
    parser.add_argument('--save-math-plot', type=str, default='',
                        help='Save the math visualization to a file')
    args = parser.parse_args()
    
    # Check if C++ module is available
    if not CPP_AVAILABLE:
        print("ERROR: C++ implementation is required but not available")
        print("Please build the C++ module first")
        return 1
    
    print("C++ implementation is available")
    
    # Math visualization if requested
    if args.visualize_math:
        if not MATH_VIZ_AVAILABLE:
            print("ERROR: Math visualizer not available")
            print("Please ensure matplotlib is installed and math_visualizer.py exists")
            return 1
        
        print("=== Mathematical Step Visualization ===")
        
        # Parse the target point
        try:
            math_point = [float(x.strip()) for x in args.math_points.split(',')]
            if len(math_point) != 3:
                raise ValueError("Point must have 3 coordinates")
        except ValueError as e:
            print(f"Error parsing math point '{args.math_points}': {e}")
            print("Use format: x,y,z (e.g., 10,5,20)")
            return 1
        
        # Create math visualizer
        try:
            math_viz = DeltaMathVisualizer()
            print(f"Visualizing mathematical steps for point: {math_point}")
            
            # Create the visualization
            save_path = args.save_math_plot if args.save_math_plot else None
            math_data = math_viz.visualize_complete_calculation(math_point, save_path)
            
            if math_data:
                print("✓ Mathematical visualization complete!")
                
                # Print some extracted data for verification
                step2 = math_data['step2_data']
                step3 = math_data['step3_data']
                cpp_result = math_data['cpp_result']
                
                print(f"\nExtracted Step 2 Data:")
                print(f"  Direction vector: {step2['direction_vector']}")
                print(f"  Actuator heights: {[f'{h:.2f}' for h in step2['actuator_heights']]}")
                
                print(f"\nExtracted Step 3 Data:")
                print(f"  Triangle sides: a={step3['side_lengths']['a']:.2f}, b={step3['side_lengths']['b']:.2f}, c={step3['side_lengths']['c']:.2f}")
                print(f"  Fermat point: {[f'{f:.2f}' for f in step3['fermat_point']]}")
                
                print(f"\nC++ Result Validation:")
                print(f"  Fermat match: {np.allclose(step3['fermat_point'], cpp_result.fermat_point, atol=1e-6)}")
                print(f"  Final pitch: {cpp_result.pitch:.4f} rad ({np.degrees(cpp_result.pitch):.2f}°)")
                print(f"  Final roll: {cpp_result.roll:.4f} rad ({np.degrees(cpp_result.roll):.2f}°)")
                
                # Show the plot if no save path specified
                if not save_path:
                    import matplotlib.pyplot as plt
                    plt.show()
                
            else:
                print("✗ Mathematical visualization failed!")
                return 1
                
        except Exception as e:
            print(f"Error creating math visualization: {e}")
            return 1
        
        # If only math visualization requested, exit here
        if not any([args.visualize, args.compare_formats, args.test_legacy, args.num_points != 1000]):
            return 0
    
    # Create the position generator
    try:
        generator = PositionGenerator()
    except ImportError as e:
        print(f"Error creating position generator: {e}")
        return 1
    
    # Generate random positions
    print(f"Generating {args.num_points} random positions...")
    positions = generator.generate_random_positions(args.num_points)
    
    # Process the positions
    print("Processing positions...")
    results, elapsed_time = generator.process_positions(positions)
    
    # Print statistics
    valid_results = sum(1 for r in results if r is not None)
    corrections = sum(1 for r in results if r is not None and r['workspace_corrected'])
    within_limits = sum(1 for r in results if r is not None and r['within_limits'])
    
    print("\n=== Results ===")
    print(f"Total points processed: {args.num_points}")
    print(f"Valid results: {valid_results} ({valid_results/args.num_points:.1%})")
    print(f"Within limits: {within_limits} ({within_limits/args.num_points:.1%})")
    print(f"Workspace corrections: {corrections} ({corrections/args.num_points:.1%})")
    print(f"Total processing time: {elapsed_time:.6f} seconds")
    print(f"Average time per point: {(elapsed_time/args.num_points)*1000:.6f} ms")
    print(f"Points processed per second: {args.num_points/elapsed_time:.1f}")
    
    # Test legacy format compatibility if requested
    if args.test_legacy:
        print("\n=== Testing Legacy Format Compatibility ===")
        legacy_results, legacy_time = generator.process_positions_legacy(positions)
        
        # Compare results
        matching_legacy = 0
        for new_res, legacy_res in zip(results, legacy_results):
            if new_res is None and legacy_res is None:
                matching_legacy += 1
            elif new_res is not None and legacy_res is not None:
                if (abs(new_res['pitch'] - legacy_res['pitch']) < 1e-10 and
                    abs(new_res['roll'] - legacy_res['roll']) < 1e-10 and
                    abs(new_res['motor_A'] - legacy_res['motor_A']) < 1e-10):
                    matching_legacy += 1
        
        print(f"Legacy format time: {legacy_time:.6f} seconds")
        print(f"Structured format time: {elapsed_time:.6f} seconds")
        if legacy_time > 0:
            print(f"Structured format overhead: {((elapsed_time/legacy_time)-1)*100:.1f}%")
        print(f"Matching results: {matching_legacy}/{args.num_points}")
    
    # Compare formats if requested
    if args.compare_formats:
        comparison = generator.compare_formats(args.num_points)
        
        # Print more detailed timing info
        print("\n=== Detailed C++ Timing ===")
        if hasattr(generator.math_engine, 'get_last_operation_stats'):
            stats = generator.math_engine.get_last_operation_stats()
            print(f"Verify and correct: {stats.verify_and_correct_ms:.3f} ms")
            print(f"Calculate top positions: {stats.calculate_top_positions_ms:.3f} ms")
            print(f"Calculate Fermat point: {stats.calculate_fermat_ms:.3f} ms")
            print(f"Optimization: {stats.optimization_ms:.3f} ms")
            print(f"Total: {stats.total_ms:.3f} ms")
    
    # Show sample results
    if valid_results > 0:
        print("\n=== Sample Results ===")
        sample_result = next(r for r in results if r is not None)
        print(f"Sample calculation:")
        print(f"  Target: {sample_result['original_target']}")
        if sample_result['workspace_corrected']:
            print(f"  Corrected: {sample_result['corrected_target']}")
        print(f"  Pitch: {sample_result['pitch']:.4f} rad")
        print(f"  Roll: {sample_result['roll']:.4f} rad")
        print(f"  Motors: [{sample_result['motor_A']:.2f}, {sample_result['motor_B']:.2f}, {sample_result['motor_C']:.2f}]")
        print(f"  Fermat: [{sample_result['fermat_point'][0]:.2f}, {sample_result['fermat_point'][1]:.2f}, {sample_result['fermat_point'][2]:.2f}]")
        print(f"  Within limits: {sample_result['within_limits']}")
    
    # Visualize if requested
    if args.visualize or args.save_plot:
        print("\nPreparing visualization...")
        visualizer = DeltaVisualizer()
        
        # Limit the number of points for visualization to avoid clutter
        vis_points = min(args.num_points, 100)
        visualizer.plot_points(positions[:vis_points], results[:vis_points], 
                               f"Delta Robot Points (n={vis_points})")
        
        if args.save_plot:
            visualizer.save(args.save_plot)
        
        if args.visualize:
            visualizer.show()

    return 0

if __name__ == "__main__":
    sys.exit(main())