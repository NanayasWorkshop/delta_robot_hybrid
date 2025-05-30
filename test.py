#!/usr/bin/env python3
"""
Test script for Delta Robot hybrid Python/C++ implementation
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

def main():
    """Main test function"""
    parser = argparse.ArgumentParser(description='Test Delta Robot Calculations')
    parser.add_argument('--num-points', type=int, default=1000, 
                        help='Number of random points to generate')
    parser.add_argument('--visualize', action='store_true',
                        help='Visualize the results')
    parser.add_argument('--compare-formats', action='store_true',
                        help='Compare structured vs legacy format performance')
    parser.add_argument('--test-legacy', action='store_true',
                        help='Test legacy vector format compatibility')
    parser.add_argument('--save-plot', type=str, default='',
                        help='Save the visualization to a file')
    args = parser.parse_args()
    
    # Check if C++ module is available
    if not CPP_AVAILABLE:
        print("ERROR: C++ implementation is required but not available")
        print("Please build the C++ module first")
        return 1
    
    print("C++ implementation is available")
    
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