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
    parser.add_argument('--compare', action='store_true',
                        help='Compare Python and C++ implementations')
    parser.add_argument('--save-plot', type=str, default='',
                        help='Save the visualization to a file')
    args = parser.parse_args()
    
    # Report on available implementations
    if CPP_AVAILABLE:
        print("C++ implementation is available")
    else:
        print("C++ implementation is NOT available, using Python only")
    
    # Create the position generator
    generator = PositionGenerator()
    
    # Generate random positions
    print(f"Generating {args.num_points} random positions...")
    positions = generator.generate_random_positions(args.num_points)
    
    # Process the positions
    print("Processing positions...")
    results, elapsed_time = generator.process_positions(positions)
    
    # Print statistics
    valid_results = sum(1 for r in results if r is not None)
    corrections = sum(1 for r in results if r is not None and r['workspace_corrected'])
    
    print("\n=== Results ===")
    print(f"Total points processed: {args.num_points}")
    print(f"Valid results: {valid_results} ({valid_results/args.num_points:.1%})")
    print(f"Workspace corrections: {corrections} ({corrections/args.num_points:.1%})")
    print(f"Total processing time: {elapsed_time:.6f} seconds")
    print(f"Average time per point: {(elapsed_time/args.num_points)*1000:.6f} ms")
    print(f"Points processed per second: {args.num_points/elapsed_time:.1f}")
    
    # Compare implementations if requested
    if args.compare:
        if CPP_AVAILABLE:
            comparison = generator.compare_implementations(args.num_points)
            
            # Print more detailed timing info
            print("\n=== Detailed C++ Timing ===")
            if hasattr(generator.math_engine, 'get_last_operation_stats'):
                stats = generator.math_engine.get_last_operation_stats()
                print(f"Verify and correct: {stats.verify_and_correct_ms:.3f} ms")
                print(f"Calculate top positions: {stats.calculate_top_positions_ms:.3f} ms")
                print(f"Calculate Fermat point: {stats.calculate_fermat_ms:.3f} ms")
                print(f"Optimization: {stats.optimization_ms:.3f} ms")
                print(f"Total: {stats.total_ms:.3f} ms")
        else:
            print("\nCannot compare implementations: C++ module not available")
    
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

if __name__ == "__main__":
    main()
