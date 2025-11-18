"""Command-line interface for the analysis framework.

This module provides a simple CLI for running parameter sweeps,
generating statistical reports, and creating visualizations.
"""

import argparse
import sys
from pathlib import Path
from typing import List, Optional

from analysis import statistics, sweep, visualize


def run_sweep(args: argparse.Namespace) -> int:
    """Run a parameter sweep from command line.

    Args:
        args: Parsed command-line arguments

    Returns:
        Exit code (0 for success)
    """
    # Parse parameters from command line
    parameters = {}
    if args.param:
        for param_spec in args.param:
            # Format: PARAM_NAME=val1,val2,val3
            if '=' not in param_spec:
                print(f"Error: Invalid parameter specification: {param_spec}")
                print("Format: --param PARAM_NAME=val1,val2,val3")
                return 1

            param_name, values_str = param_spec.split('=', 1)
            try:
                values = [float(v.strip()) for v in values_str.split(',')]
            except ValueError:
                print(f"Error: Invalid parameter values for {param_name}: {values_str}")
                return 1

            parameters[param_name] = values

    if not parameters:
        print("Error: No parameters specified!")
        print("Use --param to specify parameters to sweep")
        print("Example: --param MOTOR_KP_V=1.0,1.5,2.0")
        return 1

    # Create sweep
    print(f"Setting up parameter sweep...")
    print(f"Parameters: {parameters}")
    print(f"Runs per config: {args.runs}")
    print(f"Invalid threshold: {args.threshold}m\n")

    ps = sweep.ParameterSweep(
        parameters=parameters,
        runs_per_config=args.runs,
        invalid_threshold=args.threshold,
        verbose=args.verbose
    )

    # Run sweep
    try:
        ps.run()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
        ps.restore_config()
        return 1
    except Exception as e:
        print(f"\n\nError during sweep: {e}")
        ps.restore_config()
        if args.verbose:
            raise
        return 1

    # Save results
    output_path = args.output
    if output_path:
        output_path = Path(output_path)
    else:
        output_path = None

    csv_path = ps.save_results(output_path)

    # Print summary
    ps.print_summary(top_n=args.top)

    # Generate visualizations if requested
    if args.visualize:
        print("\nGenerating visualizations...")
        visualize.visualize_csv(csv_path)

    # Generate statistical report if requested
    if args.report:
        print("\nGenerating statistical report...")
        report_path = csv_path.with_suffix('.txt')
        statistics.generate_report(csv_path, report_path, top_n=args.top)

    print(f"\n✓ Sweep complete! Results saved to {csv_path}")

    return 0


def run_visualize(args: argparse.Namespace) -> int:
    """Generate visualizations from existing CSV.

    Args:
        args: Parsed command-line arguments

    Returns:
        Exit code (0 for success)
    """
    csv_path = Path(args.csv)

    if not csv_path.exists():
        print(f"Error: File not found: {csv_path}")
        return 1

    output_dir = Path(args.output_dir) if args.output_dir else None

    try:
        visualize.visualize_csv(csv_path, output_dir)
        return 0
    except Exception as e:
        print(f"Error: {e}")
        if args.verbose:
            raise
        return 1


def run_stats(args: argparse.Namespace) -> int:
    """Generate statistical report from existing CSV.

    Args:
        args: Parsed command-line arguments

    Returns:
        Exit code (0 for success)
    """
    csv_path = Path(args.csv)

    if not csv_path.exists():
        print(f"Error: File not found: {csv_path}")
        return 1

    output_path = Path(args.output) if args.output else None

    try:
        report = statistics.generate_report(
            csv_path,
            output_path,
            top_n=args.top
        )

        if not output_path:
            print(report)

        return 0
    except Exception as e:
        print(f"Error: {e}")
        if args.verbose:
            raise
        return 1


def main(argv: Optional[List[str]] = None) -> int:
    """Main entry point for CLI.

    Args:
        argv: Optional command-line arguments (for testing)

    Returns:
        Exit code (0 for success)
    """
    parser = argparse.ArgumentParser(
        description="Analysis framework for wagon control parameter sweeps",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run a sweep testing MOTOR_KP_V with 3 values
  python -m analysis.cli sweep --param MOTOR_KP_V=1.0,1.5,2.0 --runs 10

  # Run a sweep testing multiple parameters
  python -m analysis.cli sweep \\
    --param MOTOR_KP_V=1.0,1.5,2.0 \\
    --param FOLLOWER_BASE_LOOKAHEAD=0.6,0.8,1.0 \\
    --runs 5 --visualize --report

  # Visualize existing results
  python -m analysis.cli visualize results/parameter_sweep_20250101_120000.csv

  # Generate statistical report
  python -m analysis.cli stats results/parameter_sweep_20250101_120000.csv
        """
    )

    subparsers = parser.add_subparsers(dest='command', help='Command to run')

    # Sweep command
    sweep_parser = subparsers.add_parser(
        'sweep',
        help='Run a parameter sweep'
    )
    sweep_parser.add_argument(
        '--param',
        action='append',
        help='Parameter to sweep: PARAM_NAME=val1,val2,val3 (can specify multiple)'
    )
    sweep_parser.add_argument(
        '--runs',
        type=int,
        default=10,
        help='Number of runs per configuration (default: 10)'
    )
    sweep_parser.add_argument(
        '--threshold',
        type=float,
        default=50.0,
        help='Invalid score threshold in meters (default: 50.0)'
    )
    sweep_parser.add_argument(
        '--output',
        '-o',
        help='Output CSV path (default: auto-generated in results/)'
    )
    sweep_parser.add_argument(
        '--visualize',
        action='store_true',
        help='Generate visualizations after sweep'
    )
    sweep_parser.add_argument(
        '--report',
        action='store_true',
        help='Generate statistical report after sweep'
    )
    sweep_parser.add_argument(
        '--top',
        type=int,
        default=10,
        help='Number of top configurations to show (default: 10)'
    )
    sweep_parser.add_argument(
        '--verbose',
        '-v',
        action='store_true',
        help='Enable verbose logging'
    )

    # Visualize command
    viz_parser = subparsers.add_parser(
        'visualize',
        help='Generate visualizations from CSV'
    )
    viz_parser.add_argument(
        'csv',
        help='Path to sweep results CSV file'
    )
    viz_parser.add_argument(
        '--output-dir',
        '-o',
        help='Output directory for plots (default: same as CSV)'
    )
    viz_parser.add_argument(
        '--verbose',
        '-v',
        action='store_true',
        help='Enable verbose output'
    )

    # Stats command
    stats_parser = subparsers.add_parser(
        'stats',
        help='Generate statistical report from CSV'
    )
    stats_parser.add_argument(
        'csv',
        help='Path to sweep results CSV file'
    )
    stats_parser.add_argument(
        '--output',
        '-o',
        help='Output report path (default: same as CSV with .txt extension)'
    )
    stats_parser.add_argument(
        '--top',
        type=int,
        default=10,
        help='Number of top configurations to show (default: 10)'
    )
    stats_parser.add_argument(
        '--verbose',
        '-v',
        action='store_true',
        help='Enable verbose output'
    )

    # Parse arguments
    args = parser.parse_args(argv)

    if not args.command:
        parser.print_help()
        return 1

    # Dispatch to appropriate handler
    if args.command == 'sweep':
        return run_sweep(args)
    elif args.command == 'visualize':
        return run_visualize(args)
    elif args.command == 'stats':
        return run_stats(args)
    else:
        print(f"Unknown command: {args.command}")
        return 1


if __name__ == '__main__':
    sys.exit(main())
