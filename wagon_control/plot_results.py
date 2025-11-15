#!/usr/bin/env python3
"""
Standalone script to visualize wagon sensor data from collected runs.

This script loads IMU and GPS data from a specified run directory and generates
visualization plots including GPS trajectory and IMU sensor readings over time.
"""

import argparse
import logging
import sys
from pathlib import Path

from .visualization import plot_run_summary

# Terminal color codes - Monumental branding
TERM_ORANGE = "\033[38;2;247;72;35m"  # Monumental orange
TERM_BLUE = "\033[38;2;35;116;247m"  # Complementary blue
TERM_RESET = "\033[0m"  # Reset color


def find_latest_run(results_dir: Path) -> Path:
    """Find the most recent run directory.

    Args:
        results_dir: Path to the results directory.

    Returns:
        Path to the most recent run directory.

    Raises:
        FileNotFoundError: If no run directories are found.
    """
    if not results_dir.exists():
        raise FileNotFoundError(f"Results directory not found: {results_dir}")

    run_dirs = sorted(
        [d for d in results_dir.iterdir() if d.is_dir() and d.name.startswith("run_")]
    )

    if not run_dirs:
        raise FileNotFoundError(f"No run directories found in {results_dir}")

    return run_dirs[-1]


def list_available_runs(results_dir: Path) -> None:
    """List all available run directories.

    Args:
        results_dir: Path to the results directory.
    """
    if not results_dir.exists():
        logging.error(f"Results directory not found: {results_dir}")
        return

    run_dirs = sorted(
        [d for d in results_dir.iterdir() if d.is_dir() and d.name.startswith("run_")]
    )

    if not run_dirs:
        logging.info(f"No run directories found in {results_dir}")
        return

    logging.info("Available runs:")
    for i, run_dir in enumerate(run_dirs, 1):
        logging.info(f"  {i}. {run_dir.name}")


def main() -> None:
    """Main entry point for the plotting script."""
    # Setup basic logging
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    parser = argparse.ArgumentParser(
        description="Visualize wagon sensor data from collected runs",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Plot the most recent run
  python scripts/plot_results.py

  # Plot a specific run by name
  python scripts/plot_results.py --run run_20251114_184704

  # Plot and save figures to run directory
  python scripts/plot_results.py --save

  # List all available runs
  python scripts/plot_results.py --list
        """,
    )

    parser.add_argument(
        "--run",
        type=str,
        default=None,
        help="Name of the run directory to plot (e.g., run_20251114_184704). "
        "If not specified, plots the most recent run.",
    )

    parser.add_argument(
        "--results-dir",
        type=str,
        default="results",
        help="Path to the results directory (default: results)",
    )

    parser.add_argument(
        "--save", action="store_true", help="Save plots as PNG files in the run directory"
    )

    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not display plots interactively (useful with --save)",
    )

    parser.add_argument("--list", action="store_true", help="List all available runs and exit")

    args = parser.parse_args()

    results_dir = Path(args.results_dir)

    # Handle --list flag
    if args.list:
        list_available_runs(results_dir)
        return

    # Determine which run to plot
    if args.run:
        run_dir = results_dir / args.run
        if not run_dir.exists():
            logging.error(f"Error: Run directory not found: {run_dir}")
            logging.info("\nAvailable runs:")
            list_available_runs(results_dir)
            sys.exit(1)
    else:
        try:
            run_dir = find_latest_run(results_dir)
            logging.info(f"{TERM_BLUE}Plotting most recent run: results/{run_dir.name}{TERM_RESET}")
        except FileNotFoundError as e:
            logging.error(f"Error: {e}")
            sys.exit(1)

    # Generate plots
    try:
        plot_run_summary(run_dir=run_dir, save_plots=args.save, show_plots=not args.no_show)

        if args.save:
            logging.info(f"{TERM_BLUE}✓ Saved plots to results/{run_dir.name}/{TERM_RESET}")
    except FileNotFoundError as e:
        logging.error(f"Error: {e}")
        logging.info(f"Make sure {run_dir} contains imu_data.csv and gps_data.csv")
        sys.exit(1)
    except Exception as e:
        logging.error(f"Error generating plots: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
