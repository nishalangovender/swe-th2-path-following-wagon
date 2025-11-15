"""Diagnostic plots for path following performance analysis.

This module loads sensor data, localized state, and reference trajectory
from CSV files and generates diagnostic plots to identify tracking errors
and localization issues.
"""

import argparse
import csv
import math
from pathlib import Path
from typing import Dict

import matplotlib.pyplot as plt
import numpy as np


def load_csv_to_dict(csv_path: Path) -> Dict[str, np.ndarray]:
    """Load CSV file into dictionary of numpy arrays.

    Args:
        csv_path: Path to CSV file

    Returns:
        Dictionary mapping column names to numpy arrays
    """
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        data = {}
        for row in reader:
            for key, value in row.items():
                if key not in data:
                    data[key] = []
                try:
                    data[key].append(float(value))
                except (ValueError, TypeError):
                    data[key].append(np.nan)

    # Convert lists to numpy arrays
    return {key: np.array(values) for key, values in data.items()}


def load_run_data(run_dir: Path) -> Dict[str, Dict[str, np.ndarray]]:
    """Load all CSV data from a run directory.

    Args:
        run_dir: Path to the run directory containing CSV files

    Returns:
        Dictionary containing data dicts for 'imu', 'gps', 'state', 'reference'
    """
    data = {}

    # Load IMU data
    imu_path = run_dir / "imu_data.csv"
    if imu_path.exists():
        data['imu'] = load_csv_to_dict(imu_path)

    # Load GPS data
    gps_path = run_dir / "gps_data.csv"
    if gps_path.exists():
        data['gps'] = load_csv_to_dict(gps_path)

    # Load localized state
    state_path = run_dir / "state_data.csv"
    if state_path.exists():
        data['state'] = load_csv_to_dict(state_path)
    else:
        print(f"Warning: {state_path} not found. State plots will be missing.")

    # Load reference trajectory
    ref_path = run_dir / "reference_data.csv"
    if ref_path.exists():
        data['reference'] = load_csv_to_dict(ref_path)
    else:
        print(f"Warning: {ref_path} not found. Reference plots will be missing.")

    return data


def plot_xy_trajectory(data: Dict[str, Dict[str, np.ndarray]], save_path: Path = None) -> None:
    """Plot XY trajectory comparison: reference vs GPS vs localized.

    Args:
        data: Dictionary containing data dicts
        save_path: Optional path to save the figure
    """
    fig, ax = plt.subplots(figsize=(10, 8))

    # Plot reference trajectory (ground truth, time-based interpretation)
    if 'reference' in data:
        ref = data['reference']
        ax.plot(ref['x_ref'], ref['y_ref'], 'k-', linewidth=2, label='Reference (time-based)', alpha=0.7)
        # Mark start and end
        ax.plot(ref['x_ref'][0], ref['y_ref'][0], 'go', markersize=10, label='Start')
        ax.plot(ref['x_ref'][-1], ref['y_ref'][-1], 'r*', markersize=15, label='End')

    # Plot GPS positions (sparse, 1 Hz)
    if 'gps' in data:
        gps = data['gps']
        ax.scatter(gps['x'], gps['y'], c='blue', s=20, alpha=0.5, label='GPS', marker='o')

    # Plot localized trajectory (continuous, 20 Hz)
    if 'state' in data:
        state = data['state']
        # Color by time for visualization
        scatter = ax.scatter(state['x_loc'], state['y_loc'], c=range(len(state['x_loc'])),
                           cmap='viridis', s=10, alpha=0.8, label='Localized')
        plt.colorbar(scatter, ax=ax, label='Sample Index (time →)')

    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title('Trajectory Comparison: Reference vs GPS vs Localized', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    ax.axis('equal')

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved plot: {save_path}")
    else:
        plt.tight_layout()


def plot_position_error(data: Dict[str, Dict[str, np.ndarray]], save_path: Path = None) -> None:
    """Plot position error over time.

    Args:
        data: Dictionary containing data dicts
        save_path: Optional path to save the figure
    """
    if 'state' not in data or 'reference' not in data:
        print("Warning: Missing state or reference data. Cannot plot position error.")
        return

    state = data['state']
    ref = data['reference']

    # Compute errors
    error_x = ref['x_ref'] - state['x_loc']
    error_y = ref['y_ref'] - state['y_loc']
    error_l2 = np.sqrt(error_x**2 + error_y**2)

    # Compute crosstrack and alongtrack errors (approximate)
    # Crosstrack: perpendicular to path direction
    # Alongtrack: along path direction

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Plot X and Y errors
    ax1.plot(ref['elapsed_time'], error_x, 'r-', label='X Error', linewidth=1.5)
    ax1.plot(ref['elapsed_time'], error_y, 'b-', label='Y Error', linewidth=1.5)
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax1.set_ylabel('Position Error (m)', fontsize=12)
    ax1.set_title('Position Error vs Time', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best')

    # Plot L2 error
    ax2.plot(ref['elapsed_time'], error_l2, 'g-', linewidth=2, label='L2 Error')
    ax2.axhline(y=np.mean(error_l2), color='orange', linestyle='--',
               label=f'Mean: {np.mean(error_l2):.3f} m', linewidth=2)
    ax2.set_xlabel('Elapsed Time (s)', fontsize=12)
    ax2.set_ylabel('L2 Error (m)', fontsize=12)
    ax2.set_title(f'Total L2 Error (Mean: {np.mean(error_l2):.3f} m)', fontsize=13)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best')

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved plot: {save_path}")
    else:
        plt.tight_layout()


def plot_heading_comparison(data: Dict[str, Dict[str, np.ndarray]], save_path: Path = None) -> None:
    """Plot heading comparison and error.

    Args:
        data: Dictionary containing data dicts
        save_path: Optional path to save the figure
    """
    if 'state' not in data or 'reference' not in data:
        print("Warning: Missing state or reference data. Cannot plot heading.")
        return

    state = data['state']
    ref = data['reference']

    # Compute heading error (normalized to [-pi, pi])
    error_theta = ref['theta_ref'] - state['theta_loc']
    error_theta = np.arctan2(np.sin(error_theta), np.cos(error_theta))  # Normalize

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Plot heading values
    ax1.plot(ref['elapsed_time'], ref['theta_ref'], 'k-', label='Reference', linewidth=2, alpha=0.7)
    ax1.plot(state['timestamp'] - state['timestamp'][0], state['theta_loc'],
            'r-', label='Localized', linewidth=1.5, alpha=0.8)
    ax1.set_ylabel('Heading (rad)', fontsize=12)
    ax1.set_title('Heading Comparison', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best')

    # Plot heading error
    ax2.plot(ref['elapsed_time'], np.degrees(error_theta), 'b-', linewidth=1.5)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.axhline(y=np.mean(np.degrees(error_theta)), color='orange', linestyle='--',
               label=f'Mean: {np.mean(np.degrees(error_theta)):.2f}°', linewidth=2)
    ax2.set_xlabel('Elapsed Time (s)', fontsize=12)
    ax2.set_ylabel('Heading Error (degrees)', fontsize=12)
    ax2.set_title('Heading Error', fontsize=13)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best')

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved plot: {save_path}")
    else:
        plt.tight_layout()


def plot_velocity_comparison(data: Dict[str, Dict[str, np.ndarray]], save_path: Path = None) -> None:
    """Plot velocity magnitude comparison.

    Args:
        data: Dictionary containing data dicts
        save_path: Optional path to save the figure
    """
    if 'state' not in data or 'reference' not in data:
        print("Warning: Missing state or reference data. Cannot plot velocity.")
        return

    state = data['state']
    ref = data['reference']

    # Compute velocity magnitudes
    v_loc = np.sqrt(state['v_x']**2 + state['v_y']**2)

    # Compute reference velocity from reference trajectory
    # v_ref = sqrt((dx/dt)^2 + (dy/dt)^2)
    # Approximate using finite differences
    dt = np.diff(ref['elapsed_time'], prepend=0.0)
    dt[0] = 0.1  # Set first value to 0.1 to avoid division by zero
    dx_dt = np.diff(ref['x_ref'], prepend=ref['x_ref'][0]) / dt
    dy_dt = np.diff(ref['y_ref'], prepend=ref['y_ref'][0]) / dt
    v_ref = np.sqrt(dx_dt**2 + dy_dt**2)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Plot velocity magnitudes
    ax1.plot(ref['elapsed_time'], v_ref, 'k-', label='Reference', linewidth=2, alpha=0.7)
    ax1.plot(state['timestamp'] - state['timestamp'][0], v_loc,
            'r-', label='Localized', linewidth=1.5, alpha=0.8)
    ax1.set_ylabel('Velocity Magnitude (m/s)', fontsize=12)
    ax1.set_title('Velocity Comparison', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best')

    # Plot velocity components
    ax2.plot(state['timestamp'] - state['timestamp'][0], state['v_x'],
            'b-', label='v_x (localized)', linewidth=1.5, alpha=0.7)
    ax2.plot(state['timestamp'] - state['timestamp'][0], state['v_y'],
            'g-', label='v_y (localized)', linewidth=1.5, alpha=0.7)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_xlabel('Elapsed Time (s)', fontsize=12)
    ax2.set_ylabel('Velocity Components (m/s)', fontsize=12)
    ax2.set_title('Velocity Components', fontsize=13)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best')

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved plot: {save_path}")
    else:
        plt.tight_layout()


def generate_diagnostic_report(run_dir: Path, output_dir: Path = None) -> None:
    """Generate complete diagnostic report with all plots.

    Args:
        run_dir: Path to the run directory containing CSV files
        output_dir: Optional directory to save plots (default: run_dir)
    """
    if output_dir is None:
        output_dir = run_dir

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"\nLoading data from: {run_dir}")
    data = load_run_data(run_dir)

    print("\nGenerating diagnostic plots...")

    # Plot 1: XY Trajectory
    plot_xy_trajectory(data, output_dir / "01_xy_trajectory.png")

    # Plot 2: Position Error
    plot_position_error(data, output_dir / "02_position_error.png")

    # Plot 3: Heading Comparison
    plot_heading_comparison(data, output_dir / "03_heading_comparison.png")

    # Plot 4: Velocity Comparison
    plot_velocity_comparison(data, output_dir / "04_velocity_comparison.png")

    print(f"\n✓ Diagnostic report generated in: {output_dir}")
    print("\nGenerated plots:")
    print("  1. 01_xy_trajectory.png - Spatial tracking")
    print("  2. 02_position_error.png - Position errors over time")
    print("  3. 03_heading_comparison.png - Heading tracking and error")
    print("  4. 04_velocity_comparison.png - Velocity estimates")


def main():
    """Command-line interface for diagnostic plots."""
    parser = argparse.ArgumentParser(
        description="Generate diagnostic plots for wagon path following analysis"
    )
    parser.add_argument(
        "run_dir",
        type=str,
        help="Path to run directory containing CSV files (e.g., results/run_20251115_134727)"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Output directory for plots (default: same as run_dir)"
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display plots interactively instead of just saving"
    )

    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    if not run_dir.exists():
        print(f"Error: Run directory not found: {run_dir}")
        return 1

    output_dir = Path(args.output_dir) if args.output_dir else run_dir

    generate_diagnostic_report(run_dir, output_dir)

    if args.show:
        print("\nDisplaying plots...")
        plt.show()

    return 0


if __name__ == "__main__":
    exit(main())
