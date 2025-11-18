"""
Visualization utilities for wagon sensor data.

This module provides functions to load and visualize IMU (accelerometer, gyroscope)
and GPS sensor data collected from the wagon control system.
"""

from pathlib import Path
from typing import Dict, Optional

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure

from . import path
from .plot_styles import (
    MONUMENTAL_BLUE,
    MONUMENTAL_CMAP,
    MONUMENTAL_CREAM,
    MONUMENTAL_DARK_BLUE,
    # Color scheme
    MONUMENTAL_ORANGE,
    MONUMENTAL_YELLOW_ORANGE,
    # CSV loading utilities
    load_csv_data,
    parse_timestamp,
)


def parse_imu_data(filepath: Path) -> Dict[str, np.ndarray]:
    """Parse IMU CSV data into numpy arrays.

    Args:
        filepath: Path to the IMU CSV file.

    Returns:
        Dictionary with keys: 'timestamp', 'x_dot', 'y_dot', 'theta_dot'.
        Each value is a numpy array of floats.

    Raises:
        FileNotFoundError: If the CSV file does not exist.
        ValueError: If CSV format is invalid.
    """
    headers, rows = load_csv_data(filepath)

    # Expected headers: timestamp, x_dot, y_dot, theta_dot
    if headers != ["timestamp", "x_dot", "y_dot", "theta_dot"]:
        raise ValueError(f"Unexpected IMU CSV headers: {headers}")

    timestamps = []
    x_dots = []
    y_dots = []
    theta_dots = []

    for row in rows:
        if len(row) != 4:
            continue

        try:
            timestamps.append(parse_timestamp(row[0]) if row[0] else np.nan)
            x_dots.append(float(row[1]) if row[1] else np.nan)
            y_dots.append(float(row[2]) if row[2] else np.nan)
            theta_dots.append(float(row[3]) if row[3] else np.nan)
        except (ValueError, AttributeError):
            # Skip rows with invalid data
            continue

    return {
        "timestamp": np.array(timestamps),
        "x_dot": np.array(x_dots),
        "y_dot": np.array(y_dots),
        "theta_dot": np.array(theta_dots),
    }


def parse_gps_data(filepath: Path) -> Dict[str, np.ndarray]:
    """Parse GPS CSV data into numpy arrays.

    Args:
        filepath: Path to the GPS CSV file.

    Returns:
        Dictionary with keys: 'timestamp', 'x', 'y'.
        Each value is a numpy array of floats.

    Raises:
        FileNotFoundError: If the CSV file does not exist.
        ValueError: If CSV format is invalid.
    """
    headers, rows = load_csv_data(filepath)

    # Expected headers: timestamp, x, y
    if headers != ["timestamp", "x", "y"]:
        raise ValueError(f"Unexpected GPS CSV headers: {headers}")

    timestamps = []
    xs = []
    ys = []

    for row in rows:
        if len(row) != 3:
            continue

        try:
            timestamps.append(parse_timestamp(row[0]) if row[0] else np.nan)
            xs.append(float(row[1]) if row[1] else np.nan)
            ys.append(float(row[2]) if row[2] else np.nan)
        except (ValueError, AttributeError):
            # Skip rows with invalid data
            continue

    return {"timestamp": np.array(timestamps), "x": np.array(xs), "y": np.array(ys)}


def plot_gps_trajectory(
    gps_data: Dict[str, np.ndarray],
    title: str = "GPS Trajectory",
    save_path: Optional[Path] = None,
    show_reference: bool = True,
) -> Figure:
    """Plot GPS trajectory (x vs y position).

    Args:
        gps_data: Dictionary containing 'x', 'y', and 'timestamp' arrays.
        title: Plot title.
        save_path: Optional path to save the figure.
        show_reference: If True, overlay the reference path on the plot.

    Returns:
        Matplotlib figure object.
    """
    fig, ax = plt.subplots(figsize=(10, 8), facecolor=MONUMENTAL_DARK_BLUE)
    ax.set_facecolor(MONUMENTAL_DARK_BLUE)

    x = gps_data["x"]
    y = gps_data["y"]
    timestamps = gps_data["timestamp"]

    # Remove NaN values
    valid_mask = ~(np.isnan(x) | np.isnan(y))
    x = x[valid_mask]
    y = y[valid_mask]
    timestamps = timestamps[valid_mask]

    if len(timestamps) > 0:
        # Plot trajectory line showing the path - Monumental orange
        ax.plot(
            x,
            y,
            "-",
            color=MONUMENTAL_ORANGE,
            linewidth=1.5,
            alpha=0.6,
            label="Trajectory",
            zorder=1,
        )

        # Create scatter plot with color gradient based on time - Monumental colormap
        scatter = ax.scatter(
            x,
            y,
            c=timestamps,
            cmap=MONUMENTAL_CMAP,
            s=20,
            alpha=0.8,
            edgecolors="black",
            linewidths=0.5,
            zorder=3,
        )
        plt.colorbar(scatter, ax=ax, label="Time (s)")

        # Mark start and end points - Monumental colors
        ax.plot(
            x[0],
            y[0],
            "o",
            color=MONUMENTAL_BLUE,
            markersize=8,
            label="Start",
            zorder=5,
            markeredgecolor="black",
            markeredgewidth=1.0,
        )
        ax.plot(
            x[-1],
            y[-1],
            "o",
            color=MONUMENTAL_ORANGE,
            markersize=8,
            label="End",
            zorder=5,
            markeredgecolor="black",
            markeredgewidth=1.0,
        )

    # Plot reference path if requested
    if show_reference:
        ref_path = path.path_trajectory(t_max=20.0, dt=0.1)
        ax.plot(
            ref_path["x"],
            ref_path["y"],
            "--",
            color=MONUMENTAL_YELLOW_ORANGE,
            linewidth=2.0,
            alpha=0.9,
            label="Reference Path",
            zorder=2,
        )

    ax.set_xlabel("X Position (m)", color=MONUMENTAL_CREAM)
    ax.set_ylabel("Y Position (m)", color=MONUMENTAL_CREAM)
    ax.set_title(title, color=MONUMENTAL_CREAM)
    ax.grid(True, alpha=0.2, color=MONUMENTAL_CREAM)
    ax.set_xlim(-2, 2)
    ax.set_ylim(-0.5, 4.5)
    ax.set_aspect("equal")

    # Dark mode styling
    ax.spines["bottom"].set_color(MONUMENTAL_CREAM)
    ax.spines["top"].set_color(MONUMENTAL_CREAM)
    ax.spines["left"].set_color(MONUMENTAL_CREAM)
    ax.spines["right"].set_color(MONUMENTAL_CREAM)
    ax.tick_params(colors=MONUMENTAL_CREAM, which="both")

    legend = ax.legend(facecolor=MONUMENTAL_DARK_BLUE, edgecolor=MONUMENTAL_CREAM)
    plt.setp(legend.get_texts(), color=MONUMENTAL_CREAM)

    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")

    return fig


def plot_imu_data(
    imu_data: Dict[str, np.ndarray], title: str = "IMU Data", save_path: Optional[Path] = None
) -> Figure:
    """Plot IMU data (accelerometer and gyroscope over time).

    Args:
        imu_data: Dictionary containing 'timestamp', 'x_dot', 'y_dot', 'theta_dot' arrays.
        title: Plot title prefix.
        save_path: Optional path to save the figure.

    Returns:
        Matplotlib figure object.
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), facecolor=MONUMENTAL_DARK_BLUE)

    # Apply dark mode styling to both axes
    for ax in [ax1, ax2]:
        ax.set_facecolor(MONUMENTAL_DARK_BLUE)
        ax.spines["bottom"].set_color(MONUMENTAL_CREAM)
        ax.spines["top"].set_color(MONUMENTAL_CREAM)
        ax.spines["left"].set_color(MONUMENTAL_CREAM)
        ax.spines["right"].set_color(MONUMENTAL_CREAM)
        ax.tick_params(colors=MONUMENTAL_CREAM, which="both")
        ax.xaxis.label.set_color(MONUMENTAL_CREAM)
        ax.yaxis.label.set_color(MONUMENTAL_CREAM)
        ax.title.set_color(MONUMENTAL_CREAM)

    timestamps = imu_data["timestamp"]
    x_dot = imu_data["x_dot"]
    y_dot = imu_data["y_dot"]
    theta_dot = imu_data["theta_dot"]

    # Remove NaN timestamps
    valid_mask = ~np.isnan(timestamps)
    timestamps = timestamps[valid_mask]
    x_dot = x_dot[valid_mask]
    y_dot = y_dot[valid_mask]
    theta_dot = theta_dot[valid_mask]

    # Normalize timestamps to start from 0
    if len(timestamps) > 0:
        timestamps = timestamps - timestamps[0]

    # Plot accelerometer data - Monumental colors
    ax1.plot(timestamps, x_dot, label="X", alpha=0.7, color=MONUMENTAL_ORANGE)
    ax1.plot(timestamps, y_dot, label="Y", alpha=0.7, color=MONUMENTAL_BLUE)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Acceleration (m/s²)")
    ax1.set_title(f"{title} - Accelerometer")
    ax1.grid(True, alpha=0.2, color=MONUMENTAL_CREAM)
    legend1 = ax1.legend(facecolor=MONUMENTAL_DARK_BLUE, edgecolor=MONUMENTAL_CREAM)
    plt.setp(legend1.get_texts(), color=MONUMENTAL_CREAM)

    # Plot gyroscope data - Monumental orange
    ax2.plot(timestamps, theta_dot, label="Angular Velocity", alpha=0.7, color=MONUMENTAL_ORANGE)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Angular Velocity (rad/s)")
    ax2.set_title(f"{title} - Gyroscope")
    ax2.grid(True, alpha=0.2, color=MONUMENTAL_CREAM)
    legend2 = ax2.legend(facecolor=MONUMENTAL_DARK_BLUE, edgecolor=MONUMENTAL_CREAM)
    plt.setp(legend2.get_texts(), color=MONUMENTAL_CREAM)

    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")

    return fig


def plot_run_summary(run_dir: Path, save_plots: bool = False, show_plots: bool = True) -> None:
    """Generate summary plots for a complete run.

    Args:
        run_dir: Directory containing imu_data.csv and gps_data.csv.
        save_plots: If True, save plots to run directory.
        show_plots: If True, display plots interactively.

    Raises:
        FileNotFoundError: If required CSV files are not found.
    """
    imu_path = run_dir / "imu_data.csv"

    # Load data
    imu_data = parse_imu_data(imu_path)

    run_name = run_dir.name

    # Note: IMU plots have been removed - not needed
    # GPS trajectory, position error, heading comparison, and velocity plots
    # are now consolidated into live_plot_snapshot.png for cleaner output

    # Output is handled by the calling script for consistent formatting
