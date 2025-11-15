"""
Real-time visualization for live sensor data streaming.

This module provides live plotting capabilities that update as data is being
collected from the wagon control system.
"""

import logging
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.colors import LinearSegmentedColormap

from . import path
from .visualization import parse_gps_data, parse_imu_data

# Monumental brand colors (hex for plots)
MONUMENTAL_ORANGE = "#f74823"
MONUMENTAL_BLUE = "#2374f7"
MONUMENTAL_CREAM = "#fffdee"
MONUMENTAL_TAUPE = "#686a5f"
MONUMENTAL_YELLOW_ORANGE = "#ffa726"  # Yellow-orange (Atrium style)
MONUMENTAL_DARK_BLUE = "#0d1b2a"  # Dark blue for background

# Terminal color codes
TERM_ORANGE = "\033[38;2;247;72;35m"  # Monumental orange
TERM_BLUE = "\033[38;2;35;116;247m"  # Complementary blue
TERM_RESET = "\033[0m"  # Reset color

# Create custom colormap: orange -> blue
MONUMENTAL_CMAP = LinearSegmentedColormap.from_list(
    "monumental", [MONUMENTAL_ORANGE, MONUMENTAL_BLUE]
)


class LivePlotter:
    """Real-time plotter that updates as CSV data files are written.

    Monitors GPS and IMU CSV files and updates plots in real-time as new
    data becomes available.

    Attributes:
        run_dir: Directory containing the CSV files to monitor.
        gps_path: Path to GPS CSV file.
        imu_path: Path to IMU CSV file.
        update_interval: Update interval in milliseconds.
        fig: Matplotlib figure containing all subplots.
    """

    def __init__(self, run_dir: Path, update_interval: int = 500) -> None:
        """Initialize the live plotter.

        Args:
            run_dir: Directory containing imu_data.csv and gps_data.csv.
            update_interval: Plot update interval in milliseconds (default: 500ms).
        """
        self.run_dir = run_dir
        self.gps_path = run_dir / "gps_data.csv"
        self.imu_path = run_dir / "imu_data.csv"
        self.update_interval = update_interval

        # Create figure with subplots - dark mode
        self.fig = plt.figure(figsize=(14, 10), facecolor=MONUMENTAL_DARK_BLUE)
        self.fig.suptitle(
            f"Live Data - {run_dir.name}", fontsize=14, fontweight="bold", color=MONUMENTAL_CREAM
        )

        # Create subplot grid: 2 rows, 2 columns
        # Top row: GPS trajectory (spans both columns)
        # Bottom row: Accelerometer (left), Gyroscope (right)
        self.ax_gps = plt.subplot2grid((2, 2), (0, 0), colspan=2)
        self.ax_accel = plt.subplot2grid((2, 2), (1, 0))
        self.ax_gyro = plt.subplot2grid((2, 2), (1, 1))

        # Initialize line objects - Monumental colors
        (self.gps_line,) = self.ax_gps.plot(
            [], [], "-", color=MONUMENTAL_ORANGE, linewidth=1.5, alpha=0.6, label="Trajectory"
        )
        self.gps_scatter = self.ax_gps.scatter(
            [], [], c=[], cmap=MONUMENTAL_CMAP, s=20, alpha=0.8, edgecolors="black", linewidths=0.5
        )
        (self.gps_start,) = self.ax_gps.plot(
            [],
            [],
            "o",
            color=MONUMENTAL_BLUE,
            markersize=8,
            label="Start",
            markeredgecolor="black",
            markeredgewidth=1.0,
        )
        (self.gps_current,) = self.ax_gps.plot(
            [],
            [],
            "o",
            color=MONUMENTAL_ORANGE,
            markersize=8,
            label="Current",
            markeredgecolor="black",
            markeredgewidth=1.0,
        )

        # Pre-compute and plot reference path
        ref_path = path.path_trajectory(t_max=20.0, dt=0.1)
        (self.gps_reference,) = self.ax_gps.plot(
            ref_path["x"],
            ref_path["y"],
            "--",
            color=MONUMENTAL_YELLOW_ORANGE,
            linewidth=2.0,
            alpha=0.9,
            label="Reference Path",
        )

        (self.accel_x_line,) = self.ax_accel.plot(
            [], [], label="X", alpha=0.7, color=MONUMENTAL_ORANGE
        )
        (self.accel_y_line,) = self.ax_accel.plot(
            [], [], label="Y", alpha=0.7, color=MONUMENTAL_BLUE
        )

        (self.gyro_line,) = self.ax_gyro.plot(
            [], [], label="Angular Velocity", alpha=0.7, color=MONUMENTAL_ORANGE
        )

        # Setup axes
        self._setup_axes()

        # Initialize data tracking
        self.last_gps_size = 0
        self.last_imu_size = 0
        self.start_time: Optional[float] = None

    def _setup_axes(self) -> None:
        """Configure plot axes with labels and styling - dark mode."""
        # Apply dark mode styling to all axes
        for ax in [self.ax_gps, self.ax_accel, self.ax_gyro]:
            ax.set_facecolor(MONUMENTAL_DARK_BLUE)
            ax.spines["bottom"].set_color(MONUMENTAL_CREAM)
            ax.spines["top"].set_color(MONUMENTAL_CREAM)
            ax.spines["left"].set_color(MONUMENTAL_CREAM)
            ax.spines["right"].set_color(MONUMENTAL_CREAM)
            ax.tick_params(colors=MONUMENTAL_CREAM, which="both")
            ax.xaxis.label.set_color(MONUMENTAL_CREAM)
            ax.yaxis.label.set_color(MONUMENTAL_CREAM)
            ax.title.set_color(MONUMENTAL_CREAM)

        # GPS plot - viewing area adjusted for reference path
        self.ax_gps.set_xlabel("X Position (m)", fontsize=10)
        self.ax_gps.set_ylabel("Y Position (m)", fontsize=10)
        self.ax_gps.set_title("GPS Trajectory", fontsize=11, fontweight="bold")
        self.ax_gps.grid(True, alpha=0.2, color=MONUMENTAL_CREAM)
        self.ax_gps.set_xlim(-2, 2)
        self.ax_gps.set_ylim(-0.5, 4.5)
        self.ax_gps.set_aspect("equal")
        legend = self.ax_gps.legend(
            loc="upper right", facecolor=MONUMENTAL_DARK_BLUE, edgecolor=MONUMENTAL_CREAM
        )
        plt.setp(legend.get_texts(), color=MONUMENTAL_CREAM)

        # Accelerometer plot
        self.ax_accel.set_xlabel("Time (s)", fontsize=10)
        self.ax_accel.set_ylabel("Acceleration (m/s²)", fontsize=10)
        self.ax_accel.set_title("Accelerometer", fontsize=11, fontweight="bold")
        self.ax_accel.grid(True, alpha=0.2, color=MONUMENTAL_CREAM)
        legend = self.ax_accel.legend(
            loc="upper right", facecolor=MONUMENTAL_DARK_BLUE, edgecolor=MONUMENTAL_CREAM
        )
        plt.setp(legend.get_texts(), color=MONUMENTAL_CREAM)

        # Gyroscope plot
        self.ax_gyro.set_xlabel("Time (s)", fontsize=10)
        self.ax_gyro.set_ylabel("Angular Velocity (rad/s)", fontsize=10)
        self.ax_gyro.set_title("Gyroscope", fontsize=11, fontweight="bold")
        self.ax_gyro.grid(True, alpha=0.2, color=MONUMENTAL_CREAM)
        legend = self.ax_gyro.legend(
            loc="upper right", facecolor=MONUMENTAL_DARK_BLUE, edgecolor=MONUMENTAL_CREAM
        )
        plt.setp(legend.get_texts(), color=MONUMENTAL_CREAM)

        plt.tight_layout()

    def _update(self, frame: int) -> tuple:
        """Update plots with new data from CSV files.

        Args:
            frame: Animation frame number (unused).

        Returns:
            Tuple of updated artists.
        """
        try:
            # Update GPS data
            if self.gps_path.exists():
                self._update_gps()

            # Update IMU data
            if self.imu_path.exists():
                self._update_imu()

        except Exception as e:
            logging.debug(f"Error updating plots: {e}")

        return (
            self.gps_line,
            self.gps_scatter,
            self.gps_start,
            self.gps_current,
            self.accel_x_line,
            self.accel_y_line,
            self.gyro_line,
        )

    def _update_gps(self) -> None:
        """Update GPS trajectory plot."""
        try:
            gps_data = parse_gps_data(self.gps_path)

            x = gps_data["x"]
            y = gps_data["y"]
            timestamps = gps_data["timestamp"]

            # Remove NaN values
            valid_mask = ~(np.isnan(x) | np.isnan(y) | np.isnan(timestamps))
            x = x[valid_mask]
            y = y[valid_mask]
            timestamps = timestamps[valid_mask]

            if len(x) > 0:
                # Update trajectory line
                self.gps_line.set_data(x, y)

                # Update scatter plot with time coloring
                self.gps_scatter.set_offsets(np.c_[x, y])
                self.gps_scatter.set_array(timestamps)

                # Update start and current position markers
                self.gps_start.set_data([x[0]], [y[0]])
                self.gps_current.set_data([x[-1]], [y[-1]])

        except Exception:
            pass  # Silently ignore parsing errors during live update

    def _update_imu(self) -> None:
        """Update IMU data plots."""
        try:
            imu_data = parse_imu_data(self.imu_path)

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

            if len(timestamps) > 0:
                # Normalize timestamps to start from 0
                if self.start_time is None:
                    self.start_time = timestamps[0]

                relative_times = timestamps - self.start_time

                # Update accelerometer plot
                self.accel_x_line.set_data(relative_times, x_dot)
                self.accel_y_line.set_data(relative_times, y_dot)

                # Update gyroscope plot
                self.gyro_line.set_data(relative_times, theta_dot)

                # Auto-scale axes
                self.ax_accel.relim()
                self.ax_accel.autoscale_view()
                self.ax_gyro.relim()
                self.ax_gyro.autoscale_view()

        except Exception:
            pass  # Silently ignore parsing errors during live update

    def start(self) -> None:
        """Start the live plotting animation.

        Blocks until the plot window is closed.
        """
        # Create animation
        anim = FuncAnimation(
            self.fig,
            self._update,
            interval=self.update_interval,
            blit=False,
            cache_frame_data=False,
        )

        # Show plot
        plt.show()


def start_live_plot(run_dir: Path, update_interval: int = 500) -> None:
    """Start a live plotting session for a run directory.

    Args:
        run_dir: Directory containing CSV files to monitor.
        update_interval: Update interval in milliseconds (default: 500ms).
    """
    plotter = LivePlotter(run_dir, update_interval)
    plotter.start()


if __name__ == "__main__":
    import sys

    # Setup basic logging for standalone usage
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    if len(sys.argv) < 2:
        logging.error("Usage: python -m wagon_control.live_plot <run_directory>")
        sys.exit(1)

    run_dir = Path(sys.argv[1])
    if not run_dir.exists():
        logging.error(f"Error: Directory not found: {run_dir}")
        sys.exit(1)

    logging.info(f"{TERM_BLUE}✓ Starting live plot for: results/{run_dir.name}{TERM_RESET}")
    start_live_plot(run_dir)
