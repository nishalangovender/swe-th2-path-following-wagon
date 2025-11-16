"""
Real-time visualization for live sensor data streaming.

This module provides live plotting capabilities that update as data is being
collected from the wagon control system.
"""

import csv
import logging
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from . import path
from .config import (
    TERM_BLUE,
    TERM_RESET,
)
from .plot_styles import (
    MONUMENTAL_BLUE,
    MONUMENTAL_CREAM,
    MONUMENTAL_DARK_BLUE,
    MONUMENTAL_ORANGE,
    MONUMENTAL_TAUPE,
    MONUMENTAL_YELLOW_ORANGE,
)
from .visualization import parse_gps_data, parse_imu_data


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
        self.state_path = run_dir / "state_data.csv"
        self.reference_path = run_dir / "reference_data.csv"
        self.motor_path = run_dir / "motor_data.csv"
        self.update_interval = update_interval

        # Create figure with subplots - dark mode (improved 2-row layout)
        self.fig = plt.figure(figsize=(18, 10), facecolor=MONUMENTAL_DARK_BLUE)
        self.fig.suptitle(
            f"Live Data - {run_dir.name}",
            fontsize=14,
            fontweight="bold",
            color=MONUMENTAL_CREAM,
            y=0.98,
        )

        # Create subplot grid: 2 rows, 3 columns
        # Row 0: GPS trajectory (rows 0-1, col 0), Velocity (col 1), Angular Velocity (col 2)
        # Row 1: GPS continues, Position X/Y Errors (col 1), Heading Error (col 2)
        self.ax_gps = plt.subplot2grid((2, 3), (0, 0), rowspan=2)
        self.ax_velocity = plt.subplot2grid((2, 3), (0, 1))
        self.ax_angular_velocity = plt.subplot2grid((2, 3), (0, 2))
        self.ax_position_errors = plt.subplot2grid((2, 3), (1, 1))
        self.ax_heading_error = plt.subplot2grid((2, 3), (1, 2))

        # Initialize line objects - Trajectory comparison (REF vs GPS vs LOC)

        # Static reference path (background guide, faint)
        ref_path = path.path_trajectory(t_max=20.0, dt=0.1)
        (self.static_reference,) = self.ax_gps.plot(
            ref_path["x"],
            ref_path["y"],
            ":",
            color=MONUMENTAL_TAUPE,
            linewidth=1.0,
            alpha=0.3,
            label="Static Guide",
        )

        # Time-based reference trajectory (REF - what controller should follow)
        (self.ref_line,) = self.ax_gps.plot(
            [], [], "-", color=MONUMENTAL_BLUE, linewidth=2.0, alpha=0.8, label="REF (planned)"
        )

        # GPS trajectory (sparse 1 Hz measurements)
        (self.gps_line,) = self.ax_gps.plot(
            [],
            [],
            "o-",
            color=MONUMENTAL_ORANGE,
            linewidth=1.0,
            alpha=0.6,
            markersize=4,
            label="GPS (1 Hz)",
        )

        # Localized trajectory (dense 20 Hz estimated path)
        (self.loc_line,) = self.ax_gps.plot(
            [], [], "-", color="#00ff00", linewidth=1.5, alpha=0.7, label="LOC (odom)"
        )

        # Start marker
        (self.traj_start,) = self.ax_gps.plot(
            [],
            [],
            "o",
            color=MONUMENTAL_CREAM,
            markersize=10,
            label="Start",
            markeredgecolor="black",
            markeredgewidth=1.5,
            zorder=10,
        )

        # Diagnostic plot line objects
        # Heading error plot (primary axis - error in degrees)
        (self.heading_error_line,) = self.ax_heading_error.plot(
            [], [], color=MONUMENTAL_ORANGE, linewidth=2, alpha=0.8, label="Error"
        )
        (self.heading_error_mean,) = self.ax_heading_error.plot(
            [], [], "--", color=MONUMENTAL_ORANGE, linewidth=1.5, alpha=0.5, label="Mean Error"
        )

        # Create twin axis for absolute heading values
        self.ax_heading_values = self.ax_heading_error.twinx()
        (self.heading_ref_line,) = self.ax_heading_values.plot(
            [], [], color=MONUMENTAL_BLUE, linewidth=1.5, alpha=0.7, label="REF"
        )
        (self.heading_loc_line,) = self.ax_heading_values.plot(
            [], [], color=MONUMENTAL_YELLOW_ORANGE, linewidth=1.5, alpha=0.7, label="LOC"
        )

        (self.position_error_x_line,) = self.ax_position_errors.plot(
            [], [], color=MONUMENTAL_ORANGE, linewidth=1.5, alpha=0.8, label="X Error"
        )
        (self.position_error_y_line,) = self.ax_position_errors.plot(
            [], [], color=MONUMENTAL_BLUE, linewidth=1.5, alpha=0.8, label="Y Error"
        )

        (self.velocity_ref_line,) = self.ax_velocity.plot(
            [], [], color=MONUMENTAL_BLUE, linewidth=2, alpha=0.7, label="REF (path follower)"
        )
        (self.velocity_cmd_line,) = self.ax_velocity.plot(
            [], [], color=MONUMENTAL_YELLOW_ORANGE, linewidth=2, alpha=0.8, label="CMD (motor ctrl)"
        )
        (self.velocity_loc_line,) = self.ax_velocity.plot(
            [], [], color=MONUMENTAL_ORANGE, linewidth=1.5, alpha=0.7, label="LOC (actual)"
        )

        (self.angular_velocity_ref_line,) = self.ax_angular_velocity.plot(
            [], [], color=MONUMENTAL_BLUE, linewidth=2, alpha=0.7, label="REF (path follower)"
        )
        (self.angular_velocity_cmd_line,) = self.ax_angular_velocity.plot(
            [], [], color=MONUMENTAL_YELLOW_ORANGE, linewidth=2, alpha=0.8, label="CMD (motor ctrl)"
        )
        (self.angular_velocity_loc_line,) = self.ax_angular_velocity.plot(
            [], [], color=MONUMENTAL_ORANGE, linewidth=1.5, alpha=0.7, label="LOC (actual)"
        )

        # Setup axes
        self._setup_axes()

        # Initialize data tracking
        self.last_gps_size = 0
        self.last_imu_size = 0
        self.start_time: Optional[float] = None

    def _setup_axes(self) -> None:
        """Configure plot axes with labels and styling - dark mode."""
        # Apply dark mode styling to all axes (including twin axis)
        all_axes = [
            self.ax_gps,
            self.ax_velocity,
            self.ax_angular_velocity,
            self.ax_heading_error,
            self.ax_heading_values,
            self.ax_position_errors,
        ]
        for ax in all_axes:
            ax.set_facecolor(MONUMENTAL_DARK_BLUE)
            ax.spines["bottom"].set_color(MONUMENTAL_CREAM)
            ax.spines["top"].set_color(MONUMENTAL_CREAM)
            ax.spines["left"].set_color(MONUMENTAL_CREAM)
            ax.spines["right"].set_color(MONUMENTAL_CREAM)
            ax.tick_params(colors=MONUMENTAL_CREAM, which="both")
            ax.xaxis.label.set_color(MONUMENTAL_CREAM)
            ax.yaxis.label.set_color(MONUMENTAL_CREAM)
            ax.title.set_color(MONUMENTAL_CREAM)

        # Trajectory comparison plot - REF vs GPS vs LOC
        self.ax_gps.set_xlabel("X Position (m)", fontsize=10)
        self.ax_gps.set_ylabel("Y Position (m)", fontsize=10)
        self.ax_gps.set_title("Trajectory: REF vs GPS vs LOC", fontsize=11, fontweight="bold")
        self.ax_gps.grid(True, alpha=0.2, color=MONUMENTAL_CREAM)
        self.ax_gps.set_xlim(-2, 2)
        self.ax_gps.set_ylim(-0.5, 4.5)
        self.ax_gps.set_aspect("equal")
        legend = self.ax_gps.legend(
            loc="upper right",
            fontsize=8,
            facecolor=MONUMENTAL_DARK_BLUE,
            edgecolor=MONUMENTAL_CREAM,
            framealpha=0.9,
        )
        plt.setp(legend.get_texts(), color=MONUMENTAL_CREAM)

        # Velocity plot
        self.ax_velocity.set_xlabel("Time (s)", fontsize=9)
        self.ax_velocity.set_ylabel("Velocity (m/s)", fontsize=10)
        self.ax_velocity.set_title(
            "Linear Velocity: v (Control Loop)", fontsize=10, fontweight="bold"
        )
        self.ax_velocity.grid(True, alpha=0.2, color=MONUMENTAL_CREAM)
        legend = self.ax_velocity.legend(
            loc="upper right",
            fontsize=8,
            facecolor=MONUMENTAL_DARK_BLUE,
            edgecolor=MONUMENTAL_CREAM,
        )
        plt.setp(legend.get_texts(), color=MONUMENTAL_CREAM)

        # Angular velocity plot
        self.ax_angular_velocity.set_xlabel("Time (s)", fontsize=9)
        self.ax_angular_velocity.set_ylabel("Angular Velocity (rad/s)", fontsize=10)
        self.ax_angular_velocity.set_title(
            "Angular Velocity: ω (Control Loop)", fontsize=10, fontweight="bold"
        )
        self.ax_angular_velocity.grid(True, alpha=0.2, color=MONUMENTAL_CREAM)
        legend = self.ax_angular_velocity.legend(
            loc="upper right",
            fontsize=8,
            facecolor=MONUMENTAL_DARK_BLUE,
            edgecolor=MONUMENTAL_CREAM,
        )
        plt.setp(legend.get_texts(), color=MONUMENTAL_CREAM)

        # Heading Error plot (dual axis: error + absolute values)
        self.ax_heading_error.set_xlabel("Time (s)", fontsize=9)
        self.ax_heading_error.set_ylabel("Error (°)", fontsize=10, color=MONUMENTAL_ORANGE)
        self.ax_heading_error.set_title(
            "Heading: θ (REF vs LOC + Error)", fontsize=10, fontweight="bold"
        )
        self.ax_heading_error.grid(True, alpha=0.2, color=MONUMENTAL_CREAM)
        self.ax_heading_error.axhline(y=0, color=MONUMENTAL_CREAM, linestyle="--", alpha=0.3)
        self.ax_heading_error.tick_params(axis="y", labelcolor=MONUMENTAL_ORANGE)

        # Configure twin axis for absolute heading values
        self.ax_heading_values.set_ylabel("Heading (°)", fontsize=10, color=MONUMENTAL_BLUE)
        self.ax_heading_values.spines["right"].set_color(MONUMENTAL_CREAM)
        self.ax_heading_values.tick_params(axis="y", colors=MONUMENTAL_BLUE)

        # Combined legend for both axes
        lines_error = [self.heading_error_line, self.heading_error_mean]
        lines_values = [self.heading_ref_line, self.heading_loc_line]
        labels_error = [line.get_label() for line in lines_error]
        labels_values = [line.get_label() for line in lines_values]
        legend = self.ax_heading_error.legend(
            lines_error + lines_values,
            labels_error + labels_values,
            loc="upper right",
            fontsize=8,
            facecolor=MONUMENTAL_DARK_BLUE,
            edgecolor=MONUMENTAL_CREAM,
            framealpha=0.9,
        )
        plt.setp(legend.get_texts(), color=MONUMENTAL_CREAM)

        # Position X/Y Errors plot
        self.ax_position_errors.set_xlabel("Time (s)", fontsize=9)
        self.ax_position_errors.set_ylabel("Position Error (m)", fontsize=10)
        self.ax_position_errors.set_title("Position Errors (X/Y)", fontsize=10, fontweight="bold")
        self.ax_position_errors.grid(True, alpha=0.2, color=MONUMENTAL_CREAM)
        self.ax_position_errors.axhline(y=0, color=MONUMENTAL_CREAM, linestyle="--", alpha=0.3)
        legend = self.ax_position_errors.legend(
            loc="upper right", facecolor=MONUMENTAL_DARK_BLUE, edgecolor=MONUMENTAL_CREAM
        )
        plt.setp(legend.get_texts(), color=MONUMENTAL_CREAM)

        plt.tight_layout(rect=[0, 0, 1, 0.96], h_pad=3.0, w_pad=2.5)

    def _load_state_data(self) -> Optional[Dict[str, np.ndarray]]:
        """Load state data from CSV file.

        Returns:
            Dictionary with keys: 'timestamp', 'x_loc', 'y_loc', 'theta_loc', 'v_x', 'v_y'
            or None if file doesn't exist or can't be parsed.
        """
        try:
            if not self.state_path.exists():
                return None

            with open(self.state_path) as f:
                reader = csv.DictReader(f)
                data: Dict[str, List[float]] = {
                    "timestamp": [],
                    "x_loc": [],
                    "y_loc": [],
                    "theta_loc": [],
                    "v_x": [],
                    "v_y": [],
                }
                for row in reader:
                    try:
                        data["timestamp"].append(float(row["timestamp"]))
                        data["x_loc"].append(float(row["x_loc"]))
                        data["y_loc"].append(float(row["y_loc"]))
                        data["theta_loc"].append(float(row["theta_loc"]))
                        data["v_x"].append(float(row["v_x"]))
                        data["v_y"].append(float(row["v_y"]))
                    except (ValueError, KeyError):
                        continue

            return {key: np.array(values) for key, values in data.items()}
        except Exception:
            return None

    def _load_reference_data(self) -> Optional[Dict[str, np.ndarray]]:
        """Load reference data from CSV file.

        Returns:
            Dictionary with keys: 'timestamp', 'elapsed_time', 'x_ref', 'y_ref', 'theta_ref'
            or None if file doesn't exist or can't be parsed.
        """
        try:
            if not self.reference_path.exists():
                return None

            with open(self.reference_path) as f:
                reader = csv.DictReader(f)
                data: Dict[str, List[float]] = {
                    "timestamp": [],
                    "elapsed_time": [],
                    "x_ref": [],
                    "y_ref": [],
                    "theta_ref": [],
                }
                for row in reader:
                    try:
                        data["timestamp"].append(float(row["timestamp"]))
                        data["elapsed_time"].append(float(row["elapsed_time"]))
                        data["x_ref"].append(float(row["x_ref"]))
                        data["y_ref"].append(float(row["y_ref"]))
                        data["theta_ref"].append(float(row["theta_ref"]))
                    except (ValueError, KeyError):
                        continue

            return {key: np.array(values) for key, values in data.items()}
        except Exception:
            return None

    def _load_motor_data(self) -> Optional[Dict[str, np.ndarray]]:
        """Load motor controller data from CSV file.

        Returns:
            Dictionary with keys: 'timestamp', 'v_ref', 'omega_ref', 'v_loc', 'omega_loc',
            'v_err', 'omega_err', 'v_cmd', 'omega_cmd'
            or None if file doesn't exist or can't be parsed.
        """
        try:
            if not self.motor_path.exists():
                return None

            with open(self.motor_path) as f:
                reader = csv.DictReader(f)
                data: Dict[str, List[float]] = {
                    "timestamp": [],
                    "v_ref": [],
                    "omega_ref": [],
                    "v_loc": [],
                    "omega_loc": [],
                    "v_err": [],
                    "omega_err": [],
                    "v_cmd": [],
                    "omega_cmd": [],
                }
                for row in reader:
                    try:
                        data["timestamp"].append(float(row["timestamp"]))
                        data["v_ref"].append(float(row["v_ref"]))
                        data["omega_ref"].append(float(row["omega_ref"]))
                        data["v_loc"].append(float(row["v_loc"]))
                        data["omega_loc"].append(float(row["omega_loc"]))
                        data["v_err"].append(float(row["v_err"]))
                        data["omega_err"].append(float(row["omega_err"]))
                        data["v_cmd"].append(float(row["v_cmd"]))
                        data["omega_cmd"].append(float(row["omega_cmd"]))
                    except (ValueError, KeyError):
                        continue

            return {key: np.array(values) for key, values in data.items()}
        except Exception:
            return None

    def _update(self, frame: int) -> tuple:
        """Update plots with new data from CSV files.

        Args:
            frame: Animation frame number (unused).

        Returns:
            Tuple of updated artists.
        """
        try:
            # Update trajectory comparison (REF, GPS, LOC)
            if self.gps_path.exists() or self.state_path.exists() or self.reference_path.exists():
                self._update_trajectories()

            # Update diagnostic plots (state and reference data)
            if self.state_path.exists() and self.reference_path.exists():
                self._update_diagnostic_plots()

            # Update angular velocity plot
            if self.state_path.exists() and self.reference_path.exists() and self.imu_path.exists():
                self._update_angular_velocity()

        except Exception as e:
            logging.debug(f"Error updating plots: {e}")

        return (
            self.ref_line,
            self.gps_line,
            self.loc_line,
            self.traj_start,
            self.heading_error_line,
            self.heading_error_mean,
            self.heading_ref_line,
            self.heading_loc_line,
            self.position_error_x_line,
            self.position_error_y_line,
            self.velocity_ref_line,
            self.velocity_cmd_line,
            self.velocity_loc_line,
            self.angular_velocity_ref_line,
            self.angular_velocity_cmd_line,
            self.angular_velocity_loc_line,
        )

    def _update_trajectories(self) -> None:
        """Update trajectory comparison plot showing REF, GPS, and LOC paths."""
        try:
            # Load all trajectory data
            ref_data = self._load_reference_data()
            state_data = self._load_state_data()
            gps_data = None
            if self.gps_path.exists():
                gps_data = parse_gps_data(self.gps_path)

            # Update reference trajectory (REF - time-based planned path)
            if ref_data is not None and len(ref_data["x_ref"]) > 0:
                self.ref_line.set_data(ref_data["x_ref"], ref_data["y_ref"])

            # Update GPS trajectory (sparse 1 Hz measurements)
            if gps_data is not None:
                x = gps_data["x"]
                y = gps_data["y"]
                timestamps = gps_data["timestamp"]

                # Remove NaN values
                valid_mask = ~(np.isnan(x) | np.isnan(y) | np.isnan(timestamps))
                x = x[valid_mask]
                y = y[valid_mask]

                if len(x) > 0:
                    self.gps_line.set_data(x, y)

            # Update localized trajectory (LOC - dense 20 Hz odometry)
            if state_data is not None and len(state_data["x_loc"]) > 0:
                self.loc_line.set_data(state_data["x_loc"], state_data["y_loc"])

            # Update start marker (use reference if available, else GPS, else localized)
            if ref_data is not None and len(ref_data["x_ref"]) > 0:
                self.traj_start.set_data([ref_data["x_ref"][0]], [ref_data["y_ref"][0]])
            elif gps_data is not None and len(x) > 0:
                self.traj_start.set_data([x[0]], [y[0]])
            elif state_data is not None and len(state_data["x_loc"]) > 0:
                self.traj_start.set_data([state_data["x_loc"][0]], [state_data["y_loc"][0]])

        except Exception as e:
            logging.debug(f"Error updating trajectories: {e}")

    def _update_angular_velocity(self) -> None:
        """Update angular velocity comparison plot showing ω_ref vs ω_cmd vs ω_loc."""
        try:
            # Load reference, motor, and IMU data
            ref_data = self._load_reference_data()
            motor_data = self._load_motor_data()
            imu_data = parse_imu_data(self.imu_path)

            if ref_data is None:
                return

            # Compute reference angular velocity from heading derivative
            dt = np.diff(ref_data["elapsed_time"], prepend=0.0)
            dt[0] = 0.1  # Avoid division by zero for first element
            dtheta_dt = np.diff(ref_data["theta_ref"], prepend=ref_data["theta_ref"][0]) / dt
            # Normalize the angle difference to [-pi, pi] before computing derivative
            dtheta = np.diff(ref_data["theta_ref"], prepend=ref_data["theta_ref"][0])
            dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))
            omega_ref = dtheta / dt

            # Get localized angular velocity from IMU
            timestamps = imu_data["timestamp"]
            omega_loc = imu_data["theta_dot"]

            # Remove NaN timestamps
            valid_mask = ~np.isnan(timestamps)
            timestamps = timestamps[valid_mask]
            omega_loc = omega_loc[valid_mask]

            if len(timestamps) == 0:
                return

            # Normalize timestamps to match reference elapsed time
            if self.start_time is None and len(timestamps) > 0:
                self.start_time = timestamps[0]

            relative_times_imu = timestamps - self.start_time
            relative_times_ref = ref_data["elapsed_time"]

            # Align data lengths for plotting (use minimum length)
            min_len = min(len(relative_times_ref), len(relative_times_imu))
            if min_len == 0:
                return

            # Update angular velocity plot
            self.angular_velocity_ref_line.set_data(
                relative_times_ref[:min_len], omega_ref[:min_len]
            )
            self.angular_velocity_loc_line.set_data(
                relative_times_imu[:min_len], omega_loc[:min_len]
            )

            # Add motor command data if available
            if motor_data is not None and len(motor_data["omega_cmd"]) > 0:
                min_motor_len = min(len(motor_data["omega_cmd"]), min_len)
                self.angular_velocity_cmd_line.set_data(
                    relative_times_ref[:min_motor_len], motor_data["omega_cmd"][:min_motor_len]
                )

            # Auto-scale axes
            self.ax_angular_velocity.relim()
            self.ax_angular_velocity.autoscale_view()

        except Exception as e:
            logging.debug(f"Error updating angular velocity: {e}")

    def _update_diagnostic_plots(self) -> None:
        """Update all diagnostic plots with state, reference, and motor data."""
        try:
            # Load state, reference, and motor data
            state_data = self._load_state_data()
            ref_data = self._load_reference_data()
            motor_data = self._load_motor_data()

            if state_data is None or ref_data is None:
                return

            # Ensure arrays have same length (minimum of both)
            min_len = min(len(state_data["timestamp"]), len(ref_data["timestamp"]))
            if min_len == 0:
                return

            # Truncate to same length
            for key in state_data:
                state_data[key] = state_data[key][:min_len]
            for key in ref_data:
                ref_data[key] = ref_data[key][:min_len]

            # Compute errors
            error_x = ref_data["x_ref"] - state_data["x_loc"]
            error_y = ref_data["y_ref"] - state_data["y_loc"]
            error_l2 = np.sqrt(error_x**2 + error_y**2)

            # Unwrap headings before computing error to avoid discontinuity spikes
            theta_ref_unwrapped = np.unwrap(ref_data["theta_ref"])
            theta_loc_unwrapped = np.unwrap(state_data["theta_loc"])
            error_theta = theta_ref_unwrapped - theta_loc_unwrapped

            # Normalize theta error to [-pi, pi] for meaningful statistics
            error_theta = np.arctan2(np.sin(error_theta), np.cos(error_theta))
            error_theta_deg = np.degrees(error_theta)

            # Compute velocity magnitudes
            v_loc = np.sqrt(state_data["v_x"] ** 2 + state_data["v_y"] ** 2)
            # Approximate reference velocity from position derivatives
            dt = np.diff(ref_data["elapsed_time"], prepend=0.0)
            dt[0] = 0.1  # Avoid division by zero
            dx_dt = np.diff(ref_data["x_ref"], prepend=ref_data["x_ref"][0]) / dt
            dy_dt = np.diff(ref_data["y_ref"], prepend=ref_data["y_ref"][0]) / dt
            v_ref = np.sqrt(dx_dt**2 + dy_dt**2)

            # Normalize timestamps
            if self.start_time is None and len(ref_data["timestamp"]) > 0:
                self.start_time = ref_data["timestamp"][0]

            relative_times = ref_data["elapsed_time"]

            # Update Heading Error plot (primary axis)
            self.heading_error_line.set_data(relative_times, error_theta_deg)
            if len(error_theta_deg) > 0:
                mean_heading_error = np.mean(error_theta_deg)
                self.heading_error_mean.set_data(
                    [relative_times[0], relative_times[-1]],
                    [mean_heading_error, mean_heading_error],
                )
            self.ax_heading_error.relim()
            self.ax_heading_error.autoscale_view()

            # Update Heading Values (twin axis) - convert to degrees
            theta_ref_deg = np.degrees(ref_data["theta_ref"])
            theta_loc_deg = np.degrees(state_data["theta_loc"])
            self.heading_ref_line.set_data(relative_times, theta_ref_deg)
            self.heading_loc_line.set_data(relative_times, theta_loc_deg)
            self.ax_heading_values.relim()
            self.ax_heading_values.autoscale_view()

            # Update Position X/Y Errors plot
            self.position_error_x_line.set_data(relative_times, error_x)
            self.position_error_y_line.set_data(relative_times, error_y)
            self.ax_position_errors.relim()
            self.ax_position_errors.autoscale_view()

            # Update Velocity plot
            self.velocity_ref_line.set_data(relative_times, v_ref)
            self.velocity_loc_line.set_data(relative_times, v_loc)

            # Add motor command data if available
            if motor_data is not None and len(motor_data["v_cmd"]) > 0:
                min_motor_len = min(len(motor_data["v_cmd"]), min_len)
                self.velocity_cmd_line.set_data(
                    relative_times[:min_motor_len], motor_data["v_cmd"][:min_motor_len]
                )

            self.ax_velocity.relim()
            self.ax_velocity.autoscale_view()

        except Exception as e:
            logging.debug(f"Error updating diagnostic plots: {e}")

    def save_snapshot(self, filename: str = "live_plot_snapshot.png") -> None:
        """Save the current state of the live plot to a file.

        Args:
            filename: Name of the file to save (default: live_plot_snapshot.png).
        """
        save_path = self.run_dir / filename
        self.fig.savefig(save_path, dpi=150, facecolor=self.fig.get_facecolor())
        logging.debug(f"Saved live plot snapshot to: {save_path}")

    def start(self, auto_save: bool = False) -> None:
        """Start the live plotting animation.

        Args:
            auto_save: If True, save a snapshot when the plot window is closed.

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

        # Save snapshot after window is closed if requested
        if auto_save:
            self.save_snapshot()


def start_live_plot(run_dir: Path, update_interval: int = 500, auto_save: bool = False) -> None:
    """Start a live plotting session for a run directory.

    Args:
        run_dir: Directory containing CSV files to monitor.
        update_interval: Update interval in milliseconds (default: 500ms).
        auto_save: If True, save a snapshot when the plot window is closed.
    """
    plotter = LivePlotter(run_dir, update_interval)
    plotter.start(auto_save=auto_save)


if __name__ == "__main__":
    import sys

    # Setup basic logging for standalone usage
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    if len(sys.argv) < 2:
        logging.error("Usage: python -m wagon_control.live_plot <run_directory> [--save]")
        sys.exit(1)

    run_dir = Path(sys.argv[1])
    if not run_dir.exists():
        logging.error(f"Error: Directory not found: {run_dir}")
        sys.exit(1)

    # Check for --save flag
    auto_save = "--save" in sys.argv

    logging.info(f"{TERM_BLUE}✓ Starting live plot for: results/{run_dir.name}{TERM_RESET}")
    start_live_plot(run_dir, auto_save=auto_save)
