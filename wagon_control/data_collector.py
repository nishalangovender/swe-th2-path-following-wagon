"""Data collection and CSV logging for wagon sensor and control data.

This module provides CSV data logging for:
- IMU data (accelerometer, gyroscope)
- GPS data (position measurements)
- State estimates (localized position, velocity, heading)
- Reference trajectory (planned path)
- Motor controller diagnostics (velocities, errors, commands)
- EKF diagnostics (covariance, innovations, gyro bias)
- Tracking metrics (position errors, L2 error, localization quality)
- Final score (ground truth from system)
"""

import csv
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, TextIO

from .config import TERM_BLUE, TERM_RESET


class DataCollector:
    """Manages CSV file creation and logging for wagon control data.

    This class handles all data logging responsibilities:
    - Creates timestamped output directories
    - Initializes CSV files with headers
    - Writes sensor, state, and control data
    - Ensures proper cleanup on shutdown

    Attributes:
        run_dir: Directory path for this run's output files.
        imu_csv_file: File handle for IMU data CSV.
        gps_csv_file: File handle for GPS data CSV.
        state_csv_file: File handle for state estimates CSV.
        reference_csv_file: File handle for reference trajectory CSV.
        motor_csv_file: File handle for motor controller CSV.
        ekf_csv_file: File handle for EKF diagnostics CSV.
        tracking_csv_file: File handle for tracking metrics CSV.
        score_path: Path for final score text file.
    """

    def __init__(self, output_dir: str = ".", run_dir: Optional[str] = None) -> None:
        """Initialize the data collector.

        Args:
            output_dir: Base directory for output files (default: current directory).
            run_dir: Optional specific run directory. If None, creates timestamped
                directory. Can also be set via RUN_DIR environment variable.

        Raises:
            ValueError: If output_dir is not a valid directory.
        """
        # Validate output directory
        output_path = Path(output_dir)
        if output_path.exists() and not output_path.is_dir():
            raise ValueError(f"Output path exists but is not a directory: {output_dir}")

        # CSV file handles
        self.imu_csv_file: Optional[TextIO] = None
        self.imu_csv_writer: Any = None
        self.gps_csv_file: Optional[TextIO] = None
        self.gps_csv_writer: Any = None
        self.state_csv_file: Optional[TextIO] = None
        self.state_csv_writer: Any = None
        self.reference_csv_file: Optional[TextIO] = None
        self.reference_csv_writer: Any = None
        self.motor_csv_file: Optional[TextIO] = None
        self.motor_csv_writer: Any = None
        self.ekf_csv_file: Optional[TextIO] = None
        self.ekf_csv_writer: Any = None
        self.tracking_csv_file: Optional[TextIO] = None
        self.tracking_csv_writer: Any = None

        # Determine run directory
        if run_dir:
            # Use provided run directory
            self.run_dir: Path = Path(run_dir)
        elif env_run_dir := os.environ.get("RUN_DIR"):
            # Use environment variable (for live plotting integration)
            self.run_dir = Path(env_run_dir)
        else:
            # Create timestamped directory: results/run_YYYYMMDD_HHMMSS/
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            results_dir = output_path / "results"
            self.run_dir = results_dir / f"run_{timestamp}"

        # Create directory structure
        self.run_dir.mkdir(parents=True, exist_ok=True)

        # Define output file paths
        self.imu_output_path: Path = self.run_dir / "imu_data.csv"
        self.gps_output_path: Path = self.run_dir / "gps_data.csv"
        self.state_output_path: Path = self.run_dir / "state_data.csv"
        self.reference_output_path: Path = self.run_dir / "reference_data.csv"
        self.motor_output_path: Path = self.run_dir / "motor_data.csv"
        self.ekf_output_path: Path = self.run_dir / "ekf_diagnostics.csv"
        self.tracking_output_path: Path = self.run_dir / "tracking_metrics.csv"
        self.score_output_path: Path = self.run_dir / "score.txt"

    def setup(self) -> None:
        """Initialize CSV files with headers.

        Creates and opens all CSV files with appropriate column headers.
        Must be called before writing data.
        """
        # Setup IMU CSV file
        self.imu_csv_file = open(self.imu_output_path, "w", newline="")
        self.imu_csv_writer = csv.writer(self.imu_csv_file)
        self.imu_csv_writer.writerow(["timestamp", "x_dot", "y_dot", "theta_dot"])
        self.imu_csv_file.flush()

        # Setup GPS CSV file
        self.gps_csv_file = open(self.gps_output_path, "w", newline="")
        self.gps_csv_writer = csv.writer(self.gps_csv_file)
        self.gps_csv_writer.writerow(["timestamp", "x", "y"])
        self.gps_csv_file.flush()

        # Setup State CSV file (localized estimates)
        self.state_csv_file = open(self.state_output_path, "w", newline="")
        self.state_csv_writer = csv.writer(self.state_csv_file)
        self.state_csv_writer.writerow(["timestamp", "x_loc", "y_loc", "theta_loc", "v_x", "v_y"])
        self.state_csv_file.flush()

        # Setup Reference CSV file (path follower's time-based reference)
        self.reference_csv_file = open(self.reference_output_path, "w", newline="")
        self.reference_csv_writer = csv.writer(self.reference_csv_file)
        self.reference_csv_writer.writerow(
            ["timestamp", "elapsed_time", "x_ref", "y_ref", "theta_ref"]
        )
        self.reference_csv_file.flush()

        # Setup Motor CSV file (motor controller feedback)
        self.motor_csv_file = open(self.motor_output_path, "w", newline="")
        self.motor_csv_writer = csv.writer(self.motor_csv_file)
        self.motor_csv_writer.writerow(
            [
                "timestamp",
                "v_ref",
                "omega_ref",
                "v_loc",
                "omega_loc",
                "v_err",
                "omega_err",
                "v_cmd",
                "omega_cmd",
            ]
        )
        self.motor_csv_file.flush()

        # Setup EKF diagnostics CSV file
        self.ekf_csv_file = open(self.ekf_output_path, "w", newline="")
        self.ekf_csv_writer = csv.writer(self.ekf_csv_file)
        self.ekf_csv_writer.writerow(
            [
                "timestamp",
                "P_trace",
                "P_px",
                "P_py",
                "P_theta",
                "P_v",
                "P_b_g",
                "innovation_norm",
                "mahalanobis",
                "gyro_bias",
                "outliers_rejected",
                "slip_magnitude",
            ]
        )
        self.ekf_csv_file.flush()

        # Setup tracking metrics CSV file (position errors and L2 tracking)
        self.tracking_csv_file = open(self.tracking_output_path, "w", newline="")
        self.tracking_csv_writer = csv.writer(self.tracking_csv_file)
        self.tracking_csv_writer.writerow(
            [
                "timestamp",
                "error_x",
                "error_y",
                "error_l2",
                "cumulative_l2_error",
                "sample_count",
                "avg_sample_error_mm",
            ]
        )
        self.tracking_csv_file.flush()

        print(
            f"{TERM_BLUE}✓ Initialized data collection to results/{self.run_dir.name}/{TERM_RESET}"
        )

    def log_imu(
        self,
        timestamp: float,
        x_dot: Optional[float] = None,
        y_dot: Optional[float] = None,
        theta_dot: Optional[float] = None,
    ) -> None:
        """Log IMU data to CSV.

        Args:
            timestamp: Sensor timestamp (seconds).
            x_dot: Forward acceleration (m/s²), optional.
            y_dot: Lateral acceleration (m/s²), optional.
            theta_dot: Angular velocity (rad/s), optional.
        """
        self.imu_csv_writer.writerow(
            [
                timestamp,
                x_dot if x_dot is not None else "",
                y_dot if y_dot is not None else "",
                theta_dot if theta_dot is not None else "",
            ]
        )
        if self.imu_csv_file:
            self.imu_csv_file.flush()

    def log_gps(self, timestamp: float, x: float, y: float) -> None:
        """Log GPS data to CSV.

        Args:
            timestamp: Sensor timestamp (seconds).
            x: GPS x-coordinate (meters).
            y: GPS y-coordinate (meters).
        """
        self.gps_csv_writer.writerow([timestamp, x, y])
        if self.gps_csv_file:
            self.gps_csv_file.flush()

    def log_state(
        self, timestamp: float, x: float, y: float, theta: float, v_x: float, v_y: float
    ) -> None:
        """Log localized state estimate to CSV.

        Args:
            timestamp: Current time (seconds).
            x: Estimated x position (meters).
            y: Estimated y position (meters).
            theta: Estimated heading (radians).
            v_x: Estimated x velocity (m/s).
            v_y: Estimated y velocity (m/s).
        """
        self.state_csv_writer.writerow([timestamp, x, y, theta, v_x, v_y])
        if self.state_csv_file:
            self.state_csv_file.flush()

    def log_reference(
        self, timestamp: float, elapsed_time: float, x_ref: float, y_ref: float, theta_ref: float
    ) -> None:
        """Log reference trajectory point to CSV.

        Args:
            timestamp: Current time (seconds).
            elapsed_time: Elapsed time since start (seconds).
            x_ref: Reference x position (meters).
            y_ref: Reference y position (meters).
            theta_ref: Reference heading (radians).
        """
        self.reference_csv_writer.writerow([timestamp, elapsed_time, x_ref, y_ref, theta_ref])
        if self.reference_csv_file:
            self.reference_csv_file.flush()

    def log_motor_diagnostics(self, timestamp: float, diagnostics: Dict[str, float]) -> None:
        """Log motor controller diagnostics to CSV.

        Args:
            timestamp: Current time (seconds).
            diagnostics: Dictionary containing motor controller data with keys:
                'v_ref', 'omega_ref', 'v_loc', 'omega_loc',
                'v_err', 'omega_err', 'v_cmd', 'omega_cmd'
        """
        self.motor_csv_writer.writerow(
            [
                timestamp,
                diagnostics["v_ref"],
                diagnostics["omega_ref"],
                diagnostics["v_loc"],
                diagnostics["omega_loc"],
                diagnostics["v_err"],
                diagnostics["omega_err"],
                diagnostics["v_cmd"],
                diagnostics["omega_cmd"],
            ]
        )
        if self.motor_csv_file:
            self.motor_csv_file.flush()

    def log_ekf_diagnostics(self, timestamp: float, diagnostics: Dict[str, float]) -> None:
        """Log EKF diagnostics to CSV.

        Args:
            timestamp: Current time (seconds).
            diagnostics: Dictionary containing EKF data with keys:
                'P_trace', 'P_px', 'P_py', 'P_theta', 'P_v', 'P_b_g',
                'innovation_norm', 'mahalanobis', 'gyro_bias', 'outliers_rejected',
                'slip_magnitude'
        """
        self.ekf_csv_writer.writerow(
            [
                timestamp,
                diagnostics["P_trace"],
                diagnostics["P_px"],
                diagnostics["P_py"],
                diagnostics["P_theta"],
                diagnostics["P_v"],
                diagnostics["P_b_g"],
                diagnostics["innovation_norm"],
                diagnostics["mahalanobis"],
                diagnostics["gyro_bias"],
                diagnostics["outliers_rejected"],
                diagnostics["slip_magnitude"],
            ]
        )
        if self.ekf_csv_file:
            self.ekf_csv_file.flush()

    def log_tracking_metrics(
        self,
        timestamp: float,
        error_x: float,
        error_y: float,
        error_l2: float,
        cumulative_l2_error: float,
        sample_count: int,
        avg_sample_error_mm: float,
    ) -> None:
        """Log tracking error metrics to CSV.

        Args:
            timestamp: Current time (seconds).
            error_x: X position error (meters).
            error_y: Y position error (meters).
            error_l2: L2 norm of position error (meters).
            cumulative_l2_error: Running sum of L2 errors (meters).
            sample_count: Number of samples so far.
            avg_sample_error_mm: Average error per sample (millimeters).
        """
        self.tracking_csv_writer.writerow(
            [
                timestamp,
                error_x,
                error_y,
                error_l2,
                cumulative_l2_error,
                sample_count,
                avg_sample_error_mm,
            ]
        )
        if self.tracking_csv_file:
            self.tracking_csv_file.flush()

    def log_final_score(self, score: float) -> None:
        """Log final ground truth score to text file.

        Args:
            score: Final L2 error from system (meters).
        """
        with open(self.score_output_path, "w") as f:
            f.write(f"{score:.6f}\n")
        print(
            f"{TERM_BLUE}✓ Saved final score: {score:.3f}m to {self.score_output_path.name}{TERM_RESET}"
        )

    def cleanup(self) -> None:
        """Close all CSV files and log final output location."""
        if self.imu_csv_file:
            self.imu_csv_file.close()
        if self.gps_csv_file:
            self.gps_csv_file.close()
        if self.state_csv_file:
            self.state_csv_file.close()
        if self.reference_csv_file:
            self.reference_csv_file.close()
        if self.motor_csv_file:
            self.motor_csv_file.close()
        if self.ekf_csv_file:
            self.ekf_csv_file.close()
        if self.tracking_csv_file:
            self.tracking_csv_file.close()

        print(f"{TERM_BLUE}✓ Saved sensor data to results/{self.run_dir.name}/{TERM_RESET}")

    def __enter__(self) -> "DataCollector":
        """Context manager entry point.

        Returns:
            Self reference for use in with statement.
        """
        self.setup()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """Context manager exit point - ensures cleanup is called.

        Args:
            exc_type: Exception type if an exception occurred.
            exc_val: Exception value if an exception occurred.
            exc_tb: Exception traceback if an exception occurred.
        """
        self.cleanup()
