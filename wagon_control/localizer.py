"""Localization module for wagon state estimation.

This module provides state estimation by fusing GPS and IMU sensor data.
Implements an Extended Kalman Filter (EKF) approach:
- GPS corrections at 1 Hz for position measurements
- IMU integration at 20 Hz for continuous state prediction
- Gyro bias estimation to reduce heading drift
- Principled uncertainty quantification via covariance propagation
"""

import math
import numpy as np
from typing import Dict, Optional


class WagonLocalizer:
    """State estimator using Extended Kalman Filter (EKF) with GPS and IMU fusion.

    State vector (5 elements):
        - px, py: Position (meters) in global frame
        - theta: Heading (radians)
        - v: Forward velocity (m/s) in body frame
        - b_g: Gyroscope bias (rad/s)

    Sensors:
        - IMU: gyro_z (angular rate), acc_x (longitudinal acceleration) at ~20 Hz
        - GPS: position (px, py) at ~1 Hz

    The EKF maintains state covariance P and propagates uncertainty through
    process model (predict) and measurement model (update).
    """

    def __init__(self, velocity_correction_gain: float = 0.5, config=None):
        """Initialize the EKF localizer.

        Args:
            velocity_correction_gain: Maintained for API compatibility but unused in EKF.
            config: Configuration dictionary or module with EKF parameters.
                    If None, uses default values from wagon_control.config.
        """
        # Import config if not provided
        if config is None:
            from wagon_control import config as cfg
        else:
            cfg = config

        # EKF noise parameters (apply scaling factors for tuning)
        self.Q = cfg.EKF_Q.copy() * cfg.EKF_Q_SCALE  # Process noise covariance (5×5)
        self.R_gps = cfg.EKF_R_GPS.copy() * cfg.EKF_R_SCALE  # GPS measurement noise covariance (2×2)
        self.mahalanobis_threshold = cfg.EKF_MAHALANOBIS_THRESHOLD

        # State vector: [px, py, theta, v, b_g]
        self.x = np.zeros((5, 1))

        # State covariance
        self.P = cfg.EKF_P0.copy()

        # Initialization flag
        self.initialized = False

        # Timestamp for dt calculation
        self.t_prev = 0.0

        # Diagnostics (for logging/tuning)
        self.last_innovation = np.zeros((2, 1))
        self.last_mahalanobis = 0.0
        self.gps_outliers_rejected = 0

    def update_gps(self, x: float, y: float, timestamp: Optional[float] = None) -> None:
        """Update state estimate from GPS position measurement.

        Uses EKF measurement update (correction step) with Mahalanobis distance
        outlier rejection. Rejects GPS measurements that are statistically
        inconsistent with current state estimate and uncertainty.

        Args:
            x: GPS x-coordinate (meters)
            y: GPS y-coordinate (meters)
            timestamp: GPS timestamp (seconds), used for initialization
        """
        # Initialize from first GPS measurement
        if not self.initialized:
            self.x[0, 0] = x  # px
            self.x[1, 0] = y  # py
            self.x[2, 0] = 0.0  # theta (assume facing +x initially)
            self.x[3, 0] = 0.0  # v (stationary initially)
            self.x[4, 0] = 0.0  # b_g (zero bias initially)
            self.t_prev = timestamp if timestamp is not None else 0.0
            self.initialized = True
            return

        # Measurement model: z = h(x) = [px, py]
        H = np.zeros((2, 5))
        H[0, 0] = 1.0  # ∂h₁/∂px = 1
        H[1, 1] = 1.0  # ∂h₂/∂py = 1

        # Measurement vector
        z = np.array([[x], [y]])

        # Predicted measurement
        z_pred = H @ self.x

        # Innovation (measurement residual)
        y_innov = z - z_pred
        self.last_innovation = y_innov.copy()

        # Innovation covariance
        S = H @ self.P @ H.T + self.R_gps

        # Mahalanobis distance for outlier detection
        # d² = innovation.T @ S_inv @ innovation (chi-square distributed, 2 DOF)
        try:
            S_inv = np.linalg.inv(S)
            mahalanobis_sq = float(y_innov.T @ S_inv @ y_innov)
            self.last_mahalanobis = math.sqrt(mahalanobis_sq)

            # Reject outliers using chi-square threshold
            if mahalanobis_sq > self.mahalanobis_threshold:
                self.gps_outliers_rejected += 1
                print(
                    f"GPS outlier rejected: Mahalanobis distance {self.last_mahalanobis:.2f} "
                    f"(threshold {math.sqrt(self.mahalanobis_threshold):.2f}), "
                    f"innovation [{y_innov[0,0]:.2f}, {y_innov[1,0]:.2f}]m"
                )
                return
        except np.linalg.LinAlgError:
            # If S is singular, skip this update
            print("Warning: Singular innovation covariance matrix, skipping GPS update")
            return

        # Kalman gain
        K = self.P @ H.T @ S_inv

        # State update
        self.x = self.x + K @ y_innov

        # Covariance update (Joseph form for numerical stability)
        I = np.eye(5)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ self.R_gps @ K.T

        # Normalize heading to [-π, π]
        self.x[2, 0] = math.atan2(math.sin(self.x[2, 0]), math.cos(self.x[2, 0]))

    def update_imu(
        self, a_x_body: float, a_y_body: float, theta_dot: float, dt: float
    ) -> None:
        """Update state estimate by integrating IMU measurements (EKF prediction step).

        Propagates state through nonlinear kinematic model and linearizes for
        covariance propagation. Uses gyro angular rate and longitudinal acceleration.

        Args:
            a_x_body: Body-frame x-acceleration (forward direction, m/s²)
            a_y_body: Body-frame y-acceleration (lateral direction, m/s²) - ignored
            theta_dot: Angular velocity from gyroscope (rad/s)
            dt: Time step since last update (seconds)
        """
        if not self.initialized:
            return

        if dt <= 0:
            return

        # Extract current state
        px, py, theta, v, b_g = self.x.flatten()

        # Bias-corrected gyro measurement
        omega = theta_dot - b_g

        # Longitudinal acceleration (ignore lateral for differential drive)
        a_long = a_x_body

        # Nonlinear state propagation: x_{k+1} = f(x_k, u_k)
        theta_new = theta + omega * dt
        v_new = v + a_long * dt
        px_new = px + v * math.cos(theta) * dt
        py_new = py + v * math.sin(theta) * dt
        b_g_new = b_g  # Bias modeled as random walk

        # Update state vector
        self.x = np.array([[px_new], [py_new], [theta_new], [v_new], [b_g_new]])

        # Normalize heading to [-π, π]
        self.x[2, 0] = math.atan2(math.sin(self.x[2, 0]), math.cos(self.x[2, 0]))

        # Compute Jacobian F = ∂f/∂x (linearization for covariance propagation)
        F = np.eye(5)

        # ∂px_new/∂theta = -v * sin(theta) * dt
        F[0, 2] = -v * math.sin(theta) * dt

        # ∂px_new/∂v = cos(theta) * dt
        F[0, 3] = math.cos(theta) * dt

        # ∂py_new/∂theta = v * cos(theta) * dt
        F[1, 2] = v * math.cos(theta) * dt

        # ∂py_new/∂v = sin(theta) * dt
        F[1, 3] = math.sin(theta) * dt

        # ∂theta_new/∂b_g = -dt (since omega = theta_dot - b_g)
        F[2, 4] = -dt

        # ∂v_new/∂v = 1 (already in identity)
        # ∂b_g_new/∂b_g = 1 (already in identity)

        # Covariance propagation: P_{k+1} = F @ P_k @ F.T + Q
        self.P = F @ self.P @ F.T + self.Q

    def get_state(self) -> Dict[str, float]:
        """Get current state estimate.

        Returns:
            Dictionary containing:
                - x: Position x-coordinate (m)
                - y: Position y-coordinate (m)
                - theta: Heading angle (rad)
                - v_x: Velocity x-component in global frame (m/s)
                - v_y: Velocity y-component in global frame (m/s)

        Note: EKF tracks forward velocity v in body frame. This method converts
        to global frame velocities (v_x, v_y) for compatibility with existing
        control system.
        """
        px, py, theta, v, b_g = self.x.flatten()

        # Convert body-frame forward velocity to global frame components
        v_x = v * math.cos(theta)
        v_y = v * math.sin(theta)

        return {
            "x": float(px),
            "y": float(py),
            "theta": float(theta),
            "v_x": float(v_x),
            "v_y": float(v_y),
        }

    def get_diagnostics(self) -> Dict[str, float]:
        """Get EKF diagnostic information for tuning and monitoring.

        Returns:
            Dictionary containing:
                - P_trace: Trace of covariance matrix (total uncertainty)
                - P_diag: Diagonal elements of P (individual state uncertainties)
                - innovation_norm: L2 norm of last GPS innovation
                - mahalanobis: Mahalanobis distance of last GPS update
                - gyro_bias: Current gyro bias estimate (rad/s)
                - outliers_rejected: Total count of GPS outliers rejected
        """
        return {
            "P_trace": float(np.trace(self.P)),
            "P_px": float(self.P[0, 0]),
            "P_py": float(self.P[1, 1]),
            "P_theta": float(self.P[2, 2]),
            "P_v": float(self.P[3, 3]),
            "P_b_g": float(self.P[4, 4]),
            "innovation_norm": float(np.linalg.norm(self.last_innovation)),
            "mahalanobis": float(self.last_mahalanobis),
            "gyro_bias": float(self.x[4, 0]),
            "outliers_rejected": self.gps_outliers_rejected,
        }

    def reset(self) -> None:
        """Reset EKF to initial conditions (uninitialized state)."""
        self.x = np.zeros((5, 1))
        # Reload initial covariance
        from wagon_control import config as cfg
        self.P = cfg.EKF_P0.copy()
        self.initialized = False
        self.t_prev = 0.0
        self.last_innovation = np.zeros((2, 1))
        self.last_mahalanobis = 0.0
        self.gps_outliers_rejected = 0
