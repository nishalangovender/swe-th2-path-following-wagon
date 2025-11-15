"""Localization module for wagon state estimation.

This module provides state estimation by fusing GPS and IMU sensor data.
Implements a complementary filter approach:
- GPS corrections at 1 Hz for position
- IMU integration at 20 Hz for continuous state prediction
- Accelerometer odometry for velocity and position between GPS updates
"""

import math
from typing import Dict, Optional


class WagonLocalizer:
    """State estimator using complementary filter with GPS and IMU fusion.

    State components:
        - x, y: Position (meters) - predicted from IMU, corrected by GPS
        - theta: Heading (radians) - integrated from gyroscope
        - v_x, v_y: Velocity (m/s) in global frame - integrated from accelerometer
    """

    def __init__(self, velocity_correction_gain: float = 0.5):
        """Initialize the localizer with zero state.

        Args:
            velocity_correction_gain: Gain for velocity correction when GPS arrives.
                Range [0, 1]. Higher = more aggressive correction of drift.
                Default: 0.5 (moderate damping, balanced stability and drift correction)
        """
        # Position and heading
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0  # Initial heading along +x axis

        # Velocity (global frame)
        self.v_x: float = 0.0
        self.v_y: float = 0.0

        # Filtered velocity (low-pass filtered for feedback control)
        self.v_x_filtered: float = 0.0
        self.v_y_filtered: float = 0.0
        self.velocity_filter_alpha: float = 0.3  # EMA smoothing: 30% new, 70% old

        # Velocity correction gain
        self.velocity_correction_gain: float = velocity_correction_gain

        # IMU bias compensation (gyro and accelerometer are in same unit)
        # Gyroscope bias (estimated from baseline drift: -16.68° over 20s)
        self.gyro_bias_compensation: float = 0.015  # rad/s
        # Accelerometer X-axis bias (from stationary phase analysis: 0.0958 m/s²)
        self.accel_x_bias: float = 0.096  # m/s²

    def update_gps(self, x: float, y: float, timestamp: float = None) -> None:
        """Update position estimate from GPS measurement and correct velocity drift.

        Uses complementary filter approach:
        1. GPS outlier detection (reject if >5m from predicted)
        2. Compute position error (GPS - estimated)
        3. Hard reset position to GPS (full correction)
        4. Soft correct velocity proportional to position error (damped correction)

        Args:
            x: GPS x-coordinate (meters)
            y: GPS y-coordinate (meters)
            timestamp: GPS timestamp (seconds), optional (unused in baseline)
        """
        # GPS outlier detection: reject updates too far from predicted position
        innovation_x = x - self.x
        innovation_y = y - self.y
        innovation_magnitude = math.sqrt(innovation_x**2 + innovation_y**2)

        if innovation_magnitude > 5.0:  # Threshold: 5 meters
            # Reject likely GPS outlier (multipath, atmospheric error, etc.)
            print(f"GPS outlier rejected: {innovation_magnitude:.2f}m deviation from predicted position")
            return  # Skip this GPS update

        # Compute position error for velocity correction
        error_x = innovation_x
        error_y = innovation_y

        # Hard reset position to GPS measurement (full correction)
        self.x = x
        self.y = y

        # Soft correct velocity drift (proportional to position error)
        # This prevents unbounded velocity drift from accelerometer integration
        self.v_x += self.velocity_correction_gain * error_x
        self.v_y += self.velocity_correction_gain * error_y

    def update_imu(self, a_x_body: float, a_y_body: float, theta_dot: float, dt: float) -> None:
        """Update state estimate by integrating IMU measurements.

        Performs complementary filter prediction step:
        1. Apply IMU bias compensation
        2. Integrate gyroscope to update heading
        3. Transform body-frame accelerations to global frame
        4. Integrate accelerations to update velocity
        5. Integrate velocity to update position

        Args:
            a_x_body: Body-frame x-acceleration (forward direction, m/s²)
            a_y_body: Body-frame y-acceleration (lateral direction, m/s²)
            theta_dot: Angular velocity from gyroscope (rad/s)
            dt: Time step since last update (seconds)
        """
        # Step 1: Apply IMU bias compensation
        a_x_body = a_x_body - self.accel_x_bias  # Remove forward acceleration bias
        theta_dot = theta_dot + self.gyro_bias_compensation  # Correct gyro bias

        # Step 2: Update heading by integrating corrected gyroscope
        self.theta += theta_dot * dt
        # Normalize angle to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Step 3: Transform body-frame accelerations to global frame
        # Differential drive constraint: robot cannot move sideways (v_y_body = 0)
        # Therefore, ignore lateral acceleration and only use forward acceleration
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)

        # Only use forward acceleration (a_x_body), ignore lateral (a_y_body)
        # to enforce kinematic constraint of differential drive
        a_x_global = a_x_body * cos_theta
        a_y_global = a_x_body * sin_theta

        # Step 4: Integrate accelerations to update velocity (global frame)
        self.v_x += a_x_global * dt
        self.v_y += a_y_global * dt

        # Step 5: Apply low-pass filter to velocities for smooth feedback control
        # Exponential moving average: v_filtered = alpha * v_new + (1-alpha) * v_old
        alpha = self.velocity_filter_alpha
        self.v_x_filtered = alpha * self.v_x + (1 - alpha) * self.v_x_filtered
        self.v_y_filtered = alpha * self.v_y + (1 - alpha) * self.v_y_filtered

        # Step 6: Integrate velocity to update position
        self.x += self.v_x * dt
        self.y += self.v_y * dt

    def get_state(self) -> Dict[str, float]:
        """Get current state estimate.

        Returns:
            Dictionary containing:
                - x: Position x-coordinate (m)
                - y: Position y-coordinate (m)
                - theta: Heading angle (rad)
                - v_x: Velocity x-component in global frame (m/s, filtered)
                - v_y: Velocity y-component in global frame (m/s, filtered)
        """
        return {
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'v_x': self.v_x_filtered,  # Return filtered velocity for feedback control
            'v_y': self.v_y_filtered   # Return filtered velocity for feedback control
        }

    def reset(self) -> None:
        """Reset state to initial conditions (origin, facing +x, stationary)."""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_x_filtered = 0.0
        self.v_y_filtered = 0.0
