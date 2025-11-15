"""Motor controller for velocity and angular velocity feedback.

This module provides a feedback controller that sits between the path follower
and inverse kinematics, using localized velocity estimates to correct for
tracking errors, model mismatch, and disturbances.
"""

import math
from typing import Dict, Tuple


class MotorController:
    """PI feedback controller for velocity tracking.

    The controller receives reference commands (v_ref, omega_ref) from the
    path follower and localized velocity estimates (v_loc, omega_loc) from
    the state estimator, then applies PI feedback to improve velocity tracking
    and eliminate steady-state errors.

    Control law:
        v_cmd = v_ref + K_v * e_v + K_i_v * integral(e_v)
        omega_cmd = omega_ref + K_omega * e_omega + K_i_omega * integral(e_omega)

    Attributes:
        k_v: Proportional gain for linear velocity (default: 0.5)
        k_omega: Proportional gain for angular velocity (default: 0.5)
        k_i_v: Integral gain for linear velocity (default: 0.1)
        k_i_omega: Integral gain for angular velocity (default: 0.1)
    """

    def __init__(self, k_v: float = 0.5, k_omega: float = 0.5,
                 k_i_v: float = 0.1, k_i_omega: float = 0.1):
        """Initialize the motor controller.

        Args:
            k_v: Proportional gain for linear velocity correction.
                Range [0, 2]. Higher = more aggressive correction.
                Default: 0.5 (moderate correction)
            k_omega: Proportional gain for angular velocity correction.
                Range [0, 2]. Higher = more aggressive correction.
                Default: 0.5 (moderate correction)
            k_i_v: Integral gain for linear velocity correction.
                Range [0, 0.5]. Higher = faster elimination of steady-state error.
                Default: 0.1 (conservative integral action)
            k_i_omega: Integral gain for angular velocity correction.
                Range [0, 0.5]. Higher = faster elimination of steady-state error.
                Default: 0.1 (conservative integral action)
        """
        # Proportional gains
        self.k_v = k_v
        self.k_omega = k_omega

        # Integral gains
        self.k_i_v = k_i_v
        self.k_i_omega = k_i_omega

        # Integral state (accumulated error)
        self.integral_v: float = 0.0
        self.integral_omega: float = 0.0

        # Anti-windup limits
        self.integral_limit: float = 0.5  # Clamp integral at ±0.5

    def transform_velocity_to_body_frame(
        self, v_x: float, v_y: float, theta: float
    ) -> float:
        """Transform global frame velocities to body frame forward velocity.

        Args:
            v_x: Velocity in global x direction (m/s)
            v_y: Velocity in global y direction (m/s)
            theta: Heading angle (radians)

        Returns:
            Forward velocity in body frame (m/s)
        """
        # Project global velocity onto body frame forward direction
        # v_forward = v_x * cos(theta) + v_y * sin(theta)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        v_forward = v_x * cos_theta + v_y * sin_theta
        return v_forward

    def compute_control(
        self, v_ref: float, omega_ref: float, v_loc: float, omega_loc: float, dt: float
    ) -> Tuple[float, float]:
        """Compute corrected velocity commands using PI feedback.

        Args:
            v_ref: Reference linear velocity from path follower (m/s)
            omega_ref: Reference angular velocity from path follower (rad/s)
            v_loc: Localized forward velocity (m/s)
            omega_loc: Localized angular velocity (rad/s)
            dt: Time step since last control update (seconds)

        Returns:
            Tuple of (v_cmd, omega_cmd) - corrected velocity commands
        """
        # Compute velocity errors
        v_err = v_ref - v_loc
        omega_err = omega_ref - omega_loc

        # Accumulate integral of error with anti-windup
        self.integral_v += v_err * dt
        self.integral_omega += omega_err * dt

        # Apply anti-windup: clamp integral terms
        self.integral_v = max(-self.integral_limit, min(self.integral_limit, self.integral_v))
        self.integral_omega = max(-self.integral_limit, min(self.integral_limit, self.integral_omega))

        # PI control law: P term + I term
        v_cmd = v_ref + self.k_v * v_err + self.k_i_v * self.integral_v
        omega_cmd = omega_ref + self.k_omega * omega_err + self.k_i_omega * self.integral_omega

        return v_cmd, omega_cmd

    def reset(self) -> None:
        """Reset integral states to zero.

        Call this when starting a new control session or when integral
        windup needs to be cleared.
        """
        self.integral_v = 0.0
        self.integral_omega = 0.0

    def get_diagnostics(
        self, v_ref: float, omega_ref: float, v_loc: float, omega_loc: float,
        v_cmd: float, omega_cmd: float
    ) -> Dict[str, float]:
        """Get diagnostic information for logging and debugging.

        Args:
            v_ref: Reference linear velocity (m/s)
            omega_ref: Reference angular velocity (rad/s)
            v_loc: Localized forward velocity (m/s)
            omega_loc: Localized angular velocity (rad/s)
            v_cmd: Output linear velocity command (m/s)
            omega_cmd: Output angular velocity command (rad/s)

        Returns:
            Dictionary containing all diagnostic values
        """
        v_err = v_ref - v_loc
        omega_err = omega_ref - omega_loc

        return {
            'v_ref': v_ref,
            'omega_ref': omega_ref,
            'v_loc': v_loc,
            'omega_loc': omega_loc,
            'v_err': v_err,
            'omega_err': omega_err,
            'v_cmd': v_cmd,
            'omega_cmd': omega_cmd,
            'integral_v': self.integral_v,
            'integral_omega': self.integral_omega,
        }
