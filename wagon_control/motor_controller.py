"""Motor controller for velocity and angular velocity feedback.

This module provides a feedback controller that sits between the path follower
and inverse kinematics, using localized velocity estimates to correct for
tracking errors, model mismatch, and disturbances.
"""

import math
from typing import Dict, Tuple


class MotorController:
    """PID feedback controller with acceleration feedforward for velocity tracking.

    The controller receives reference commands (v_ref, omega_ref) from the
    path follower and localized velocity estimates (v_loc, omega_loc) from
    the state estimator, then applies PID feedback with optional feedforward
    to improve velocity tracking and eliminate steady-state errors.

    Control law:
        v_cmd = v_ref + K_v * e_v + K_i_v * integral(e_v) + K_d_v * d(e_v)/dt + K_f_v * a_ref
        omega_cmd = omega_ref + K_omega * e_omega + K_i_omega * integral(e_omega) + K_d_omega * d(e_omega)/dt + K_f_omega * alpha_ref

    Attributes:
        k_v: Proportional gain for linear velocity (default: 0.5)
        k_omega: Proportional gain for angular velocity (default: 0.5)
        k_i_v: Integral gain for linear velocity (default: 0.1)
        k_i_omega: Integral gain for angular velocity (default: 0.1)
        k_d_v: Derivative gain for linear velocity (default: 0.0)
        k_d_omega: Derivative gain for angular velocity (default: 0.0)
        k_f_v: Feedforward gain for linear acceleration (default: 0.0)
        k_f_omega: Feedforward gain for angular acceleration (default: 0.0)
    """

    def __init__(
        self,
        k_v: float = 0.5,
        k_omega: float = 0.5,
        k_i_v: float = 0.1,
        k_i_omega: float = 0.1,
        k_d_v: float = 0.0,
        k_d_omega: float = 0.0,
        k_f_v: float = 0.0,
        k_f_omega: float = 0.0,
        velocity_compensation: float = 1.0,
        omega_compensation: float = 1.0,
    ):
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
            k_d_v: Derivative gain for linear velocity correction.
                Range [0, 0.5]. Higher = more damping and faster transient response.
                Default: 0.0 (no derivative action)
            k_d_omega: Derivative gain for angular velocity correction.
                Range [0, 0.5]. Higher = more damping and faster transient response.
                Default: 0.0 (no derivative action)
            k_f_v: Feedforward gain for linear acceleration.
                Range [0, 1]. Higher = more anticipatory control.
                Default: 0.0 (no feedforward)
            k_f_omega: Feedforward gain for angular acceleration.
                Range [0, 1]. Higher = more anticipatory control.
                Default: 0.0 (no feedforward)
            velocity_compensation: Compensation factor for actuator response lag.
                Range [1.0, 1.2]. Multiplies final v_cmd to compensate for systematic deficit.
                Default: 1.0 (no compensation)
            omega_compensation: Compensation factor for angular velocity.
                Range [1.0, 1.2]. Multiplies final omega_cmd.
                Default: 1.0 (no compensation)
        """
        # Proportional gains
        self.k_v = k_v
        self.k_omega = k_omega

        # Integral gains
        self.k_i_v = k_i_v
        self.k_i_omega = k_i_omega

        # Derivative gains
        self.k_d_v = k_d_v
        self.k_d_omega = k_d_omega

        # Feedforward gains
        self.k_f_v = k_f_v
        self.k_f_omega = k_f_omega

        # Velocity compensation factors
        self.velocity_compensation = velocity_compensation
        self.omega_compensation = omega_compensation

        # Integral state (accumulated error)
        self.integral_v: float = 0.0
        self.integral_omega: float = 0.0

        # Previous error for derivative computation
        self.prev_v_err: float = 0.0
        self.prev_omega_err: float = 0.0

        # Anti-windup limits
        self.integral_limit: float = 0.5  # Clamp integral at ±0.5

    def transform_velocity_to_body_frame(self, v_x: float, v_y: float, theta: float) -> float:
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
        self,
        v_ref: float,
        omega_ref: float,
        v_loc: float,
        omega_loc: float,
        dt: float,
        a_ref: float = 0.0,
        alpha_ref: float = 0.0,
    ) -> Tuple[float, float]:
        """Compute corrected velocity commands using PID feedback with optional feedforward.

        Args:
            v_ref: Reference linear velocity from path follower (m/s)
            omega_ref: Reference angular velocity from path follower (rad/s)
            v_loc: Localized forward velocity (m/s)
            omega_loc: Localized angular velocity (rad/s)
            dt: Time step since last control update (seconds)
            a_ref: Reference linear acceleration for feedforward (m/s²). Default: 0.0
            alpha_ref: Reference angular acceleration for feedforward (rad/s²). Default: 0.0

        Returns:
            Tuple of (v_cmd, omega_cmd) - corrected velocity commands
        """
        # Compute velocity errors
        v_err = v_ref - v_loc
        omega_err = omega_ref - omega_loc

        # Compute derivative of error (rate of change)
        if dt > 0:
            v_err_derivative = (v_err - self.prev_v_err) / dt
            omega_err_derivative = (omega_err - self.prev_omega_err) / dt
        else:
            v_err_derivative = 0.0
            omega_err_derivative = 0.0

        # Store current error for next iteration
        self.prev_v_err = v_err
        self.prev_omega_err = omega_err

        # Accumulate integral of error with anti-windup
        self.integral_v += v_err * dt
        self.integral_omega += omega_err * dt

        # Apply anti-windup: clamp integral terms
        self.integral_v = max(-self.integral_limit, min(self.integral_limit, self.integral_v))
        self.integral_omega = max(
            -self.integral_limit, min(self.integral_limit, self.integral_omega)
        )

        # PID + Feedforward control law: P term + I term + D term + FF term
        v_cmd = (
            v_ref
            + self.k_v * v_err
            + self.k_i_v * self.integral_v
            + self.k_d_v * v_err_derivative
            + self.k_f_v * a_ref
        )
        omega_cmd = (
            omega_ref
            + self.k_omega * omega_err
            + self.k_i_omega * self.integral_omega
            + self.k_d_omega * omega_err_derivative
            + self.k_f_omega * alpha_ref
        )

        # Apply velocity compensation for actuator response lag
        v_cmd = v_cmd * self.velocity_compensation
        omega_cmd = omega_cmd * self.omega_compensation

        return v_cmd, omega_cmd

    def reset(self) -> None:
        """Reset integral and derivative states to zero.

        Call this when starting a new control session or when integral
        windup needs to be cleared.
        """
        self.integral_v = 0.0
        self.integral_omega = 0.0
        self.prev_v_err = 0.0
        self.prev_omega_err = 0.0

    def get_diagnostics(
        self,
        v_ref: float,
        omega_ref: float,
        v_loc: float,
        omega_loc: float,
        v_cmd: float,
        omega_cmd: float,
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
            "v_ref": v_ref,
            "omega_ref": omega_ref,
            "v_loc": v_loc,
            "omega_loc": omega_loc,
            "v_err": v_err,
            "omega_err": omega_err,
            "v_cmd": v_cmd,
            "omega_cmd": omega_cmd,
            "integral_v": self.integral_v,
            "integral_omega": self.integral_omega,
        }
