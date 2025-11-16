"""Pure Pursuit Path Follower for wagon control.

This module implements a pure pursuit path following algorithm that:
- Estimates heading by integrating gyroscope data
- Finds a lookahead point on the reference path
- Computes velocity and angular velocity commands
"""

import math
from typing import Optional, Tuple

import numpy as np


class PurePursuitFollower:
    """Pure Pursuit path follower using localized state.

    Uses state estimates from a localization module (position and heading)
    to follow a time-parameterized reference trajectory.
    """

    def __init__(
        self,
        lookahead_distance: float = 0.7,
        adaptive: bool = True,
        lookahead_time: float = 0.8,
        lookahead_offset: float = 0.3,
        min_lookahead: float = 0.5,
        max_lookahead: float = 2.0,
    ):
        """Initialize the pure pursuit controller.

        Args:
            lookahead_distance: Base distance ahead on path to target (meters).
                Used if adaptive=False.
            adaptive: If True, use velocity-adaptive lookahead. Default: True.
            lookahead_time: Time-based lookahead gain (seconds). Default: 0.8.
                Lookahead = lookahead_time * |v| + lookahead_offset
            lookahead_offset: Minimum base lookahead (meters). Default: 0.3.
            min_lookahead: Minimum lookahead distance (meters). Default: 0.5.
            max_lookahead: Maximum lookahead distance (meters). Default: 2.0.
        """
        self.base_lookahead_distance = lookahead_distance
        self.adaptive = adaptive
        self.lookahead_time = lookahead_time
        self.lookahead_offset = lookahead_offset
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.lookahead_distance = lookahead_distance  # Will be updated if adaptive
        self.start_time: Optional[float] = None  # Start time for trajectory following

    def compute_adaptive_lookahead(self, velocity: float) -> float:
        """Compute velocity-adaptive lookahead distance.

        Higher speeds → longer lookahead for smoother tracking
        Lower speeds → shorter lookahead for tighter control

        Args:
            velocity: Current or reference velocity (m/s)

        Returns:
            Adaptive lookahead distance (meters), clamped to [min, max]
        """
        if not self.adaptive:
            return self.base_lookahead_distance

        # Velocity-proportional lookahead: lookahead_time * |v| + offset
        lookahead = self.lookahead_time * abs(velocity) + self.lookahead_offset

        # Clamp to safe bounds
        lookahead = max(self.min_lookahead, min(self.max_lookahead, lookahead))

        return lookahead

    def find_closest_point(
        self, current_x: float, current_y: float, path_x: np.ndarray, path_y: np.ndarray
    ) -> int:
        """Find index of closest point on path to current position.

        Args:
            current_x: Current x position (m)
            current_y: Current y position (m)
            path_x: Array of path x coordinates
            path_y: Array of path y coordinates

        Returns:
            Index of closest point on path
        """
        distances = np.sqrt((path_x - current_x) ** 2 + (path_y - current_y) ** 2)
        return int(np.argmin(distances))

    def find_lookahead_point(
        self,
        current_x: float,
        current_y: float,
        path_x: np.ndarray,
        path_y: np.ndarray,
        start_idx: int,
    ) -> Tuple[float, float, int]:
        """Find lookahead point on path at target distance ahead.

        Args:
            current_x: Current x position (m)
            current_y: Current y position (m)
            path_x: Array of path x coordinates
            path_y: Array of path y coordinates
            start_idx: Index to start searching from (typically closest point)

        Returns:
            Tuple of (lookahead_x, lookahead_y, lookahead_idx)
        """
        # Search forward from start_idx to find point at lookahead distance
        for i in range(start_idx, len(path_x)):
            dx = path_x[i] - current_x
            dy = path_y[i] - current_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance >= self.lookahead_distance:
                return path_x[i], path_y[i], i

        # If we reach end of path, return last point
        return path_x[-1], path_y[-1], len(path_x) - 1

    def find_time_index(self, path_t: np.ndarray, target_time: float) -> int:
        """Find index in path corresponding to target time.

        Args:
            path_t: Array of path time values
            target_time: Target time to find (seconds)

        Returns:
            Index closest to target time, clamped to valid range
        """
        if target_time <= path_t[0]:
            return 0
        if target_time >= path_t[-1]:
            return len(path_t) - 1

        # Find closest time index
        idx = int(np.argmin(np.abs(path_t - target_time)))
        return idx

    def compute_reference_velocity(
        self, path_x: np.ndarray, path_y: np.ndarray, path_t: np.ndarray, ref_idx: int
    ) -> float:
        """Compute reference velocity from path time derivatives.

        Args:
            path_x: Array of path x coordinates
            path_y: Array of path y coordinates
            path_t: Array of path time values
            ref_idx: Index of reference position on path (based on time)

        Returns:
            Reference velocity (m/s)
        """
        # Use finite differences to compute velocity
        if ref_idx >= len(path_x) - 1:
            # At end of path
            return 0.0

        # Forward difference
        dx = path_x[ref_idx + 1] - path_x[ref_idx]
        dy = path_y[ref_idx + 1] - path_y[ref_idx]
        dt = path_t[ref_idx + 1] - path_t[ref_idx]

        if dt > 0:
            v_ref = math.sqrt(dx**2 + dy**2) / dt
            # Clamp to reasonable bounds (inverse kinematics limits)
            return float(min(v_ref, 2.0))
        else:
            return 0.0

    def compute_control(
        self,
        state: dict,
        path_x: np.ndarray,
        path_y: np.ndarray,
        path_t: np.ndarray,
        current_time: float,
    ) -> Tuple[float, float]:
        """Compute velocity commands using pure pursuit algorithm.

        Args:
            state: Dictionary from localizer containing:
                - x: Current x position (m)
                - y: Current y position (m)
                - theta: Current heading angle (rad)
            path_x: Array of path x coordinates
            path_y: Array of path y coordinates
            path_t: Array of path time values
            current_time: Current timestamp (seconds)

        Returns:
            Tuple of (v_cmd, omega_cmd) where:
                v_cmd: Linear velocity command (m/s)
                omega_cmd: Angular velocity command (rad/s)
        """
        # Extract state components
        current_x = state["x"]
        current_y = state["y"]
        current_theta = state["theta"]

        # Initialize start time on first call
        if self.start_time is None:
            self.start_time = current_time

        # Compute elapsed time (trajectory time)
        elapsed_time = current_time - self.start_time

        # Find reference point based on elapsed time (where we SHOULD be)
        ref_idx = self.find_time_index(path_t, elapsed_time)

        # Compute reference velocity from path at reference time
        v_cmd = self.compute_reference_velocity(path_x, path_y, path_t, ref_idx)

        # Update lookahead distance based on reference velocity (adaptive)
        self.lookahead_distance = self.compute_adaptive_lookahead(v_cmd)

        # Find lookahead point ahead of reference point
        lookahead_x, lookahead_y, lookahead_idx = self.find_lookahead_point(
            current_x, current_y, path_x, path_y, ref_idx
        )

        # Compute angle to lookahead point in global frame
        angle_to_goal = math.atan2(lookahead_y - current_y, lookahead_x - current_x)

        # Compute angle error (alpha) relative to robot heading
        alpha = angle_to_goal - current_theta
        # Normalize to [-pi, pi]
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # Pure pursuit: compute curvature to lookahead point
        # actual distance to lookahead point (may differ from target)
        actual_distance = math.sqrt((lookahead_x - current_x) ** 2 + (lookahead_y - current_y) ** 2)

        if actual_distance > 0.01:  # Avoid division by zero
            # Curvature formula: kappa = 2*sin(alpha)/L
            curvature = 2.0 * math.sin(alpha) / actual_distance
            omega_cmd = v_cmd * curvature
        else:
            # At goal, no rotation needed
            omega_cmd = 0.0

        return v_cmd, omega_cmd
