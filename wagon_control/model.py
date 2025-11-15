"""
Differential drive wagon kinematic model.

This module provides the inverse kinematics for a differential drive robot,
converting desired linear and angular velocities into individual wheel velocities.
"""

# Robot physical parameters
WHEELBASE = 0.5  # Distance between left and right wheels (meters)

# Velocity constraints
V_MIN = -2.0  # Minimum wheel velocity (m/s)
V_MAX = 2.0  # Maximum wheel velocity (m/s)

# Acceleration constraints
A_MIN = -1.0  # Minimum wheel acceleration (m/s²)
A_MAX = 1.0  # Maximum wheel acceleration (m/s²)


def inverse_kinematics(v_cmd: float, omega_cmd: float) -> tuple[float, float]:
    """
    Compute wheel velocities from desired linear and angular velocities.

    For a differential drive robot, the relationship between the robot's
    linear velocity (v), angular velocity (omega), and the individual
    wheel velocities is:
        v_left = v - (L/2) * omega
        v_right = v + (L/2) * omega

    where L is the wheelbase (distance between wheels).

    Args:
        v_cmd: Desired linear velocity of the robot center (m/s)
        omega_cmd: Desired angular velocity of the robot (rad/s)
                   Positive omega results in counter-clockwise rotation

    Returns:
        tuple[float, float]: (v_left, v_right) wheel velocities in m/s,
                            clamped to the range [V_MIN, V_MAX]

    Example:
        >>> v_left, v_right = inverse_kinematics(1.0, 0.5)
        >>> # Robot moves forward at 1 m/s while turning left
    """
    # Compute raw wheel velocities using differential drive kinematics
    v_left = v_cmd - (WHEELBASE / 2.0) * omega_cmd
    v_right = v_cmd + (WHEELBASE / 2.0) * omega_cmd

    # Clamp velocities to respect actuator limits
    v_left = max(V_MIN, min(V_MAX, v_left))
    v_right = max(V_MIN, min(V_MAX, v_right))

    return v_left, v_right
