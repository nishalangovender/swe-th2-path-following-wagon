"""Reference path definition for wagon path following.

This module defines the Lemniscate of Gerono reference path that the wagon
should follow during the assignment.
"""

import numpy as np
import numpy.typing as npt


def compute_k(t: float) -> float:
    """Compute the path parameter k based on time t.

    The parameter k determines position along the Lemniscate of Gerono curve.

    Args:
        t: Time in seconds

    Returns:
        Path parameter k in radians
    """
    k = np.pi * t / 10.0 - np.pi / 2.0 if t < 20.0 else 3.0 * np.pi / 2.0
    return k


def reference_position(t: float) -> tuple[float, float]:
    """Compute reference position (x, y) for the Lemniscate of Gerono at time t.

    The Lemniscate of Gerono is a figure-eight curve defined by:
        x = -2 * sin(k) * cos(k)
        y = 2 * (sin(k) + 1)

    where k is the path parameter computed from time t.

    Args:
        t: Time in seconds

    Returns:
        Tuple of (x_ref, y_ref) in meters
    """
    k = compute_k(t)
    x_ref = -2.0 * np.sin(k) * np.cos(k)
    y_ref = 2.0 * (np.sin(k) + 1.0)
    return x_ref, y_ref


def reference_heading(t: float) -> float:
    """Compute reference heading angle for the path at time t.

    The heading is the direction of motion along the path, computed from
    the derivatives of the parametric equations:
        theta_ref = atan2(dy/dt, dx/dt)

    Args:
        t: Time in seconds

    Returns:
        Reference heading angle in radians (angle from +x axis)
    """
    k = compute_k(t)

    # Compute dk/dt based on time
    if t < 20.0:
        dk_dt = np.pi / 10.0
    else:
        dk_dt = 0.0  # Stationary after t=20

    # Compute path derivatives using chain rule
    # x = -2*sin(k)*cos(k) = -sin(2k), so dx/dk = -2*cos(2k)
    # y = 2*(sin(k) + 1), so dy/dk = 2*cos(k)
    dx_dk = -2.0 * np.cos(2.0 * k)
    dy_dk = 2.0 * np.cos(k)

    # Apply chain rule: d/dt = (d/dk) * (dk/dt)
    dx_dt = dx_dk * dk_dt
    dy_dt = dy_dk * dk_dt

    # Heading is direction of velocity vector
    theta_ref = np.arctan2(dy_dt, dx_dt)

    return float(theta_ref)


def reference_state(t: float) -> dict[str, float]:
    """Get complete reference state at time t.

    Convenience function that returns position and heading together.

    Args:
        t: Time in seconds

    Returns:
        Dictionary containing:
            'x': Reference x-coordinate (m)
            'y': Reference y-coordinate (m)
            'theta': Reference heading angle (rad)
    """
    x_ref, y_ref = reference_position(t)
    theta_ref = reference_heading(t)

    return {
        'x': float(x_ref),
        'y': float(y_ref),
        'theta': theta_ref
    }


def path_trajectory(t_max: float = 20.0, dt: float = 0.1) -> dict[str, npt.NDArray[np.float64]]:
    """Generate complete reference path trajectory for visualization.

    Computes the reference path positions at regular time intervals from t=0
    to t=t_max. This is useful for pre-computing the entire path for plotting.

    Args:
        t_max: Maximum time in seconds (default: 20.0)
        dt: Time step in seconds (default: 0.1)

    Returns:
        Dictionary containing:
            't': Time array in seconds
            'x': X position array in meters
            'y': Y position array in meters
    """
    t_array = np.arange(0.0, t_max + dt, dt)
    x_array = np.zeros_like(t_array)
    y_array = np.zeros_like(t_array)

    for i, t in enumerate(t_array):
        x_array[i], y_array[i] = reference_position(t)

    return {
        "t": t_array,
        "x": x_array,
        "y": y_array,
    }
