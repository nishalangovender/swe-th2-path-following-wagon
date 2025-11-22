"""
Component isolation modes for modular testing.

This module defines which control system components are active/bypassed
to enable systematic evaluation of each component's contribution.
"""

from dataclasses import dataclass
import argparse
import sys


@dataclass
class ComponentMode:
    """Configuration for which control components are active."""

    # State Estimation Layer
    use_ekf: bool = True  # If False, use raw GPS + dead reckoning

    # Path Following Layer
    use_pure_pursuit: bool = True  # If False, use reference velocities from path
    use_temporal_feedback: bool = True  # If False, no velocity adjustment for timing

    # Motor Control Layer
    use_pid: bool = True  # If False, pass through reference velocities
    use_feedforward: bool = True  # If False, disable feedforward terms
    use_integral: bool = True  # If False, disable integral terms
    use_derivative: bool = True  # If False, disable derivative terms

    def __str__(self):
        """Human-readable description of active components."""
        components = []

        # State Estimation
        if self.use_ekf:
            components.append("EKF")
        else:
            components.append("Raw GPS+IMU")

        # Path Following
        if self.use_pure_pursuit:
            pp_mode = "Pure Pursuit"
            if self.use_temporal_feedback:
                pp_mode += " + Temporal"
            components.append(pp_mode)
        else:
            components.append("Reference Tracking")

        # Motor Control
        if self.use_pid:
            pid_terms = []
            if True:  # P is always on when PID is enabled
                pid_terms.append("P")
            if self.use_integral:
                pid_terms.append("I")
            if self.use_derivative:
                pid_terms.append("D")
            if self.use_feedforward:
                pid_terms.append("FF")

            if pid_terms:
                components.append(f"Motor({'+'.join(pid_terms)})")
            else:
                components.append("Motor(Bypass)")
        else:
            components.append("Motor(Bypass)")

        return " → ".join(components)

    def to_dict(self):
        """Convert to dictionary for logging."""
        return {
            'use_ekf': self.use_ekf,
            'use_pure_pursuit': self.use_pure_pursuit,
            'use_temporal_feedback': self.use_temporal_feedback,
            'use_pid': self.use_pid,
            'use_feedforward': self.use_feedforward,
            'use_integral': self.use_integral,
            'use_derivative': self.use_derivative,
        }


def parse_component_flags(args=None):
    """
    Parse command-line flags to determine which components are active.

    Args:
        args: List of command-line arguments (default: sys.argv[1:])

    Returns:
        tuple: (ComponentMode, remaining_args)
            - ComponentMode with appropriate settings
            - List of remaining arguments not consumed
    """
    parser = argparse.ArgumentParser(add_help=False)

    # Component bypass flags
    parser.add_argument('--no-ekf', action='store_true',
                        help='Bypass EKF state estimation (use raw GPS + dead reckoning)')
    parser.add_argument('--no-pure-pursuit', action='store_true',
                        help='Bypass Pure Pursuit path following (use reference velocities)')
    parser.add_argument('--no-temporal', action='store_true',
                        help='Disable temporal feedback correction')
    parser.add_argument('--no-pid', action='store_true',
                        help='Bypass PID controller (pass through reference velocities)')
    parser.add_argument('--no-feedforward', action='store_true',
                        help='Disable feedforward terms in motor controller')
    parser.add_argument('--no-integral', action='store_true',
                        help='Disable integral terms in motor controller')
    parser.add_argument('--no-derivative', action='store_true',
                        help='Disable derivative terms in motor controller')

    # Parse known args, keep the rest
    if args is None:
        args = sys.argv[1:]

    known_args, remaining_args = parser.parse_known_args(args)

    # Create ComponentMode from flags
    mode = ComponentMode(
        use_ekf=not known_args.no_ekf,
        use_pure_pursuit=not known_args.no_pure_pursuit,
        use_temporal_feedback=not known_args.no_temporal,
        use_pid=not known_args.no_pid,
        use_feedforward=not known_args.no_feedforward,
        use_integral=not known_args.no_integral,
        use_derivative=not known_args.no_derivative,
    )

    return mode, remaining_args


if __name__ == '__main__':
    # Test the parsing
    test_cases = [
        [],
        ['--no-ekf'],
        ['--no-temporal', '--no-feedforward'],
        ['--no-pid'],
        ['--no-ekf', '--no-temporal', '--no-integral'],
    ]

    for test_args in test_cases:
        mode, remaining = parse_component_flags(test_args)
        print(f"Args: {test_args}")
        print(f"Mode: {mode}")
        print(f"Config: {mode.to_dict()}")
        print()
