#!/usr/bin/env python3
"""Parameter Sweep Testing for Wagon Control System.

This script systematically tests different parameter configurations to find
the optimal settings that provide both good average performance and consistency.

Testing Strategy:
- Runs 10 trials per configuration
- Measures mean error, standard deviation, min/max values
- Rejects configurations with outliers > 50m (catastrophic failures)
- Ranks by both average performance and consistency (low std dev)

Output:
- results/parameter_sweep_TIMESTAMP.csv: Detailed results for all configs
- results/parameter_sweep_TIMESTAMP_summary.txt: Human-readable summary
- Automatically restores original configuration after testing
"""

import asyncio
import json
import logging
import os
import re
import shutil
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import websockets


@dataclass
class TestResult:
    """Results from a single test run."""

    score: float
    success: bool
    error_message: Optional[str] = None


@dataclass
class ConfigResult:
    """Aggregated results for a parameter configuration."""

    config_name: str
    params: Dict[str, Any]
    scores: List[float]
    mean: float
    std_dev: float
    min_score: float
    max_score: float
    num_runs: int
    num_failures: int
    is_valid: bool  # False if any score > 50m or failures occurred

    def consistency_score(self) -> float:
        """Calculate a consistency score (lower is better).

        Combines mean error with variance penalty.
        """
        # Penalize both high mean and high variance
        # Good configs have low mean AND low std_dev
        return self.mean + 2 * self.std_dev

    def __str__(self) -> str:
        """Human-readable representation."""
        status = "VALID" if self.is_valid else "INVALID"
        return (
            f"{self.config_name:40s} | "
            f"Mean: {self.mean:6.2f}m | "
            f"Std: {self.std_dev:5.2f}m | "
            f"Range: [{self.min_score:5.2f}, {self.max_score:5.2f}] | "
            f"Failures: {self.num_failures:2d}/10 | "
            f"{status}"
        )


class ParameterTester:
    """Manages parameter sweep testing."""

    def __init__(self, num_runs: int = 10, verbose: bool = False):
        """Initialize the parameter tester.

        Args:
            num_runs: Number of test runs per configuration
            verbose: Enable verbose logging
        """
        self.num_runs = num_runs
        self.verbose = verbose
        self.config_path = Path("wagon_control/config.py")
        self.backup_path = Path("wagon_control/config.py.backup")
        self.results: List[ConfigResult] = []

        # Setup logging
        level = logging.DEBUG if verbose else logging.INFO
        logging.basicConfig(
            level=level,
            format="%(asctime)s - %(levelname)s - %(message)s",
            datefmt="%H:%M:%S",
        )
        self.logger = logging.getLogger(__name__)

    def backup_config(self) -> None:
        """Backup current configuration."""
        shutil.copy2(self.config_path, self.backup_path)
        self.logger.info(f"✓ Backed up config to {self.backup_path}")

    def restore_config(self) -> None:
        """Restore original configuration."""
        if self.backup_path.exists():
            shutil.copy2(self.backup_path, self.config_path)
            self.backup_path.unlink()
            self.logger.info("✓ Restored original config")

    def modify_config(self, parameter: str, value: Any) -> None:
        """Modify a parameter in config.py.

        Args:
            parameter: Parameter name (e.g., 'MOTOR_KP_V')
            value: New value to set
        """
        with open(self.config_path, "r") as f:
            content = f.read()

        # Find and replace the parameter value
        # Matches: PARAMETER_NAME = value (any numeric format)
        pattern = rf"^({parameter}\s*=\s*)[\d.]+(.*)$"
        replacement = rf"\g<1>{value}\g<2>"

        new_content = re.sub(pattern, replacement, content, flags=re.MULTILINE)

        if new_content == content:
            self.logger.warning(f"Parameter {parameter} not found in config!")
            return

        with open(self.config_path, "w") as f:
            f.write(new_content)

        self.logger.debug(f"Set {parameter} = {value}")

    def run_single_test(self) -> TestResult:
        """Run a single test and extract the score.

        Returns:
            TestResult with score and success status
        """
        try:
            # Run the client without verbose output, capture stdout and stderr
            env = os.environ.copy()
            env.pop("RUN_DIR", None)  # Disable live plotting

            result = subprocess.run(
                [sys.executable, "-m", "wagon_control.client"],
                env=env,
                capture_output=True,
                text=True,
                timeout=60,  # 60 second timeout
            )

            # Combine stdout and stderr for searching
            combined_output = result.stdout + "\n" + result.stderr

            # Strip ANSI escape codes that might interfere with pattern matching
            ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
            clean_output = ansi_escape.sub('', combined_output)

            # Extract score from output
            # Looking for: "✓ Received score: XX.XXm" or "Received score: XX.XXm"
            score_pattern = r"Received score:\s+([\d.]+)m"
            match = re.search(score_pattern, clean_output)

            if match:
                score = float(match.group(1))
                self.logger.debug(f"    Extracted score: {score:.2f}m")
                return TestResult(score=score, success=True)
            else:
                # Log last 500 chars for debugging
                self.logger.error(f"No score found in output.")
                self.logger.debug(f"Last 500 chars of output:\n{clean_output[-500:]}")
                return TestResult(
                    score=float("inf"), success=False, error_message="No score in output"
                )

        except subprocess.TimeoutExpired:
            self.logger.error("Test timed out after 60 seconds")
            return TestResult(
                score=float("inf"), success=False, error_message="Timeout"
            )
        except Exception as e:
            self.logger.error(f"Test failed with exception: {e}")
            return TestResult(score=float("inf"), success=False, error_message=str(e))

    def test_configuration(
        self, config_name: str, params: Dict[str, Any]
    ) -> ConfigResult:
        """Test a specific parameter configuration.

        Args:
            config_name: Human-readable name for this configuration
            params: Dictionary of parameter names to values

        Returns:
            ConfigResult with aggregated statistics
        """
        self.logger.info(f"\n{'='*70}")
        self.logger.info(f"Testing: {config_name}")
        self.logger.info(f"Parameters: {params}")
        self.logger.info(f"{'='*70}")

        # Apply configuration
        for param_name, param_value in params.items():
            self.modify_config(param_name, param_value)

        # Run tests
        scores = []
        failures = 0

        for i in range(self.num_runs):
            self.logger.info(f"  Run {i+1}/{self.num_runs}...")
            result = self.run_single_test()

            if result.success:
                scores.append(result.score)
                self.logger.info(f"    Score: {result.score:.2f}m")
            else:
                failures += 1
                self.logger.warning(f"    FAILED: {result.error_message}")

            # Brief pause between runs
            time.sleep(1)

        # Calculate statistics
        if not scores:
            # All runs failed
            return ConfigResult(
                config_name=config_name,
                params=params,
                scores=[],
                mean=float("inf"),
                std_dev=float("inf"),
                min_score=float("inf"),
                max_score=float("inf"),
                num_runs=self.num_runs,
                num_failures=failures,
                is_valid=False,
            )

        mean_score = statistics.mean(scores)
        std_dev = statistics.stdev(scores) if len(scores) > 1 else 0.0
        min_score = min(scores)
        max_score = max(scores)

        # Configuration is invalid if:
        # 1. Any score > 50m (catastrophic failure)
        # 2. Any runs failed
        is_valid = max_score <= 50.0 and failures == 0

        result = ConfigResult(
            config_name=config_name,
            params=params,
            scores=scores,
            mean=mean_score,
            std_dev=std_dev,
            min_score=min_score,
            max_score=max_score,
            num_runs=self.num_runs,
            num_failures=failures,
            is_valid=is_valid,
        )

        self.logger.info(f"\n  {result}")
        return result

    def run_parameter_sweep(self) -> None:
        """Run the full parameter sweep test suite."""
        self.logger.info("\n" + "=" * 70)
        self.logger.info("PARAMETER SWEEP TESTING")
        self.logger.info("=" * 70)
        self.logger.info(f"Runs per configuration: {self.num_runs}")
        self.logger.info(f"Outlier threshold: 50.0m")
        self.logger.info("=" * 70 + "\n")

        # Backup original config
        self.backup_config()

        try:
            # Define parameter ranges to test
            # We'll test each parameter independently (one-at-a-time)

            # 1. Localizer velocity correction gain
            for gain in [0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65]:
                self.results.append(
                    self.test_configuration(
                        f"VelCorrectionGain={gain}",
                        {"LOCALIZER_VELOCITY_CORRECTION_GAIN": gain},
                    )
                )

            # 2. Follower base lookahead
            for lookahead in [0.6, 0.7, 0.8, 0.9, 1.0]:
                self.results.append(
                    self.test_configuration(
                        f"BaseLookahead={lookahead}",
                        {"FOLLOWER_BASE_LOOKAHEAD": lookahead},
                    )
                )

            # 3. Follower lookahead time
            for time_val in [0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9]:
                self.results.append(
                    self.test_configuration(
                        f"LookaheadTime={time_val}",
                        {"FOLLOWER_LOOKAHEAD_TIME": time_val},
                    )
                )

            # 4. Motor KI_V (integral gain for linear velocity)
            for ki_v in [0.05, 0.06, 0.08, 0.10, 0.12, 0.15]:
                self.results.append(
                    self.test_configuration(
                        f"KI_V={ki_v}", {"MOTOR_KI_V": ki_v}
                    )
                )

            # 5. Motor KI_OMEGA (integral gain for angular velocity)
            for ki_omega in [0.04, 0.06, 0.08, 0.10, 0.12]:
                self.results.append(
                    self.test_configuration(
                        f"KI_OMEGA={ki_omega}", {"MOTOR_KI_OMEGA": ki_omega}
                    )
                )

            # 6. Test baseline configuration for comparison
            self.results.append(
                self.test_configuration(
                    "Baseline (ce8b3ef)",
                    {
                        "LOCALIZER_VELOCITY_CORRECTION_GAIN": 0.5,
                        "FOLLOWER_BASE_LOOKAHEAD": 0.7,
                        "FOLLOWER_LOOKAHEAD_TIME": 0.8,
                        "MOTOR_KI_V": 0.1,
                        "MOTOR_KI_OMEGA": 0.1,
                    },
                )
            )

            # 7. Test current configuration for comparison
            self.results.append(
                self.test_configuration(
                    "Current Config",
                    {
                        "LOCALIZER_VELOCITY_CORRECTION_GAIN": 0.45,
                        "FOLLOWER_BASE_LOOKAHEAD": 0.8,
                        "FOLLOWER_LOOKAHEAD_TIME": 0.7,
                        "MOTOR_KI_V": 0.08,
                        "MOTOR_KI_OMEGA": 0.06,
                    },
                )
            )

        finally:
            # Always restore original config
            self.restore_config()

    def save_results(self) -> Tuple[Path, Path]:
        """Save results to CSV and summary text files.

        Returns:
            Tuple of (csv_path, summary_path)
        """
        # Create results directory
        results_dir = Path("results")
        results_dir.mkdir(exist_ok=True)

        # Generate timestamp for filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Save CSV
        csv_path = results_dir / f"parameter_sweep_{timestamp}.csv"
        with open(csv_path, "w") as f:
            # Header
            f.write(
                "config_name,param_names,param_values,mean,std_dev,min,max,"
                "num_runs,num_failures,is_valid,consistency_score,all_scores\n"
            )

            # Data rows
            for result in self.results:
                param_names = ";".join(result.params.keys())
                param_values = ";".join(str(v) for v in result.params.values())
                scores_str = ";".join(f"{s:.2f}" for s in result.scores)

                f.write(
                    f"{result.config_name},"
                    f"{param_names},"
                    f"{param_values},"
                    f"{result.mean:.4f},"
                    f"{result.std_dev:.4f},"
                    f"{result.min_score:.4f},"
                    f"{result.max_score:.4f},"
                    f"{result.num_runs},"
                    f"{result.num_failures},"
                    f"{result.is_valid},"
                    f"{result.consistency_score():.4f},"
                    f"{scores_str}\n"
                )

        # Save summary
        summary_path = results_dir / f"parameter_sweep_{timestamp}_summary.txt"
        with open(summary_path, "w") as f:
            f.write("=" * 80 + "\n")
            f.write("PARAMETER SWEEP RESULTS SUMMARY\n")
            f.write("=" * 80 + "\n\n")
            f.write(f"Test Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Runs per configuration: {self.num_runs}\n")
            f.write(f"Total configurations tested: {len(self.results)}\n\n")

            # Filter valid results
            valid_results = [r for r in self.results if r.is_valid]
            invalid_results = [r for r in self.results if not r.is_valid]

            f.write(f"Valid configurations: {len(valid_results)}\n")
            f.write(f"Invalid configurations: {len(invalid_results)}\n\n")

            # Rank valid results by consistency score
            ranked = sorted(valid_results, key=lambda r: r.consistency_score())

            f.write("=" * 80 + "\n")
            f.write("TOP 10 CONFIGURATIONS (by consistency score = mean + 2*std_dev)\n")
            f.write("=" * 80 + "\n\n")

            for i, result in enumerate(ranked[:10], 1):
                f.write(f"{i:2d}. {result}\n")
                f.write(f"    Consistency Score: {result.consistency_score():.2f}\n")
                f.write(f"    Parameters: {result.params}\n\n")

            f.write("\n" + "=" * 80 + "\n")
            f.write("ALL VALID CONFIGURATIONS (sorted by mean score)\n")
            f.write("=" * 80 + "\n\n")

            ranked_by_mean = sorted(valid_results, key=lambda r: r.mean)
            for result in ranked_by_mean:
                f.write(f"{result}\n")

            if invalid_results:
                f.write("\n" + "=" * 80 + "\n")
                f.write("INVALID CONFIGURATIONS (outliers > 50m or failures)\n")
                f.write("=" * 80 + "\n\n")

                for result in invalid_results:
                    f.write(f"{result}\n")

        return csv_path, summary_path

    def print_summary(self) -> None:
        """Print a summary of results to console."""
        print("\n" + "=" * 70)
        print("PARAMETER SWEEP COMPLETE")
        print("=" * 70)

        valid_results = [r for r in self.results if r.is_valid]
        invalid_results = [r for r in self.results if not r.is_valid]

        print(f"\nTotal configurations tested: {len(self.results)}")
        print(f"Valid configurations: {len(valid_results)}")
        print(f"Invalid configurations: {len(invalid_results)}")

        if valid_results:
            print("\n" + "=" * 70)
            print("TOP 5 CONFIGURATIONS (by consistency)")
            print("=" * 70 + "\n")

            ranked = sorted(valid_results, key=lambda r: r.consistency_score())
            for i, result in enumerate(ranked[:5], 1):
                print(f"{i}. {result}")
                print(f"   Consistency Score: {result.consistency_score():.2f}")
                print(f"   Parameters: {result.params}\n")


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Parameter sweep testing for wagon control system"
    )
    parser.add_argument(
        "-n",
        "--num-runs",
        type=int,
        default=10,
        help="Number of runs per configuration (default: 10)",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Enable verbose logging"
    )

    args = parser.parse_args()

    # Create tester and run sweep
    tester = ParameterTester(num_runs=args.num_runs, verbose=args.verbose)

    try:
        tester.run_parameter_sweep()
        csv_path, summary_path = tester.save_results()
        tester.print_summary()

        print(f"\n✓ Results saved to:")
        print(f"  - {csv_path}")
        print(f"  - {summary_path}")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user. Restoring original config...")
        tester.restore_config()
        sys.exit(1)
    except Exception as e:
        print(f"\n\nError: {e}")
        tester.restore_config()
        raise


if __name__ == "__main__":
    main()
