"""Generic parameter sweep framework for wagon control system.

This module provides a flexible, reusable framework for testing any configuration
parameter combinations. It handles:
- Dynamic parameter modification in config.py
- Test execution and result collection
- Automatic config backup/restore
- Failure detection and validation
- CSV export of results

The framework is completely generic - it works with any config.py parameters
and can be used for systematic parameter optimization.
"""

import logging
import os
import re
import shutil
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


@dataclass
class TestResult:
    """Results from a single test run."""

    score: float
    success: bool
    error_message: Optional[str] = None

    @property
    def is_valid(self) -> bool:
        """Check if result is valid (successful with finite score)."""
        return self.success and self.score != float("inf")


@dataclass
class ConfigResult:
    """Aggregated results for a parameter configuration."""

    config_name: str
    params: Dict[str, Any]
    scores: List[float]
    mean: float
    std_dev: float
    median: float
    min_score: float
    max_score: float
    q1: float  # First quartile (25th percentile)
    q3: float  # Third quartile (75th percentile)
    num_runs: int
    num_failures: int
    invalid_threshold: float = 50.0

    @property
    def is_valid(self) -> bool:
        """Configuration is valid if no failures and all scores under threshold."""
        return (
            self.num_failures == 0
            and self.max_score <= self.invalid_threshold
            and len(self.scores) > 0
        )

    @property
    def iqr(self) -> float:
        """Interquartile range (Q3 - Q1)."""
        return self.q3 - self.q1

    def consistency_score(self) -> float:
        """Calculate consistency score (lower is better).

        Combines mean error with variance penalty.
        """
        return self.mean + 2 * self.std_dev

    def __str__(self) -> str:
        """Human-readable representation."""
        status = "VALID" if self.is_valid else "INVALID"
        return (
            f"{self.config_name:50s} | "
            f"Mean: {self.mean:6.2f}m | "
            f"Std: {self.std_dev:5.2f}m | "
            f"Median: {self.median:6.2f}m | "
            f"Range: [{self.min_score:5.2f}, {self.max_score:5.2f}] | "
            f"Failures: {self.num_failures:2d}/{self.num_runs} | "
            f"{status}"
        )


class ParameterSweep:
    """Generic parameter sweep testing framework.

    This class provides a flexible framework for testing arbitrary parameter
    configurations. It can sweep across any parameters defined in config.py.

    Example:
        sweep = ParameterSweep(
            parameters={
                'MOTOR_KP_V': [1.0, 1.5, 2.0],
                'FOLLOWER_BASE_LOOKAHEAD': [0.6, 0.8, 1.0]
            },
            runs_per_config=10,
            invalid_threshold=50.0
        )
        sweep.run()
        sweep.save_results('results/my_sweep.csv')
    """

    def __init__(
        self,
        parameters: Optional[Dict[str, List[Any]]] = None,
        runs_per_config: int = 10,
        invalid_threshold: float = 50.0,
        verbose: bool = False,
        config_path: Optional[Path] = None
    ):
        """Initialize parameter sweep.

        Args:
            parameters: Dict mapping parameter names to lists of values to test.
                       If None, must call add_parameter() or add_configuration().
            runs_per_config: Number of test runs per configuration
            invalid_threshold: Score threshold for marking configs as invalid (meters)
            verbose: Enable verbose logging
            config_path: Path to config.py (default: wagon_control/config.py)
        """
        self.parameters = parameters or {}
        self.runs_per_config = runs_per_config
        self.invalid_threshold = invalid_threshold
        self.verbose = verbose

        # Paths
        self.config_path = config_path or Path("wagon_control/config.py")
        self.backup_path = self.config_path.with_suffix('.py.backup')

        # Results storage
        self.results: List[ConfigResult] = []
        self.configurations: List[Tuple[str, Dict[str, Any]]] = []

        # Setup logging
        level = logging.DEBUG if verbose else logging.INFO
        logging.basicConfig(
            level=level,
            format="%(asctime)s - %(levelname)s - %(message)s",
            datefmt="%H:%M:%S",
        )
        self.logger = logging.getLogger(__name__)

    def add_parameter(self, name: str, values: List[Any]) -> None:
        """Add a parameter to sweep over.

        Args:
            name: Parameter name in config.py (e.g., 'MOTOR_KP_V')
            values: List of values to test for this parameter
        """
        self.parameters[name] = values

    def add_configuration(self, name: str, params: Dict[str, Any]) -> None:
        """Add a specific configuration to test.

        Use this to test specific parameter combinations rather than
        a full grid search.

        Args:
            name: Human-readable name for this configuration
            params: Dictionary of parameter names to values
        """
        self.configurations.append((name, params))

    def backup_config(self) -> None:
        """Backup current configuration."""
        shutil.copy2(self.config_path, self.backup_path)
        if self.verbose:
            self.logger.debug(f"Backed up config to {self.backup_path}")

    def restore_config(self) -> None:
        """Restore original configuration."""
        if self.backup_path.exists():
            shutil.copy2(self.backup_path, self.config_path)
            self.backup_path.unlink()
            if self.verbose:
                self.logger.debug("Restored original config")

    def modify_config(self, parameter: str, value: Any) -> bool:
        """Modify a parameter in config.py.

        Args:
            parameter: Parameter name (e.g., 'MOTOR_KP_V')
            value: New value to set

        Returns:
            True if parameter was found and modified, False otherwise
        """
        with open(self.config_path, "r") as f:
            content = f.read()

        # Match: PARAMETER_NAME = value
        # Captures everything from = to end of line (or # comment)
        pattern = rf"^({parameter}\s*=\s*)([^\n#]+)(.*?)$"

        # Format value appropriately
        if isinstance(value, str):
            value_str = f'"{value}"'
        else:
            value_str = str(value)

        replacement = rf"\g<1>{value_str}\g<3>"
        new_content = re.sub(pattern, replacement, content, flags=re.MULTILINE)

        if new_content == content:
            # Check if parameter exists but already has this value
            match = re.search(pattern, content, flags=re.MULTILINE)
            if match:
                current_value = match.group(2).strip()
                if current_value == value_str:
                    # Parameter already has the correct value
                    if self.verbose:
                        self.logger.debug(f"{parameter} already set to {value}")
                    return True

            # Parameter not found
            print(f"  ⚠ Warning: Parameter {parameter} not found in config!")
            if self.verbose:
                # Show what we're looking for
                print(f"  Searching for pattern: {pattern}")
                # Try to find similar parameter names
                similar = [line for line in content.split('\n') if parameter.lower() in line.lower()]
                if similar:
                    print(f"  Similar lines found:")
                    for line in similar[:3]:
                        print(f"    {line.strip()}")
            return False

        with open(self.config_path, "w") as f:
            f.write(new_content)

        if self.verbose:
            self.logger.debug(f"Set {parameter} = {value}")
        return True

    def run_single_test(self) -> TestResult:
        """Run a single test and extract the score.

        Returns:
            TestResult with score and success status
        """
        try:
            # Run the client without live plotting
            env = os.environ.copy()
            env.pop("RUN_DIR", None)  # Disable live plotting

            result = subprocess.run(
                [sys.executable, "-m", "wagon_control.client"],
                env=env,
                capture_output=True,
                text=True,
                timeout=60,
            )

            # Combine stdout and stderr
            combined_output = result.stdout + "\n" + result.stderr

            # Strip ANSI escape codes
            ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
            clean_output = ansi_escape.sub('', combined_output)

            # Extract score: "→ L2: XX.XXXm" or "L2: XX.XXXm"
            score_pattern = r"L2:\s+([\d.]+)m"
            match = re.search(score_pattern, clean_output)

            if match:
                score = float(match.group(1))
                self.logger.debug(f"    Score: {score:.2f}m")
                return TestResult(score=score, success=True)
            else:
                self.logger.error("No score found in output")
                if self.verbose:
                    self.logger.debug(f"Last 500 chars:\n{clean_output[-500:]}")
                return TestResult(
                    score=float("inf"),
                    success=False,
                    error_message="No score in output"
                )

        except subprocess.TimeoutExpired:
            self.logger.error("Test timed out after 60 seconds")
            return TestResult(
                score=float("inf"),
                success=False,
                error_message="Timeout"
            )
        except Exception as e:
            self.logger.error(f"Test failed: {e}")
            return TestResult(
                score=float("inf"),
                success=False,
                error_message=str(e)
            )

    def test_configuration(
        self,
        config_name: str,
        params: Dict[str, Any]
    ) -> ConfigResult:
        """Test a specific parameter configuration.

        Args:
            config_name: Human-readable name for this configuration
            params: Dictionary of parameter names to values

        Returns:
            ConfigResult with aggregated statistics
        """
        print(f"\n{'='*70}")
        print(f"Testing: {config_name}")
        param_str = ", ".join(f"{k}={v}" for k, v in params.items())
        print(f"Parameters: {param_str}")
        print(f"{'='*70}")

        # Apply configuration
        for param_name, param_value in params.items():
            self.modify_config(param_name, param_value)

        # Run tests
        scores = []
        failures = 0

        for i in range(self.runs_per_config):
            print(f"  Run {i+1}/{self.runs_per_config}...", end=" ", flush=True)
            result = self.run_single_test()

            if result.success:
                scores.append(result.score)
                print(f"{result.score:.3f}m")
            else:
                failures += 1
                print(f"FAILED ({result.error_message})")

            # Brief pause between runs
            time.sleep(1)

        # Calculate statistics
        if scores:
            mean = statistics.mean(scores)
            std_dev = statistics.stdev(scores) if len(scores) > 1 else 0.0
            median = statistics.median(scores)
            min_score = min(scores)
            max_score = max(scores)

            # Calculate quartiles
            sorted_scores = sorted(scores)
            n = len(sorted_scores)
            q1 = sorted_scores[n // 4] if n > 0 else 0.0
            q3 = sorted_scores[(3 * n) // 4] if n > 0 else 0.0
        else:
            # All runs failed
            mean = median = min_score = max_score = float("inf")
            std_dev = q1 = q3 = float("inf")

        result = ConfigResult(
            config_name=config_name,
            params=params,
            scores=scores,
            mean=mean,
            std_dev=std_dev,
            median=median,
            min_score=min_score,
            max_score=max_score,
            q1=q1,
            q3=q3,
            num_runs=self.runs_per_config,
            num_failures=failures,
            invalid_threshold=self.invalid_threshold,
        )

        # Print summary
        status = "✓ VALID" if result.is_valid else "✗ INVALID"
        print(f"\n  {status}")
        print(f"  Mean: {mean:.3f}m ± {std_dev:.3f}m")
        print(f"  Median: {median:.3f}m | Range: [{min_score:.3f}, {max_score:.3f}]m")
        if failures > 0:
            print(f"  Failures: {failures}/{self.runs_per_config}")

        return result

    def run(self) -> List[ConfigResult]:
        """Run the parameter sweep.

        Tests all configurations added via add_configuration() or
        generates configurations from parameters dict (one-at-a-time sweep).

        Returns:
            List of ConfigResult objects
        """
        print("\n" + "="*70)
        print("PARAMETER SWEEP")
        print("="*70)
        print(f"Runs per configuration: {self.runs_per_config}")
        print(f"Invalid threshold: {self.invalid_threshold}m")
        print("="*70)

        # Backup original config
        self.backup_config()

        try:
            # If explicit configurations were added, test those
            if self.configurations:
                for config_name, params in self.configurations:
                    result = self.test_configuration(config_name, params)
                    self.results.append(result)

            # Otherwise, generate one-at-a-time configurations from parameters
            elif self.parameters:
                for param_name, values in self.parameters.items():
                    for value in values:
                        config_name = f"{param_name}={value}"
                        params = {param_name: value}
                        result = self.test_configuration(config_name, params)
                        self.results.append(result)

            else:
                self.logger.error("No configurations or parameters specified!")
                raise ValueError(
                    "Must either add configurations via add_configuration() "
                    "or specify parameters dict"
                )

        finally:
            # Always restore original config
            self.restore_config()

        return self.results

    def save_results(self, filepath: Optional[Path] = None) -> Path:
        """Save results to CSV file.

        Args:
            filepath: Optional path for CSV file. If None, auto-generates
                     timestamped filename in results/ directory.

        Returns:
            Path to saved CSV file
        """
        if filepath is None:
            results_dir = Path("results")
            results_dir.mkdir(exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = results_dir / f"parameter_sweep_{timestamp}.csv"
        else:
            filepath = Path(filepath)
            filepath.parent.mkdir(parents=True, exist_ok=True)

        with open(filepath, "w") as f:
            # Header
            f.write(
                "config_name,param_names,param_values,"
                "mean,std_dev,median,min,max,q1,q3,iqr,"
                "num_runs,num_failures,is_valid,consistency_score,"
                "all_scores\n"
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
                    f"{result.median:.4f},"
                    f"{result.min_score:.4f},"
                    f"{result.max_score:.4f},"
                    f"{result.q1:.4f},"
                    f"{result.q3:.4f},"
                    f"{result.iqr:.4f},"
                    f"{result.num_runs},"
                    f"{result.num_failures},"
                    f"{result.is_valid},"
                    f"{result.consistency_score():.4f},"
                    f"{scores_str}\n"
                )

        print(f"\n✓ Results saved to {filepath}")
        return filepath

    def print_summary(self, top_n: int = 10) -> None:
        """Print summary of results to console.

        Args:
            top_n: Number of top configurations to display
        """
        print("\n" + "="*80)
        print("PARAMETER SWEEP COMPLETE")
        print("="*80)

        valid_results = [r for r in self.results if r.is_valid]
        invalid_results = [r for r in self.results if not r.is_valid]

        print(f"\nTotal configurations tested: {len(self.results)}")
        print(f"Valid configurations: {len(valid_results)}")
        print(f"Invalid configurations: {len(invalid_results)}")

        if valid_results:
            print("\n" + "="*80)
            print(f"TOP {min(top_n, len(valid_results))} CONFIGURATIONS (by consistency)")
            print("="*80 + "\n")

            ranked = sorted(valid_results, key=lambda r: r.consistency_score())
            for i, result in enumerate(ranked[:top_n], 1):
                print(f"{i}. {result}")
                print(f"   Consistency: {result.consistency_score():.2f}")
                print(f"   IQR: {result.iqr:.2f}m")
                print(f"   Parameters: {result.params}\n")
