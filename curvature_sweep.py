#!/usr/bin/env python3
"""Curvature-Adaptive Sensor Fusion Parameter Sweep.

This script systematically tests different curvature scaling factors to optimize
the adaptive sensor fusion between GPS and IMU based on path curvature.

Parameters Tested:
- CURVATURE_HEADING_SCALE: Controls GPS vs IMU trust in turns
- CURVATURE_THRESHOLD_SCALE: Controls GPS outlier rejection relaxation in turns

Testing Strategy:
- Focused sweep on highest-impact parameters
- Heading scale: [3.0, 4.0, 5.0, 6.0, 7.0]
- Threshold scale: [0.5, 1.0, 1.5, 2.0]
- Bias scale fixed at 2.0
- 10 runs per configuration for statistical validity
- Total: 20 configurations × 10 runs = 200 runs (~1.5 hours)
"""

import argparse
import re
import shutil
import statistics
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

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
    params: Dict[str, float]
    scores: List[float]
    mean: float
    std_dev: float
    min_score: float
    max_score: float
    num_runs: int
    num_failures: int
    catastrophic_failures: int  # Scores >= 30m

    @property
    def is_valid(self) -> bool:
        """Configuration is valid if no failures and no catastrophic scores."""
        return self.num_failures == 0 and self.catastrophic_failures == 0

    def consistency_score(self) -> float:
        """Calculate consistency score (lower is better)."""
        return self.mean + 2 * self.std_dev

    def __str__(self) -> str:
        """Human-readable representation."""
        status = "VALID" if self.is_valid else "INVALID"
        return (
            f"{self.config_name:50s} | "
            f"Mean: {self.mean:6.2f}m | "
            f"Std: {self.std_dev:5.2f}m | "
            f"Range: [{self.min_score:5.2f}, {self.max_score:5.2f}] | "
            f"Cat.Fails: {self.catastrophic_failures:2d}/10 | "
            f"{status}"
        )


class CurvatureSweep:
    """Manages curvature parameter sweep testing."""

    def __init__(self):
        self.config_path = Path("wagon_control/config.py")
        self.backup_path = Path("wagon_control/config.py.backup")
        self.results: List[ConfigResult] = []

    def backup_config(self):
        """Backup the current config."""
        shutil.copy2(self.config_path, self.backup_path)
        print(f"✓ Backed up config to {self.backup_path}")

    def restore_config(self):
        """Restore the original config."""
        if self.backup_path.exists():
            shutil.copy2(self.backup_path, self.config_path)
            self.backup_path.unlink()
            print("✓ Restored original config")

    def modify_config(self, parameter: str, value: float):
        """Modify a parameter in config.py."""
        with open(self.config_path, "r") as f:
            content = f.read()

        # Match: PARAMETER_NAME = value
        pattern = rf"^({parameter}\s*=\s*)[\d.]+(.*)$"
        replacement = rf"\g<1>{value}\g<2>"

        new_content = re.sub(pattern, replacement, content, flags=re.MULTILINE)

        if new_content == content:
            print(f"⚠ Warning: Parameter {parameter} not found in config!")
            return

        with open(self.config_path, "w") as f:
            f.write(new_content)

    def run_single_test(self) -> TestResult:
        """Run a single test and extract the score."""
        try:
            result = subprocess.run(
                [sys.executable, "-m", "wagon_control.client"],
                capture_output=True,
                text=True,
                timeout=60,
            )

            # Combine output and remove ANSI codes
            combined_output = result.stdout + "\n" + result.stderr
            ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
            clean_output = ansi_escape.sub('', combined_output)

            # Extract score
            score_pattern = r"Received score:\s+([\d.]+)m"
            match = re.search(score_pattern, clean_output)

            if match:
                score = float(match.group(1))
                return TestResult(score=score, success=True)
            else:
                return TestResult(
                    score=float("inf"),
                    success=False,
                    error_message="No score in output"
                )

        except subprocess.TimeoutExpired:
            return TestResult(
                score=float("inf"),
                success=False,
                error_message="Timeout"
            )
        except Exception as e:
            return TestResult(
                score=float("inf"),
                success=False,
                error_message=str(e)
            )

    def test_configuration(
        self,
        heading_scale: float,
        threshold_scale: float,
        bias_scale: float = 2.0,
        num_runs: int = 10
    ) -> ConfigResult:
        """Test a specific configuration."""
        config_name = f"H{heading_scale:.1f}_T{threshold_scale:.1f}_B{bias_scale:.1f}"

        print(f"\n{'='*80}")
        print(f"Testing: {config_name}")
        print(f"  Heading Scale:   {heading_scale}")
        print(f"  Threshold Scale: {threshold_scale}")
        print(f"  Bias Scale:      {bias_scale}")
        print(f"{'='*80}")

        # Apply configuration
        self.modify_config("CURVATURE_HEADING_SCALE", heading_scale)
        self.modify_config("CURVATURE_THRESHOLD_SCALE", threshold_scale)
        self.modify_config("CURVATURE_BIAS_SCALE", bias_scale)

        # Run tests
        scores = []
        failures = 0

        for i in range(num_runs):
            print(f"  Run {i+1}/{num_runs}...", end=" ", flush=True)
            result = self.run_single_test()

            if result.success:
                scores.append(result.score)
                print(f"{result.score:.2f}m")
            else:
                failures += 1
                print(f"FAILED ({result.error_message})")

        # Calculate statistics
        if scores:
            mean = statistics.mean(scores)
            std_dev = statistics.stdev(scores) if len(scores) > 1 else 0.0
            min_score = min(scores)
            max_score = max(scores)
            catastrophic = sum(1 for s in scores if s >= 30.0)
        else:
            mean = float("inf")
            std_dev = float("inf")
            min_score = float("inf")
            max_score = float("inf")
            catastrophic = 0

        config_result = ConfigResult(
            config_name=config_name,
            params={
                "heading_scale": heading_scale,
                "threshold_scale": threshold_scale,
                "bias_scale": bias_scale,
            },
            scores=scores,
            mean=mean,
            std_dev=std_dev,
            min_score=min_score,
            max_score=max_score,
            num_runs=num_runs,
            num_failures=failures,
            catastrophic_failures=catastrophic,
        )

        print(f"\n  {config_result}")

        return config_result

    def run_sweep(self, num_runs: int = 10):
        """Run the full parameter sweep."""
        # Define parameter ranges
        heading_scales = [3.0, 4.0, 5.0, 6.0, 7.0]
        threshold_scales = [0.5, 1.0, 1.5, 2.0]
        bias_scale = 2.0  # Fixed

        total_configs = len(heading_scales) * len(threshold_scales)

        print(f"\n{'='*80}")
        print("CURVATURE-ADAPTIVE SENSOR FUSION PARAMETER SWEEP")
        print(f"{'='*80}")
        print(f"Heading scales:   {heading_scales}")
        print(f"Threshold scales: {threshold_scales}")
        print(f"Bias scale:       {bias_scale} (fixed)")
        print(f"Runs per config:  {num_runs}")
        print(f"Total configs:    {total_configs}")
        print(f"Total runs:       {total_configs * num_runs}")
        print(f"{'='*80}\n")

        # Backup config
        self.backup_config()

        try:
            # Test all combinations
            config_num = 0
            for heading_scale in heading_scales:
                for threshold_scale in threshold_scales:
                    config_num += 1
                    print(f"\n[Config {config_num}/{total_configs}]")

                    result = self.test_configuration(
                        heading_scale=heading_scale,
                        threshold_scale=threshold_scale,
                        bias_scale=bias_scale,
                        num_runs=num_runs
                    )

                    self.results.append(result)

        finally:
            # Always restore config
            self.restore_config()

    def print_summary(self):
        """Print summary of all results."""
        print(f"\n{'='*80}")
        print("SWEEP RESULTS SUMMARY")
        print(f"{'='*80}\n")

        # Separate valid and invalid configs
        valid_results = [r for r in self.results if r.is_valid]
        invalid_results = [r for r in self.results if not r.is_valid]

        print(f"Valid configurations:   {len(valid_results)}/{len(self.results)}")
        print(f"Invalid configurations: {len(invalid_results)}/{len(self.results)}\n")

        if valid_results:
            # Sort by consistency score
            sorted_results = sorted(valid_results, key=lambda r: r.consistency_score())

            print("TOP 10 CONFIGURATIONS (by consistency score):")
            print("-" * 80)
            for i, result in enumerate(sorted_results[:10], 1):
                cons_score = result.consistency_score()
                print(f"{i:2d}. {result} | Consistency: {cons_score:.2f}")

            print(f"\n{'='*80}")
            print("BEST CONFIGURATION:")
            print(f"{'='*80}")
            best = sorted_results[0]
            print(f"Config: {best.config_name}")
            print(f"  Heading Scale:   {best.params['heading_scale']}")
            print(f"  Threshold Scale: {best.params['threshold_scale']}")
            print(f"  Bias Scale:      {best.params['bias_scale']}")
            print(f"\nPerformance:")
            print(f"  Mean:        {best.mean:.2f}m")
            print(f"  Std Dev:     {best.std_dev:.2f}m")
            print(f"  Range:       {best.min_score:.2f}m - {best.max_score:.2f}m")
            print(f"  Consistency: {best.consistency_score():.2f}")
            print(f"{'='*80}\n")

        if invalid_results:
            print("INVALID CONFIGURATIONS:")
            print("-" * 80)
            for result in invalid_results:
                print(f"  {result}")
            print()

    def save_results(self):
        """Save results to CSV file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"results/curvature_sweep_{timestamp}.csv"

        Path("results").mkdir(exist_ok=True)

        with open(filename, "w") as f:
            # Header
            f.write("config_name,heading_scale,threshold_scale,bias_scale,")
            f.write("mean,std_dev,min,max,num_runs,failures,catastrophic,")
            f.write("is_valid,consistency_score\n")

            # Data rows
            for result in self.results:
                f.write(f"{result.config_name},")
                f.write(f"{result.params['heading_scale']},")
                f.write(f"{result.params['threshold_scale']},")
                f.write(f"{result.params['bias_scale']},")
                f.write(f"{result.mean:.2f},")
                f.write(f"{result.std_dev:.2f},")
                f.write(f"{result.min_score:.2f},")
                f.write(f"{result.max_score:.2f},")
                f.write(f"{result.num_runs},")
                f.write(f"{result.num_failures},")
                f.write(f"{result.catastrophic_failures},")
                f.write(f"{result.is_valid},")
                f.write(f"{result.consistency_score():.2f}\n")

        print(f"✓ Saved results to {filename}")


def main():
    parser = argparse.ArgumentParser(description="Curvature parameter sweep")
    parser.add_argument(
        "--num-runs",
        type=int,
        default=10,
        help="Number of runs per configuration (default: 10)"
    )
    args = parser.parse_args()

    sweep = CurvatureSweep()
    sweep.run_sweep(num_runs=args.num_runs)
    sweep.print_summary()
    sweep.save_results()


if __name__ == "__main__":
    main()
