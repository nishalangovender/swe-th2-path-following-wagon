"""Statistical analysis utilities for sweep results.

This module provides statistical computation and reporting for parameter sweep
results, including:
- Descriptive statistics (mean, std, median, quartiles)
- Confidence intervals
- Result ranking and comparison
- Statistical report generation
"""

import csv
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class Statistics:
    """Statistical summary for a dataset."""

    mean: float
    std: float
    median: float
    min: float
    max: float
    q1: float  # 25th percentile
    q3: float  # 75th percentile
    iqr: float  # Interquartile range
    ci_95_lower: float  # 95% confidence interval lower bound
    ci_95_upper: float  # 95% confidence interval upper bound
    n: int  # Sample size

    def __str__(self) -> str:
        """Human-readable summary."""
        return (
            f"Mean: {self.mean:.2f} ± {self.std:.2f} | "
            f"Median: {self.median:.2f} | "
            f"Range: [{self.min:.2f}, {self.max:.2f}] | "
            f"IQR: {self.iqr:.2f} | "
            f"95% CI: [{self.ci_95_lower:.2f}, {self.ci_95_upper:.2f}] | "
            f"N={self.n}"
        )


def compute_statistics(data: List[float]) -> Optional[Statistics]:
    """Compute comprehensive statistics for a dataset.

    Args:
        data: List of numeric values

    Returns:
        Statistics object, or None if data is empty
    """
    if not data:
        return None

    # Remove any infinite values
    finite_data = [x for x in data if math.isfinite(x)]

    if not finite_data:
        return None

    n = len(finite_data)
    sorted_data = sorted(finite_data)

    # Basic statistics
    mean = statistics.mean(finite_data)
    std = statistics.stdev(finite_data) if n > 1 else 0.0
    median = statistics.median(finite_data)
    min_val = min(finite_data)
    max_val = max(finite_data)

    # Quartiles
    q1 = np.percentile(finite_data, 25)
    q3 = np.percentile(finite_data, 75)
    iqr = q3 - q1

    # 95% Confidence interval for the mean
    # Using t-distribution for small samples
    if n > 1:
        se = std / math.sqrt(n)  # Standard error
        # For 95% CI with t-distribution, use ~1.96 for large n
        # For small n, this is approximate (should use t-table)
        t_critical = 1.96 if n > 30 else 2.0 + (30 - n) * 0.05
        ci_95_lower = mean - t_critical * se
        ci_95_upper = mean + t_critical * se
    else:
        ci_95_lower = ci_95_upper = mean

    return Statistics(
        mean=mean,
        std=std,
        median=median,
        min=min_val,
        max=max_val,
        q1=q1,
        q3=q3,
        iqr=iqr,
        ci_95_lower=ci_95_lower,
        ci_95_upper=ci_95_upper,
        n=n,
    )


def load_sweep_results(csv_path: Path) -> List[Dict]:
    """Load sweep results from CSV file.

    Args:
        csv_path: Path to CSV file from parameter sweep

    Returns:
        List of result dictionaries
    """
    results = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Parse all_scores field
            scores_str = row.get('all_scores', '')
            if scores_str:
                scores = [float(s) for s in scores_str.split(';') if s]
            else:
                scores = []

            result = {
                'config_name': row['config_name'],
                'param_names': row.get('param_names', '').split(';'),
                'param_values': row.get('param_values', '').split(';'),
                'mean': float(row['mean']),
                'std_dev': float(row['std_dev']),
                'median': float(row.get('median', row['mean'])),
                'min': float(row['min']),
                'max': float(row['max']),
                'q1': float(row.get('q1', 0)),
                'q3': float(row.get('q3', 0)),
                'iqr': float(row.get('iqr', 0)),
                'num_runs': int(row['num_runs']),
                'num_failures': int(row['num_failures']),
                'is_valid': row['is_valid'].lower() == 'true',
                'consistency_score': float(row.get('consistency_score', 0)),
                'scores': scores,
            }
            results.append(result)

    return results


def rank_configurations(
    results: List[Dict],
    metric: str = 'consistency_score',
    ascending: bool = True
) -> List[Dict]:
    """Rank configurations by a specified metric.

    Args:
        results: List of result dictionaries
        metric: Metric to rank by ('mean', 'std_dev', 'consistency_score', etc.)
        ascending: True for ascending order (lower is better), False for descending

    Returns:
        Sorted list of results
    """
    valid_results = [r for r in results if r['is_valid']]

    return sorted(
        valid_results,
        key=lambda r: r[metric],
        reverse=not ascending
    )


def compare_configurations(
    config1: Dict,
    config2: Dict
) -> Dict[str, float]:
    """Compare two configurations statistically.

    Args:
        config1: First configuration result dict
        config2: Second configuration result dict

    Returns:
        Dictionary with comparison metrics
    """
    scores1 = config1['scores']
    scores2 = config2['scores']

    comparison = {
        'mean_diff': config1['mean'] - config2['mean'],
        'median_diff': config1['median'] - config2['median'],
        'std_ratio': config1['std_dev'] / config2['std_dev'] if config2['std_dev'] > 0 else float('inf'),
        'improvement_pct': ((config2['mean'] - config1['mean']) / config2['mean'] * 100) if config2['mean'] > 0 else 0,
    }

    # Simple significance test (t-test approximation)
    if len(scores1) > 1 and len(scores2) > 1:
        n1, n2 = len(scores1), len(scores2)
        s1, s2 = config1['std_dev'], config2['std_dev']
        m1, m2 = config1['mean'], config2['mean']

        # Pooled standard error
        se = math.sqrt((s1**2 / n1) + (s2**2 / n2))

        # T-statistic
        if se > 0:
            t_stat = abs(m1 - m2) / se
            comparison['t_statistic'] = t_stat
            # Rough significance: |t| > 2 suggests difference is significant
            comparison['likely_significant'] = t_stat > 2.0
        else:
            comparison['t_statistic'] = 0.0
            comparison['likely_significant'] = False
    else:
        comparison['t_statistic'] = None
        comparison['likely_significant'] = None

    return comparison


def generate_report(
    csv_path: Path,
    output_path: Optional[Path] = None,
    top_n: int = 10
) -> str:
    """Generate a comprehensive statistical report from sweep results.

    Args:
        csv_path: Path to CSV file from parameter sweep
        output_path: Optional path to save report text file
        top_n: Number of top configurations to include

    Returns:
        Report text as string
    """
    results = load_sweep_results(csv_path)

    report_lines = []
    report_lines.append("=" * 80)
    report_lines.append("STATISTICAL ANALYSIS REPORT")
    report_lines.append("=" * 80)
    report_lines.append(f"\nSource: {csv_path}")
    report_lines.append(f"Total configurations: {len(results)}")

    # Filter valid results
    valid_results = [r for r in results if r['is_valid']]
    invalid_results = [r for r in results if not r['is_valid']]

    report_lines.append(f"Valid configurations: {len(valid_results)}")
    report_lines.append(f"Invalid configurations: {len(invalid_results)}")

    if not valid_results:
        report_lines.append("\nNo valid configurations found!")
        report_text = "\n".join(report_lines)
        if output_path:
            output_path.write_text(report_text)
        return report_text

    # Overall statistics across all valid configurations
    report_lines.append("\n" + "=" * 80)
    report_lines.append("OVERALL STATISTICS (all valid configurations)")
    report_lines.append("=" * 80)

    all_scores = []
    for result in valid_results:
        all_scores.extend(result['scores'])

    overall_stats = compute_statistics(all_scores)
    if overall_stats:
        report_lines.append(f"\n{overall_stats}")

    # Rank by different metrics
    report_lines.append("\n" + "=" * 80)
    report_lines.append(f"TOP {top_n} CONFIGURATIONS BY CONSISTENCY SCORE")
    report_lines.append("=" * 80)

    ranked_by_consistency = rank_configurations(
        results,
        metric='consistency_score',
        ascending=True
    )

    for i, result in enumerate(ranked_by_consistency[:top_n], 1):
        report_lines.append(f"\n{i}. {result['config_name']}")
        report_lines.append(f"   Parameters: {dict(zip(result['param_names'], result['param_values']))}")
        stats = compute_statistics(result['scores'])
        if stats:
            report_lines.append(f"   {stats}")
        report_lines.append(f"   Consistency Score: {result['consistency_score']:.2f}")

    # Rank by mean
    report_lines.append("\n" + "=" * 80)
    report_lines.append(f"TOP {top_n} CONFIGURATIONS BY MEAN SCORE")
    report_lines.append("=" * 80)

    ranked_by_mean = rank_configurations(results, metric='mean', ascending=True)

    for i, result in enumerate(ranked_by_mean[:top_n], 1):
        report_lines.append(f"\n{i}. {result['config_name']}")
        report_lines.append(f"   Mean: {result['mean']:.2f}m ± {result['std_dev']:.2f}m")
        report_lines.append(f"   Median: {result['median']:.2f}m")
        report_lines.append(f"   Range: [{result['min']:.2f}, {result['max']:.2f}]m")
        report_lines.append(f"   IQR: {result['iqr']:.2f}m")

    # Rank by stability (lowest std dev)
    report_lines.append("\n" + "=" * 80)
    report_lines.append(f"TOP {top_n} MOST STABLE CONFIGURATIONS (lowest std dev)")
    report_lines.append("=" * 80)

    ranked_by_stability = rank_configurations(
        results,
        metric='std_dev',
        ascending=True
    )

    for i, result in enumerate(ranked_by_stability[:top_n], 1):
        report_lines.append(f"\n{i}. {result['config_name']}")
        report_lines.append(f"   Std Dev: {result['std_dev']:.2f}m")
        report_lines.append(f"   Mean: {result['mean']:.2f}m")
        report_lines.append(f"   IQR: {result['iqr']:.2f}m")

    # Best vs Baseline comparison (if baseline exists)
    baseline = None
    for result in results:
        if 'baseline' in result['config_name'].lower():
            baseline = result
            break

    if baseline and ranked_by_consistency:
        best = ranked_by_consistency[0]

        report_lines.append("\n" + "=" * 80)
        report_lines.append("BEST VS BASELINE COMPARISON")
        report_lines.append("=" * 80)

        comparison = compare_configurations(best, baseline)

        report_lines.append(f"\nBest: {best['config_name']}")
        report_lines.append(f"  Mean: {best['mean']:.2f}m ± {best['std_dev']:.2f}m")

        report_lines.append(f"\nBaseline: {baseline['config_name']}")
        report_lines.append(f"  Mean: {baseline['mean']:.2f}m ± {baseline['std_dev']:.2f}m")

        report_lines.append(f"\nImprovement: {comparison['improvement_pct']:.1f}%")
        report_lines.append(f"Mean difference: {comparison['mean_diff']:.2f}m")
        report_lines.append(f"Median difference: {comparison['median_diff']:.2f}m")

        if comparison['likely_significant'] is not None:
            sig_str = "YES" if comparison['likely_significant'] else "NO"
            report_lines.append(f"Likely significant: {sig_str} (t={comparison['t_statistic']:.2f})")

    report_lines.append("\n" + "=" * 80)

    report_text = "\n".join(report_lines)

    if output_path:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(report_text)
        print(f"✓ Report saved to {output_path}")

    return report_text
