"""Generic visualization utilities for sweep results.

This module provides flexible visualization tools for parameter sweep results:
- Box plots for comparing configurations
- Trend plots for parameter effects
- Statistical summary plots
- Automatic metric detection and plotting
"""

import csv
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np

# Import Monumental branding colors
try:
    from wagon_control.config import (
        MONUMENTAL_BLUE,
        MONUMENTAL_CREAM,
        MONUMENTAL_DARK_BLUE,
        MONUMENTAL_ORANGE,
        MONUMENTAL_TAUPE,
    )
except ImportError:
    # Fallback colors if config not available
    MONUMENTAL_ORANGE = "#f74823"
    MONUMENTAL_BLUE = "#2374f7"
    MONUMENTAL_CREAM = "#fffdee"
    MONUMENTAL_TAUPE = "#686a5f"
    MONUMENTAL_DARK_BLUE = "#0d1b2a"


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


def plot_box_comparison(
    results: List[Dict],
    output_path: Optional[Path] = None,
    max_configs: int = 20,
    title: str = "Parameter Sweep Results"
) -> None:
    """Create box plots comparing all configurations.

    Args:
        results: List of result dictionaries
        output_path: Optional path to save figure
        max_configs: Maximum number of configurations to display
        title: Plot title
    """
    # Filter valid results
    valid_results = [r for r in results if r['is_valid'] and r['scores']]

    if not valid_results:
        print("No valid results to plot!")
        return

    # Sort by consistency score and take top N
    valid_results = sorted(
        valid_results,
        key=lambda r: r['consistency_score']
    )[:max_configs]

    # Prepare data
    config_names = [r['config_name'] for r in valid_results]
    scores_data = [r['scores'] for r in valid_results]

    # Create figure
    fig, ax = plt.subplots(figsize=(14, 8))

    # Create box plot
    bp = ax.boxplot(
        scores_data,
        labels=config_names,
        patch_artist=True,
        showmeans=True,
        meanline=True,
    )

    # Style box plots with Monumental colors
    for patch in bp['boxes']:
        patch.set_facecolor(MONUMENTAL_ORANGE)
        patch.set_alpha(0.6)

    for element in ['whiskers', 'fliers', 'caps']:
        plt.setp(bp[element], color=MONUMENTAL_TAUPE)

    plt.setp(bp['medians'], color=MONUMENTAL_BLUE, linewidth=2)
    plt.setp(bp['means'], color=MONUMENTAL_CREAM, linewidth=2)

    # Styling
    ax.set_xlabel("Configuration", fontsize=12, fontweight='bold')
    ax.set_ylabel("Score (meters)", fontsize=12, fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold', pad=20)
    ax.grid(True, alpha=0.3, axis='y')
    ax.set_facecolor('#ffffff')

    # Rotate labels for readability
    plt.xticks(rotation=45, ha='right')

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"✓ Box plot saved to {output_path}")
    else:
        plt.show()

    plt.close()


def plot_parameter_trend(
    results: List[Dict],
    parameter_name: str,
    output_path: Optional[Path] = None,
    title: Optional[str] = None
) -> None:
    """Plot how a single parameter affects performance.

    Args:
        results: List of result dictionaries
        parameter_name: Name of parameter to plot (e.g., 'MOTOR_KP_V')
        output_path: Optional path to save figure
        title: Optional plot title
    """
    # Filter results for this parameter
    param_results = []
    for r in results:
        if not r['is_valid']:
            continue

        # Check if this parameter was tested
        if parameter_name in r['param_names']:
            param_idx = r['param_names'].index(parameter_name)
            param_value = float(r['param_values'][param_idx])

            param_results.append({
                'value': param_value,
                'mean': r['mean'],
                'std_dev': r['std_dev'],
                'median': r['median'],
                'q1': r['q1'],
                'q3': r['q3'],
                'scores': r['scores'],
            })

    if not param_results:
        print(f"No valid results found for parameter '{parameter_name}'")
        return

    # Sort by parameter value
    param_results = sorted(param_results, key=lambda r: r['value'])

    # Extract data for plotting
    param_values = [r['value'] for r in param_results]
    means = [r['mean'] for r in param_results]
    medians = [r['median'] for r in param_results]
    q1s = [r['q1'] for r in param_results]
    q3s = [r['q3'] for r in param_results]

    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

    # Plot 1: Mean with confidence band
    ax1.plot(
        param_values,
        means,
        'o-',
        color=MONUMENTAL_ORANGE,
        linewidth=2,
        markersize=8,
        label='Mean'
    )
    ax1.plot(
        param_values,
        medians,
        's--',
        color=MONUMENTAL_BLUE,
        linewidth=2,
        markersize=6,
        label='Median'
    )

    # Add IQR shaded region
    ax1.fill_between(
        param_values,
        q1s,
        q3s,
        alpha=0.2,
        color=MONUMENTAL_ORANGE,
        label='IQR (Q1-Q3)'
    )

    ax1.set_xlabel(parameter_name, fontsize=12, fontweight='bold')
    ax1.set_ylabel("Score (meters)", fontsize=12, fontweight='bold')
    ax1.set_title(
        title or f"Effect of {parameter_name} on Performance",
        fontsize=14,
        fontweight='bold',
        pad=15
    )
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)

    # Plot 2: Box plots for each parameter value
    scores_data = [r['scores'] for r in param_results]
    positions = list(range(len(param_values)))

    bp = ax2.boxplot(
        scores_data,
        positions=positions,
        patch_artist=True,
        showmeans=True,
        meanline=True,
    )

    # Style box plots
    for patch in bp['boxes']:
        patch.set_facecolor(MONUMENTAL_ORANGE)
        patch.set_alpha(0.6)

    for element in ['whiskers', 'fliers', 'caps']:
        plt.setp(bp[element], color=MONUMENTAL_TAUPE)

    plt.setp(bp['medians'], color=MONUMENTAL_BLUE, linewidth=2)
    plt.setp(bp['means'], color=MONUMENTAL_CREAM, linewidth=2)

    ax2.set_xlabel(parameter_name, fontsize=12, fontweight='bold')
    ax2.set_ylabel("Score (meters)", fontsize=12, fontweight='bold')
    ax2.set_xticks(positions)
    ax2.set_xticklabels([f"{v:.3f}" for v in param_values], rotation=45, ha='right')
    ax2.grid(True, alpha=0.3, axis='y')
    ax2.set_title("Score Distribution by Parameter Value", fontsize=12, fontweight='bold')

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"✓ Parameter trend plot saved to {output_path}")
    else:
        plt.show()

    plt.close()


def plot_summary(
    csv_path: Path,
    output_dir: Optional[Path] = None,
    max_configs: int = 15
) -> None:
    """Generate all summary plots for a sweep result CSV.

    Args:
        csv_path: Path to CSV file from parameter sweep
        output_dir: Optional directory to save plots (default: same as CSV)
        max_configs: Maximum configurations to show in box plot
    """
    results = load_sweep_results(csv_path)

    if output_dir is None:
        output_dir = csv_path.parent

    output_dir.mkdir(parents=True, exist_ok=True)

    # Base filename without extension
    base_name = csv_path.stem

    # 1. Overall comparison box plot
    print("Creating overall comparison box plot...")
    plot_box_comparison(
        results,
        output_path=output_dir / f"{base_name}_comparison.png",
        max_configs=max_configs,
        title=f"Parameter Sweep Results: {csv_path.name}"
    )

    # 2. Individual parameter trend plots
    # Detect which parameters were swept
    all_params = set()
    for result in results:
        all_params.update(result['param_names'])

    # Remove empty strings
    all_params.discard('')

    for param in sorted(all_params):
        print(f"Creating trend plot for {param}...")
        plot_parameter_trend(
            results,
            param,
            output_path=output_dir / f"{base_name}_{param}.png"
        )

    print(f"\n✓ All plots saved to {output_dir}")


def visualize_csv(csv_path: Path, output_dir: Optional[Path] = None) -> None:
    """Main entry point for visualizing a sweep CSV file.

    Args:
        csv_path: Path to CSV file from parameter sweep
        output_dir: Optional directory to save plots
    """
    if not csv_path.exists():
        print(f"Error: File not found: {csv_path}")
        return

    print(f"Loading results from {csv_path}...")
    plot_summary(csv_path, output_dir)
