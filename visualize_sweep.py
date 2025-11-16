#!/usr/bin/env python3
"""Visualize parameter sweep results.

This script creates plots to visualize the results of parameter sweep testing,
making it easy to identify optimal parameter values and understand tradeoffs.
"""

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np

from wagon_control.config import (
    MONUMENTAL_BLUE,
    MONUMENTAL_CREAM,
    MONUMENTAL_DARK_BLUE,
    MONUMENTAL_ORANGE,
    MONUMENTAL_TAUPE,
)


def load_sweep_results(csv_path: Path) -> List[Dict]:
    """Load parameter sweep results from CSV.

    Args:
        csv_path: Path to the parameter sweep CSV file

    Returns:
        List of result dictionaries
    """
    results = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Parse numeric fields
            row["mean"] = float(row["mean"])
            row["std_dev"] = float(row["std_dev"])
            row["min"] = float(row["min"])
            row["max"] = float(row["max"])
            row["consistency_score"] = float(row["consistency_score"])
            row["is_valid"] = row["is_valid"] == "True"
            row["num_failures"] = int(row["num_failures"])

            # Parse scores list
            if row["all_scores"]:
                row["scores"] = [float(s) for s in row["all_scores"].split(";")]
            else:
                row["scores"] = []

            results.append(row)

    return results


def extract_parameter_sweep(
    results: List[Dict], param_name: str
) -> Tuple[List[float], List[Dict]]:
    """Extract results for a specific parameter sweep.

    Args:
        results: All results from CSV
        param_name: Parameter name to filter by

    Returns:
        Tuple of (parameter_values, filtered_results)
    """
    filtered = []
    for r in results:
        # Check if this result tests the target parameter
        if param_name in r["param_names"]:
            param_names = r["param_names"].split(";")
            param_values = r["param_values"].split(";")

            # Find the index of our parameter
            idx = param_names.index(param_name)
            r["param_value"] = float(param_values[idx])
            filtered.append(r)

    # Sort by parameter value
    filtered.sort(key=lambda x: x["param_value"])

    param_values = [r["param_value"] for r in filtered]
    return param_values, filtered


def plot_parameter_sweep(
    param_name: str,
    param_display_name: str,
    param_values: List[float],
    results: List[Dict],
    ax: plt.Axes,
) -> None:
    """Plot results for a single parameter sweep.

    Args:
        param_name: Parameter name (e.g., 'MOTOR_KP_V')
        param_display_name: Human-readable name for axis labels
        param_values: List of parameter values tested
        results: List of result dictionaries
        ax: Matplotlib axes to plot on
    """
    # Extract statistics
    means = [r["mean"] for r in results]
    std_devs = [r["std_dev"] for r in results]
    mins = [r["min"] for r in results]
    maxs = [r["max"] for r in results]
    valid = [r["is_valid"] for r in results]

    # Convert to numpy for easier math
    param_values = np.array(param_values)
    means = np.array(means)
    std_devs = np.array(std_devs)

    # Plot mean with error bars (std dev)
    valid_indices = [i for i, v in enumerate(valid) if v]
    invalid_indices = [i for i, v in enumerate(valid) if not v]

    # Plot valid results
    if valid_indices:
        ax.errorbar(
            param_values[valid_indices],
            means[valid_indices],
            yerr=std_devs[valid_indices],
            fmt="o-",
            color=MONUMENTAL_BLUE,
            capsize=5,
            capthick=2,
            linewidth=2,
            markersize=8,
            label="Valid configs",
        )

    # Plot invalid results (if any)
    if invalid_indices:
        # Plot with different color/marker
        ax.scatter(
            param_values[invalid_indices],
            means[invalid_indices],
            color=MONUMENTAL_ORANGE,
            marker="x",
            s=100,
            linewidth=3,
            label="Invalid configs (outlier >50m)",
            zorder=10,
        )

    # Add range indicators (min/max) as shaded region
    if valid_indices:
        valid_params = param_values[valid_indices]
        valid_mins = np.array(mins)[valid_indices]
        valid_maxs = np.array(maxs)[valid_indices]
        ax.fill_between(
            valid_params,
            valid_mins,
            valid_maxs,
            alpha=0.2,
            color=MONUMENTAL_BLUE,
            label="Min/Max range",
        )

    # Styling
    ax.set_xlabel(param_display_name, fontsize=12, fontweight="bold")
    ax.set_ylabel("Error (meters)", fontsize=12, fontweight="bold")
    ax.set_title(
        f"{param_display_name} vs. Performance", fontsize=14, fontweight="bold"
    )
    ax.grid(True, alpha=0.3, color=MONUMENTAL_TAUPE)
    ax.legend(loc="best", fontsize=10)

    # Add horizontal line at baseline if we have it
    baseline = next((r for r in results if "Baseline" in r["config_name"]), None)
    if baseline and baseline["is_valid"]:
        ax.axhline(
            y=baseline["mean"],
            color=MONUMENTAL_ORANGE,
            linestyle="--",
            linewidth=1.5,
            alpha=0.7,
            label=f"Baseline: {baseline['mean']:.2f}m",
        )


def create_summary_plot(results: List[Dict], output_path: Path = None) -> None:
    """Create comprehensive summary plots for all parameter sweeps.

    Args:
        results: All results from CSV
        output_path: Optional path to save figure (PNG)
    """
    # Define parameters to plot
    params = [
        ("LOCALIZER_VELOCITY_CORRECTION_GAIN", "Velocity Correction Gain"),
        ("FOLLOWER_BASE_LOOKAHEAD", "Base Lookahead Distance (m)"),
        ("FOLLOWER_LOOKAHEAD_TIME", "Lookahead Time (s)"),
        ("MOTOR_KI_V", "Linear Velocity Integral Gain (KI_V)"),
        ("MOTOR_KI_OMEGA", "Angular Velocity Integral Gain (KI_OMEGA)"),
    ]

    # Create subplot grid
    fig, axes = plt.subplots(3, 2, figsize=(16, 12))
    fig.suptitle(
        "Parameter Sweep Results Summary",
        fontsize=18,
        fontweight="bold",
        y=0.995,
    )

    axes = axes.flatten()

    for i, (param_name, display_name) in enumerate(params):
        param_values, filtered_results = extract_parameter_sweep(results, param_name)

        if param_values:
            plot_parameter_sweep(
                param_name, display_name, param_values, filtered_results, axes[i]
            )
        else:
            axes[i].text(
                0.5,
                0.5,
                f"No data for {display_name}",
                ha="center",
                va="center",
                fontsize=12,
            )
            axes[i].set_title(display_name, fontsize=14, fontweight="bold")

    # Use the last subplot for a summary table
    ax_summary = axes[-1]
    ax_summary.axis("off")

    # Get top 5 valid configs by consistency score
    valid_results = [r for r in results if r["is_valid"]]
    top_5 = sorted(valid_results, key=lambda r: r["consistency_score"])[:5]

    # Create summary text
    summary_text = "Top 5 Configurations\n"
    summary_text += "=" * 50 + "\n\n"

    for i, result in enumerate(top_5, 1):
        summary_text += f"{i}. {result['config_name']}\n"
        summary_text += f"   Mean: {result['mean']:.2f}m, Std: {result['std_dev']:.2f}m\n"
        summary_text += f"   Consistency: {result['consistency_score']:.2f}\n\n"

    ax_summary.text(
        0.1,
        0.9,
        summary_text,
        fontsize=10,
        verticalalignment="top",
        family="monospace",
        bbox=dict(boxstyle="round", facecolor=MONUMENTAL_CREAM, alpha=0.3),
    )

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"✓ Saved plot to {output_path}")

    plt.show()


def create_box_plot(results: List[Dict], output_path: Path = None) -> None:
    """Create box plots showing distribution of scores for each config.

    Args:
        results: All results from CSV
        output_path: Optional path to save figure (PNG)
    """
    # Filter valid results only
    valid_results = [r for r in results if r["is_valid"] and r["scores"]]

    if not valid_results:
        print("No valid results with scores to plot")
        return

    # Sort by consistency score
    valid_results.sort(key=lambda r: r["consistency_score"])

    # Take top 15 for readability
    top_results = valid_results[:15]

    # Prepare data
    labels = [r["config_name"] for r in top_results]
    scores = [r["scores"] for r in top_results]

    # Create figure
    fig, ax = plt.subplots(figsize=(14, 10))

    # Create box plot
    bp = ax.boxplot(
        scores,
        labels=labels,
        patch_artist=True,
        vert=False,
        widths=0.6,
    )

    # Color the boxes
    for patch in bp["boxes"]:
        patch.set_facecolor(MONUMENTAL_BLUE)
        patch.set_alpha(0.6)

    for whisker in bp["whiskers"]:
        whisker.set_color(MONUMENTAL_TAUPE)
        whisker.set_linewidth(1.5)

    for cap in bp["caps"]:
        cap.set_color(MONUMENTAL_TAUPE)
        cap.set_linewidth(1.5)

    for median in bp["medians"]:
        median.set_color(MONUMENTAL_ORANGE)
        median.set_linewidth(2)

    # Add reference line at 50m (outlier threshold)
    ax.axvline(x=50, color="red", linestyle="--", linewidth=2, alpha=0.5, label="Outlier threshold (50m)")

    # Styling
    ax.set_xlabel("Error (meters)", fontsize=12, fontweight="bold")
    ax.set_ylabel("Configuration", fontsize=12, fontweight="bold")
    ax.set_title(
        "Score Distribution for Top 15 Configurations",
        fontsize=14,
        fontweight="bold",
    )
    ax.grid(True, alpha=0.3, axis="x", color=MONUMENTAL_TAUPE)
    ax.legend(loc="best")

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"✓ Saved box plot to {output_path}")

    plt.show()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Visualize parameter sweep results"
    )
    parser.add_argument(
        "csv_file",
        nargs="?",
        type=Path,
        help="Path to parameter sweep CSV file (or auto-detect latest)",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save plots to PNG files",
    )
    parser.add_argument(
        "--box-plot",
        action="store_true",
        help="Also create box plot of score distributions",
    )

    args = parser.parse_args()

    # Find CSV file
    if args.csv_file:
        csv_path = args.csv_file
    else:
        # Auto-detect latest CSV in results directory
        results_dir = Path("results")
        csv_files = sorted(results_dir.glob("parameter_sweep_*.csv"), reverse=True)

        if not csv_files:
            print("Error: No parameter sweep CSV files found in results/")
            print("Run parameter_sweep.py first, or specify a CSV file.")
            return

        csv_path = csv_files[0]
        print(f"Using latest results: {csv_path}")

    # Load results
    print(f"Loading results from {csv_path}...")
    results = load_sweep_results(csv_path)
    print(f"✓ Loaded {len(results)} configurations")

    # Create output paths
    if args.save:
        stem = csv_path.stem
        summary_plot_path = csv_path.parent / f"{stem}_plots.png"
        box_plot_path = csv_path.parent / f"{stem}_boxplot.png"
    else:
        summary_plot_path = None
        box_plot_path = None

    # Create plots
    print("Creating summary plots...")
    create_summary_plot(results, summary_plot_path)

    if args.box_plot:
        print("Creating box plot...")
        create_box_plot(results, box_plot_path)


if __name__ == "__main__":
    main()
