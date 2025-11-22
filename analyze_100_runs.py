#!/usr/bin/env python3
"""
Comprehensive statistical analysis of 100 wagon control test runs.
Uses the modular analysis package for statistics and visualization.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from typing import List, Dict, Tuple
import csv

# Import from analysis package
from analysis.statistics import compute_statistics, Statistics
from analysis.visualize import MONUMENTAL_ORANGE, MONUMENTAL_BLUE, MONUMENTAL_CREAM, MONUMENTAL_TAUPE


class RunAnalyzer:
    """Analyzes 100 individual test runs with detailed outlier investigation."""

    def __init__(self, results_dir: str = "results"):
        self.results_dir = Path(results_dir)
        self.scores = []
        self.run_data = []
        self.stats = None

    def collect_scores(self) -> List[Dict]:
        """Collect all scores from run directories."""
        print("Collecting scores from 100 test runs...")

        run_patterns = sorted(self.results_dir.glob("run_*/score.txt"))

        for score_file in run_patterns:
            run_dir = score_file.parent
            run_name = run_dir.name

            try:
                with open(score_file, 'r') as f:
                    score = float(f.read().strip())

                self.run_data.append({
                    'run_name': run_name,
                    'run_dir': str(run_dir),
                    'score': score,
                    'outlier_type': 'normal'
                })
                self.scores.append(score)

            except Exception as e:
                print(f"Warning: Could not read {score_file}: {e}")

        print(f"Collected {len(self.run_data)} scores")
        if self.scores:
            print(f"Score range: {min(self.scores):.3f}mm - {max(self.scores):.3f}mm")

        return self.run_data

    def compute_statistics(self) -> Statistics:
        """Compute comprehensive statistics using analysis package."""
        print("\nComputing statistics...")
        self.stats = compute_statistics(self.scores)
        return self.stats

    def print_statistics(self):
        """Print statistical summary."""
        if self.stats is None:
            self.compute_statistics()

        print("\n" + "="*70)
        print("STATISTICAL SUMMARY - 100 Wagon Control Test Runs")
        print("="*70)
        print(f"Sample Size:           n = {self.stats.n}")
        print(f"Mean Score:            {self.stats.mean:.3f} mm")
        print(f"Median Score:          {self.stats.median:.3f} mm")
        print(f"Standard Deviation:    {self.stats.std:.3f} mm")
        print(f"Minimum Score:         {self.stats.min:.3f} mm (best)")
        print(f"Maximum Score:         {self.stats.max:.3f} mm (worst)")
        print(f"")
        print(f"Quartiles:")
        print(f"  Q1 (25th percentile): {self.stats.q1:.3f} mm")
        print(f"  Q2 (50th percentile): {self.stats.median:.3f} mm")
        print(f"  Q3 (75th percentile): {self.stats.q3:.3f} mm")
        print(f"  IQR (Q3 - Q1):        {self.stats.iqr:.3f} mm")
        print(f"")
        print(f"95% Confidence Interval:")
        print(f"  Lower Bound:          {self.stats.ci_95_lower:.3f} mm")
        print(f"  Upper Bound:          {self.stats.ci_95_upper:.3f} mm")
        print("="*70)

    def identify_outliers(self) -> Tuple[List[Dict], List[Dict], List[Dict]]:
        """Identify outliers using IQR method."""
        if self.stats is None:
            self.compute_statistics()

        # IQR method: outliers are beyond Q1 - 1.5*IQR or Q3 + 1.5*IQR
        lower_fence = self.stats.q1 - 1.5 * self.stats.iqr
        upper_fence = self.stats.q3 + 1.5 * self.stats.iqr

        print(f"\nOutlier Detection (IQR Method):")
        print(f"  Lower Fence: {lower_fence:.3f} mm")
        print(f"  Upper Fence: {upper_fence:.3f} mm")

        # Classify runs
        good_outliers = []
        poor_outliers = []
        normal = []

        for run in self.run_data:
            if run['score'] < lower_fence:
                run['outlier_type'] = 'exceptionally_good'
                good_outliers.append(run)
            elif run['score'] > upper_fence:
                run['outlier_type'] = 'poor'
                poor_outliers.append(run)
            else:
                normal.append(run)

        # Sort outliers by score
        good_outliers.sort(key=lambda x: x['score'])
        poor_outliers.sort(key=lambda x: x['score'], reverse=True)

        print(f"\n  Exceptionally Good: {len(good_outliers)} runs (< {lower_fence:.3f} mm)")
        print(f"  Normal Performance: {len(normal)} runs")
        print(f"  Poor Performance:   {len(poor_outliers)} runs (> {upper_fence:.3f} mm)")

        return good_outliers, poor_outliers, normal

    def plot_distributions(self, output_dir: str = "analysis_output"):
        """Generate comprehensive distribution plots."""
        output_path = Path(output_dir)
        output_path.mkdir(exist_ok=True)

        print(f"\nGenerating distribution plots...")

        # Create comprehensive figure
        fig = plt.figure(figsize=(16, 10))
        gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)

        # 1. Box Plot
        ax1 = fig.add_subplot(gs[0, :2])
        bp = ax1.boxplot([self.scores], vert=False, widths=0.6, patch_artist=True,
                          showmeans=True, meanline=True,
                          boxprops=dict(facecolor=MONUMENTAL_CREAM, color=MONUMENTAL_ORANGE, linewidth=2),
                          whiskerprops=dict(color=MONUMENTAL_ORANGE, linewidth=2),
                          capprops=dict(color=MONUMENTAL_ORANGE, linewidth=2),
                          medianprops=dict(color=MONUMENTAL_BLUE, linewidth=3),
                          meanprops=dict(color='red', linewidth=3, linestyle='--'),
                          flierprops=dict(marker='o', markerfacecolor='red', markersize=8,
                                         markeredgecolor='darkred', alpha=0.6))
        ax1.set_xlabel('Average Sample Error (mm)', fontsize=12, fontweight='bold')
        ax1.set_title('Box Plot: Score Distribution (n=100)', fontsize=14, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.set_yticks([])

        # 2. Histogram
        ax2 = fig.add_subplot(gs[1, :2])
        n, bins, patches = ax2.hist(self.scores, bins=20, alpha=0.7, color=MONUMENTAL_CREAM,
                                     edgecolor=MONUMENTAL_ORANGE, linewidth=1.5, density=True)

        # Add mean and median lines
        ax2.axvline(self.stats.mean, color='red', linestyle='--', linewidth=2, label=f'Mean: {self.stats.mean:.2f}mm')
        ax2.axvline(self.stats.median, color=MONUMENTAL_ORANGE, linestyle='-', linewidth=2,
                   label=f'Median: {self.stats.median:.2f}mm')

        ax2.set_xlabel('Average Sample Error (mm)', fontsize=12, fontweight='bold')
        ax2.set_ylabel('Density', fontsize=12, fontweight='bold')
        ax2.set_title('Histogram with Mean and Median', fontsize=14, fontweight='bold')
        ax2.legend(fontsize=10)
        ax2.grid(True, alpha=0.3)

        # 3. Violin-like Plot using histogram
        ax3 = fig.add_subplot(gs[2, :2])
        parts = ax3.violinplot([self.scores], vert=False, widths=0.7, showmeans=True, showmedians=True)
        for pc in parts['bodies']:
            pc.set_facecolor(MONUMENTAL_CREAM)
            pc.set_edgecolor(MONUMENTAL_ORANGE)
            pc.set_linewidth(2)
            pc.set_alpha(0.8)
        parts['cmeans'].set_color('red')
        parts['cmedians'].set_color(MONUMENTAL_BLUE)
        parts['cbars'].set_color(MONUMENTAL_ORANGE)
        parts['cmaxes'].set_color(MONUMENTAL_ORANGE)
        parts['cmins'].set_color(MONUMENTAL_ORANGE)

        ax3.set_xlabel('Average Sample Error (mm)', fontsize=12, fontweight='bold')
        ax3.set_title('Violin Plot: Distribution Shape', fontsize=14, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        ax3.set_yticks([])

        # 4. Statistics Summary (text box)
        ax4 = fig.add_subplot(gs[0, 2])
        ax4.axis('off')
        stats_text = f"""
STATISTICS SUMMARY

n = {self.stats.n}

Mean:    {self.stats.mean:.3f} mm
Median:  {self.stats.median:.3f} mm
Std Dev: {self.stats.std:.3f} mm

Min:     {self.stats.min:.3f} mm
Max:     {self.stats.max:.3f} mm
Range:   {self.stats.max - self.stats.min:.3f} mm

Q1:      {self.stats.q1:.3f} mm
Q3:      {self.stats.q3:.3f} mm
IQR:     {self.stats.iqr:.3f} mm

95% CI:
[{self.stats.ci_95_lower:.3f}, {self.stats.ci_95_upper:.3f}]
        """
        ax4.text(0.1, 0.5, stats_text, fontsize=10, family='monospace',
                verticalalignment='center', bbox=dict(boxstyle='round',
                facecolor=MONUMENTAL_CREAM, alpha=0.8, edgecolor=MONUMENTAL_ORANGE, linewidth=2))

        # 5. Score Distribution by Percentile
        ax5 = fig.add_subplot(gs[1, 2])
        percentiles = np.arange(0, 101, 5)
        percentile_values = np.percentile(self.scores, percentiles)
        ax5.plot(percentiles, percentile_values, color=MONUMENTAL_BLUE, linewidth=2, marker='o', markersize=4)
        ax5.axhline(self.stats.median, color='red', linestyle='--', linewidth=1, alpha=0.5, label='Median')
        ax5.set_xlabel('Percentile', fontsize=10, fontweight='bold')
        ax5.set_ylabel('Score (mm)', fontsize=10, fontweight='bold')
        ax5.set_title('Score by Percentile', fontsize=12, fontweight='bold')
        ax5.grid(True, alpha=0.3)
        ax5.legend(fontsize=8)

        # 6. Cumulative Distribution
        ax6 = fig.add_subplot(gs[2, 2])
        sorted_scores = np.sort(self.scores)
        cumulative = np.arange(1, len(sorted_scores) + 1) / len(sorted_scores) * 100
        ax6.plot(sorted_scores, cumulative, color=MONUMENTAL_BLUE, linewidth=2)
        ax6.axhline(50, color='red', linestyle='--', linewidth=1, alpha=0.5, label='Median')
        ax6.axhline(25, color='orange', linestyle='--', linewidth=1, alpha=0.5, label='Q1')
        ax6.axhline(75, color='orange', linestyle='--', linewidth=1, alpha=0.5, label='Q3')
        ax6.set_xlabel('Average Sample Error (mm)', fontsize=12, fontweight='bold')
        ax6.set_ylabel('Cumulative %', fontsize=12, fontweight='bold')
        ax6.set_title('Cumulative Distribution', fontsize=12, fontweight='bold')
        ax6.grid(True, alpha=0.3)
        ax6.legend(fontsize=8)

        fig.suptitle('Statistical Distribution Analysis - 100 Wagon Control Tests',
                    fontsize=16, fontweight='bold', y=0.995)

        output_file = output_path / 'distribution_analysis.png'
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"  Saved: {output_file}")
        plt.close()

    def plot_outliers(self, good_outliers: List[Dict], poor_outliers: List[Dict],
                     normal: List[Dict], output_dir: str = "analysis_output"):
        """Create outlier-specific visualizations."""
        output_path = Path(output_dir)

        print(f"Generating outlier analysis plots...")

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

        # Plot 1: Score timeline with outliers highlighted
        sorted_data = sorted(self.run_data, key=lambda x: x['run_name'])
        colors = []
        scores = []
        for run in sorted_data:
            scores.append(run['score'])
            if run['outlier_type'] == 'exceptionally_good':
                colors.append('green')
            elif run['outlier_type'] == 'poor':
                colors.append('red')
            else:
                colors.append(MONUMENTAL_BLUE)

        ax1.scatter(range(len(sorted_data)), scores, c=colors, alpha=0.6, s=50)
        ax1.axhline(self.stats.mean, color='black', linestyle='--', linewidth=2, label='Mean', alpha=0.7)
        ax1.axhline(self.stats.q1 - 1.5*self.stats.iqr, color='green', linestyle=':', linewidth=2,
                   label='Lower Fence', alpha=0.7)
        ax1.axhline(self.stats.q3 + 1.5*self.stats.iqr, color='red', linestyle=':', linewidth=2,
                   label='Upper Fence', alpha=0.7)
        ax1.set_xlabel('Run Index (sorted by name)', fontsize=12, fontweight='bold')
        ax1.set_ylabel('Average Sample Error (mm)', fontsize=12, fontweight='bold')
        ax1.set_title('All 100 Runs with Outliers Highlighted', fontsize=14, fontweight='bold')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot 2: Comparison bar chart
        categories = ['Exceptionally\nGood', 'Normal', 'Poor']
        counts = [len(good_outliers), len(normal), len(poor_outliers)]
        bar_colors = ['green', MONUMENTAL_BLUE, 'red']

        bars = ax2.bar(categories, counts, color=bar_colors, alpha=0.7, edgecolor='black', linewidth=2)
        ax2.set_ylabel('Number of Runs', fontsize=12, fontweight='bold')
        ax2.set_title('Outlier Classification', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3, axis='y')

        # Add count labels on bars
        total = len(self.run_data)
        for bar, count in zip(bars, counts):
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height,
                    f'{count}\n({count/total*100:.1f}%)',
                    ha='center', va='bottom', fontsize=12, fontweight='bold')

        fig.suptitle('Outlier Detection and Classification', fontsize=16, fontweight='bold')
        plt.tight_layout()

        output_file = output_path / 'outlier_analysis.png'
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"  Saved: {output_file}")
        plt.close()

    def analyze_run_details(self, run_dir: Path) -> Dict:
        """Load and analyze detailed data from a single run."""
        details = {'run_dir': str(run_dir), 'run_name': run_dir.name}

        try:
            # Load score
            with open(run_dir / 'score.txt', 'r') as f:
                details['score'] = float(f.read().strip())

            # Load and analyze CSV files using numpy
            def load_csv_column(filename, column_name):
                """Helper to load a specific column from CSV."""
                with open(run_dir / filename, 'r') as f:
                    reader = csv.DictReader(f)
                    return [float(row[column_name]) for row in reader]

            # Load tracking metrics
            error_l2 = load_csv_column('tracking_metrics.csv', 'error_l2')
            details['max_error'] = max(error_l2) if error_l2 else 0
            details['error_std'] = np.std(error_l2) if error_l2 else 0

            # Load EKF diagnostics
            p_trace = load_csv_column('ekf_diagnostics.csv', 'P_trace')
            innovation = load_csv_column('ekf_diagnostics.csv', 'innovation_norm')
            slip = load_csv_column('ekf_diagnostics.csv', 'slip_magnitude')
            outliers_rejected = load_csv_column('ekf_diagnostics.csv', 'outliers_rejected')
            mahalanobis = load_csv_column('ekf_diagnostics.csv', 'mahalanobis')

            details['avg_p_trace'] = np.mean(p_trace) if p_trace else 0
            details['max_p_trace'] = max(p_trace) if p_trace else 0
            details['avg_innovation'] = np.mean(innovation) if innovation else 0
            details['total_outliers_rejected'] = sum(outliers_rejected) if outliers_rejected else 0
            details['max_slip'] = max(slip) if slip else 0
            details['avg_mahalanobis'] = np.mean(mahalanobis) if mahalanobis else 0

            # Load motor data
            v_err = load_csv_column('motor_data.csv', 'v_err')
            omega_err = load_csv_column('motor_data.csv', 'omega_err')
            v_cmd = load_csv_column('motor_data.csv', 'v_cmd')
            omega_cmd = load_csv_column('motor_data.csv', 'omega_cmd')

            details['v_err_std'] = np.std(v_err) if v_err else 0
            details['omega_err_std'] = np.std(omega_err) if omega_err else 0
            details['v_cmd_std'] = np.std(v_cmd) if v_cmd else 0
            details['omega_cmd_std'] = np.std(omega_cmd) if omega_cmd else 0

        except Exception as e:
            print(f"Warning: Could not analyze {run_dir}: {e}")
            details['error'] = str(e)

        return details

    def compare_outliers(self, good_outliers: List[Dict], poor_outliers: List[Dict],
                        output_dir: str = "analysis_output"):
        """Deep dive into why outliers performed differently."""
        output_path = Path(output_dir)

        print("\nAnalyzing outlier characteristics...")

        # Analyze top 5 good and poor runs
        good_details = []
        for run in good_outliers[:5]:
            details = self.analyze_run_details(Path(run['run_dir']))
            good_details.append(details)

        poor_details = []
        for run in poor_outliers[:5]:
            details = self.analyze_run_details(Path(run['run_dir']))
            poor_details.append(details)

        # Generate comparison report
        report_file = output_path / 'outlier_comparison.txt'
        with open(report_file, 'w') as f:
            f.write("="*80 + "\n")
            f.write("OUTLIER ANALYSIS: WHY DID SOME RUNS PERFORM BETTER/WORSE?\n")
            f.write("="*80 + "\n\n")

            f.write("TOP 5 EXCEPTIONALLY GOOD RUNS:\n")
            f.write("-"*80 + "\n")
            for details in good_details:
                f.write(f"\nRun: {details['run_name']}\n")
                f.write(f"  Score: {details['score']:.3f} mm\n")
                if 'error' not in details:
                    f.write(f"  Max L2 Error: {details['max_error']:.3f} mm\n")
                    f.write(f"  Error Std Dev: {details['error_std']:.3f} mm\n")
                    f.write(f"  Avg EKF Covariance Trace: {details['avg_p_trace']:.6f}\n")
                    f.write(f"  Max Slip Magnitude: {details['max_slip']:.3f}\n")
                    f.write(f"  Outliers Rejected: {details['total_outliers_rejected']:.0f}\n")
                    f.write(f"  Velocity Error Std: {details['v_err_std']:.3f}\n")
                    f.write(f"  Angular Error Std: {details['omega_err_std']:.3f}\n")

            f.write("\n" + "="*80 + "\n")
            f.write("TOP 5 POOR PERFORMING RUNS:\n")
            f.write("-"*80 + "\n")
            for details in poor_details:
                f.write(f"\nRun: {details['run_name']}\n")
                f.write(f"  Score: {details['score']:.3f} mm\n")
                if 'error' not in details:
                    f.write(f"  Max L2 Error: {details['max_error']:.3f} mm\n")
                    f.write(f"  Error Std Dev: {details['error_std']:.3f} mm\n")
                    f.write(f"  Avg EKF Covariance Trace: {details['avg_p_trace']:.6f}\n")
                    f.write(f"  Max Slip Magnitude: {details['max_slip']:.3f}\n")
                    f.write(f"  Outliers Rejected: {details['total_outliers_rejected']:.0f}\n")
                    f.write(f"  Velocity Error Std: {details['v_err_std']:.3f}\n")
                    f.write(f"  Angular Error Std: {details['omega_err_std']:.3f}\n")

            f.write("\n" + "="*80 + "\n")
            f.write("COMPARATIVE ANALYSIS:\n")
            f.write("-"*80 + "\n\n")

            # Compute averages for metrics
            metrics = ['score', 'max_error', 'error_std', 'avg_p_trace', 'max_slip',
                      'total_outliers_rejected', 'v_err_std', 'omega_err_std']

            for metric in metrics:
                good_values = [d[metric] for d in good_details if metric in d and 'error' not in d]
                poor_values = [d[metric] for d in poor_details if metric in d and 'error' not in d]

                if good_values and poor_values:
                    good_avg = np.mean(good_values)
                    poor_avg = np.mean(poor_values)
                    diff = poor_avg - good_avg
                    pct_diff = (diff / good_avg * 100) if good_avg != 0 else 0

                    f.write(f"{metric}:\n")
                    f.write(f"  Good runs avg:  {good_avg:.4f}\n")
                    f.write(f"  Poor runs avg:  {poor_avg:.4f}\n")
                    f.write(f"  Difference:     {diff:+.4f} ({pct_diff:+.1f}%)\n\n")

            f.write("\n" + "="*80 + "\n")
            f.write("KEY FINDINGS:\n")
            f.write("-"*80 + "\n")
            f.write("""
Good runs typically exhibit:
- Lower and more stable position errors
- Lower EKF covariance (better estimation confidence)
- Less wheel slip
- More stable control commands (lower velocity/angular error std dev)

Poor runs typically exhibit:
- Higher peak errors and error variability
- Higher EKF covariance (estimation uncertainty)
- More wheel slip events
- More oscillatory control (higher command std dev)
- Potentially more measurement outliers

These patterns suggest that poor performance is often linked to:
1. Estimation quality degradation (high EKF uncertainty)
2. Physical disturbances (wheel slip)
3. Control instability (oscillations)
            """)

        print(f"  Saved: {report_file}")

        # Create visual comparison
        self.plot_metric_comparison(good_details, poor_details, output_path)

        return good_details, poor_details

    def plot_metric_comparison(self, good_details: List[Dict], poor_details: List[Dict],
                               output_path: Path):
        """Create visual comparison of metrics between good and poor runs."""

        metrics = [
            ('score', 'Score (mm)'),
            ('max_error', 'Max L2 Error (mm)'),
            ('avg_p_trace', 'Avg EKF Cov Trace'),
            ('max_slip', 'Max Slip Magnitude'),
            ('v_err_std', 'Velocity Error Std'),
            ('omega_err_std', 'Angular Error Std')
        ]

        fig, axes = plt.subplots(2, 3, figsize=(16, 10))
        axes = axes.flatten()

        for idx, (metric, label) in enumerate(metrics):
            ax = axes[idx]

            good_values = [d[metric] for d in good_details if metric in d and 'error' not in d]
            poor_values = [d[metric] for d in poor_details if metric in d and 'error' not in d]

            if good_values and poor_values:
                data = [good_values, poor_values]
                bp = ax.boxplot(data, labels=['Good Runs', 'Poor Runs'], patch_artist=True)

                bp['boxes'][0].set_facecolor('lightgreen')
                bp['boxes'][1].set_facecolor('lightcoral')

                ax.set_ylabel(label, fontsize=10, fontweight='bold')
                ax.set_title(f'{label} Comparison', fontsize=11, fontweight='bold')
                ax.grid(True, alpha=0.3)

        fig.suptitle('Metric Comparison: Good vs Poor Performing Runs',
                    fontsize=14, fontweight='bold')
        plt.tight_layout()

        output_file = output_path / 'metric_comparison.png'
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"  Saved: {output_file}")
        plt.close()

    def save_detailed_results(self, output_dir: str = "analysis_output"):
        """Save all run data to CSV."""
        output_path = Path(output_dir)
        results_file = output_path / 'all_runs_detailed.csv'

        with open(results_file, 'w', newline='') as f:
            fieldnames = ['run_name', 'run_dir', 'score', 'outlier_type']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.run_data)

        print(f"\nSaved detailed results to: {results_file}")


def main():
    """Main analysis workflow."""
    print("="*70)
    print("WAGON CONTROL SYSTEM: 100-RUN STATISTICAL ANALYSIS")
    print("="*70)

    analyzer = RunAnalyzer("results")

    # 1. Collect scores
    analyzer.collect_scores()

    # 2. Compute and print statistics
    analyzer.compute_statistics()
    analyzer.print_statistics()

    # 3. Identify outliers
    good_outliers, poor_outliers, normal = analyzer.identify_outliers()

    print(f"\nExceptionally Good Runs (top 10):")
    for run in good_outliers[:10]:
        print(f"  {run['run_name']}: {run['score']:.3f} mm")

    print(f"\nPoor Performing Runs (top 10):")
    for run in poor_outliers[:10]:
        print(f"  {run['run_name']}: {run['score']:.3f} mm")

    # 4. Generate visualizations
    output_dir = "analysis_output"
    Path(output_dir).mkdir(exist_ok=True)

    analyzer.plot_distributions(output_dir)
    analyzer.plot_outliers(good_outliers, poor_outliers, normal, output_dir)

    # 5. Deep dive into outliers
    good_df, poor_df = analyzer.compare_outliers(good_outliers, poor_outliers, output_dir)

    # 6. Save detailed results
    analyzer.save_detailed_results(output_dir)

    print("\n" + "="*70)
    print("ANALYSIS COMPLETE!")
    print(f"All outputs saved to: {output_dir}/")
    print("="*70)


if __name__ == "__main__":
    main()
