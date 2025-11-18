"""Generic analysis framework for wagon control system.

This package provides a modular, reusable framework for:
- Parameter sweep testing
- Statistical analysis
- Result visualization

The framework is completely generic and works with any config.py parameters.

Quick Start:
    >>> from analysis import ParameterSweep
    >>> sweep = ParameterSweep(
    ...     parameters={'MOTOR_KP_V': [1.0, 1.5, 2.0]},
    ...     runs_per_config=10
    ... )
    >>> sweep.run()
    >>> sweep.save_results('results/my_sweep.csv')

Command Line:
    # Run a parameter sweep
    python -m analysis.cli sweep --param MOTOR_KP_V=1.0,1.5,2.0 --runs 10

    # Visualize results
    python -m analysis.cli visualize results/parameter_sweep_*.csv

    # Generate statistical report
    python -m analysis.cli stats results/parameter_sweep_*.csv
"""

# Core sweep framework
from analysis.sweep import (
    ConfigResult,
    ParameterSweep,
    TestResult,
)

# Statistical analysis
from analysis.statistics import (
    Statistics,
    compare_configurations,
    compute_statistics,
    generate_report,
    load_sweep_results,
    rank_configurations,
)

# Visualization
from analysis.visualize import (
    plot_box_comparison,
    plot_parameter_trend,
    plot_summary,
    visualize_csv,
)

__all__ = [
    # Sweep framework
    'ParameterSweep',
    'TestResult',
    'ConfigResult',
    # Statistics
    'Statistics',
    'compute_statistics',
    'load_sweep_results',
    'rank_configurations',
    'compare_configurations',
    'generate_report',
    # Visualization
    'plot_box_comparison',
    'plot_parameter_trend',
    'plot_summary',
    'visualize_csv',
]

__version__ = '1.0.0'
