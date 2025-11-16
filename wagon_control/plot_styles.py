"""Shared plotting utilities and styles for wagon control visualizations.

This module provides:
- Color schemes and colormaps (Monumental branding)
- CSV data loading functions
- Timestamp parsing utilities
- Common plot styling functions

All visualization modules should import from this module to ensure consistency.
"""

import csv
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.colors import LinearSegmentedColormap

from .config import (
    MONUMENTAL_BLUE,
    MONUMENTAL_CREAM,
    MONUMENTAL_DARK_BLUE,
    MONUMENTAL_ORANGE,
    MONUMENTAL_TAUPE,
    MONUMENTAL_YELLOW_ORANGE,
)

# ============================================================================
# Color Scheme and Colormaps
# ============================================================================

# Export colors for direct use
__all__ = [
    "MONUMENTAL_ORANGE",
    "MONUMENTAL_BLUE",
    "MONUMENTAL_CREAM",
    "MONUMENTAL_TAUPE",
    "MONUMENTAL_YELLOW_ORANGE",
    "MONUMENTAL_DARK_BLUE",
    "MONUMENTAL_CMAP",
    "parse_timestamp",
    "load_csv_data",
    "load_csv_to_dict",
    "style_axis",
    "add_branded_legend",
]

# Create custom colormap: orange -> blue
MONUMENTAL_CMAP = LinearSegmentedColormap.from_list(
    "monumental", [MONUMENTAL_ORANGE, MONUMENTAL_BLUE]
)
"""Monumental brand colormap transitioning from orange to blue."""


# ============================================================================
# Timestamp Parsing
# ============================================================================


def parse_timestamp(timestamp_str: str) -> float:
    """Parse ISO format timestamp string to seconds since epoch.

    Handles both ISO format timestamps and numeric timestamp strings.

    Args:
        timestamp_str: ISO format timestamp string or numeric string.

    Returns:
        Unix timestamp as float (seconds since epoch).

    Raises:
        ValueError: If timestamp cannot be parsed.

    Example:
        >>> t = parse_timestamp("2024-01-15T10:30:00Z")
        >>> t = parse_timestamp("1705315800.123")
    """
    try:
        # Try parsing as ISO format with timezone
        dt = datetime.fromisoformat(timestamp_str.replace("Z", "+00:00"))
        return dt.timestamp()
    except (ValueError, AttributeError):
        # Try parsing as numeric timestamp
        return float(timestamp_str)


# ============================================================================
# CSV Data Loading
# ============================================================================


def load_csv_data(filepath: Path) -> Tuple[List[str], List[List[str]]]:
    """Load CSV file and return headers and data rows.

    Args:
        filepath: Path to the CSV file.

    Returns:
        Tuple containing (headers, data_rows).

    Raises:
        FileNotFoundError: If the CSV file does not exist.

    Example:
        >>> headers, rows = load_csv_data(Path("data.csv"))
        >>> print(headers)
        ['timestamp', 'x', 'y']
    """
    if not filepath.exists():
        raise FileNotFoundError(f"CSV file not found: {filepath}")

    with open(filepath, newline="") as f:
        reader = csv.reader(f)
        headers = next(reader)
        data_rows = list(reader)

    return headers, data_rows


def load_csv_to_dict(csv_path: Path) -> Dict[str, np.ndarray]:
    """Load CSV file into dictionary of numpy arrays.

    Automatically converts numeric values to floats. Non-numeric values
    become NaN.

    Args:
        csv_path: Path to CSV file.

    Returns:
        Dictionary mapping column names to numpy arrays.

    Raises:
        FileNotFoundError: If the CSV file does not exist.

    Example:
        >>> data = load_csv_to_dict(Path("imu_data.csv"))
        >>> print(data.keys())
        dict_keys(['timestamp', 'x_dot', 'y_dot', 'theta_dot'])
        >>> print(data['timestamp'].shape)
        (500,)
    """
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    with open(csv_path) as f:
        reader = csv.DictReader(f)
        data: Dict[str, List[float]] = {}
        for row in reader:
            for key, value in row.items():
                if key not in data:
                    data[key] = []
                try:
                    # Try to convert to float
                    data[key].append(float(value))
                except (ValueError, TypeError):
                    # Non-numeric or empty values become NaN
                    data[key].append(np.nan)

    # Convert lists to numpy arrays
    return {key: np.array(values) for key, values in data.items()}


# ============================================================================
# Plot Styling Functions
# ============================================================================


def style_axis(
    ax: Axes,
    title: str = "",
    xlabel: str = "",
    ylabel: str = "",
    grid: bool = True,
    dark_mode: bool = False,
) -> None:
    """Apply consistent styling to a matplotlib axis.

    Args:
        ax: Matplotlib axis to style.
        title: Plot title (optional).
        xlabel: X-axis label (optional).
        ylabel: Y-axis label (optional).
        grid: Whether to show grid lines (default: True).
        dark_mode: Whether to use dark mode styling (default: False).

    Example:
        >>> fig, ax = plt.subplots()
        >>> style_axis(ax, title="Position", xlabel="X (m)", ylabel="Y (m)")
    """
    # Set labels and title
    if title:
        if dark_mode:
            ax.set_title(title, fontweight="bold", color=MONUMENTAL_CREAM)
        else:
            ax.set_title(title, fontweight="bold")

    if xlabel:
        if dark_mode:
            ax.set_xlabel(xlabel, color=MONUMENTAL_CREAM)
        else:
            ax.set_xlabel(xlabel)

    if ylabel:
        if dark_mode:
            ax.set_ylabel(ylabel, color=MONUMENTAL_CREAM)
        else:
            ax.set_ylabel(ylabel)

    # Grid styling
    if grid:
        ax.grid(True, alpha=0.3, linestyle="--", linewidth=0.5)

    # Dark mode specific styling
    if dark_mode:
        ax.set_facecolor(MONUMENTAL_DARK_BLUE)
        ax.tick_params(colors=MONUMENTAL_CREAM)
        for spine in ax.spines.values():
            spine.set_edgecolor(MONUMENTAL_TAUPE)
    else:
        # Light mode: use default matplotlib styling
        pass


def add_branded_legend(ax: Axes, loc: str = "best", dark_mode: bool = False, **kwargs) -> None:
    """Add a legend with Monumental brand styling.

    Args:
        ax: Matplotlib axis to add legend to.
        loc: Legend location (default: "best").
        dark_mode: Whether to use dark mode styling (default: False).
        **kwargs: Additional keyword arguments passed to ax.legend().

    Example:
        >>> ax.plot([1, 2, 3], label="Data")
        >>> add_branded_legend(ax, loc="upper right")
    """
    # Default legend styling
    legend_kwargs = {
        "loc": loc,
        "framealpha": 0.9,
        "edgecolor": MONUMENTAL_TAUPE,
    }

    # Dark mode specific styling
    if dark_mode:
        legend_kwargs["facecolor"] = MONUMENTAL_DARK_BLUE
        legend_kwargs["labelcolor"] = MONUMENTAL_CREAM
        legend_kwargs["edgecolor"] = MONUMENTAL_TAUPE

    # Merge with user-provided kwargs (user kwargs take precedence)
    legend_kwargs.update(kwargs)

    ax.legend(**legend_kwargs)


# ============================================================================
# Figure Creation Helpers
# ============================================================================


def create_figure(
    figsize: Tuple[float, float] = (12, 8), dark_mode: bool = False, title: str = ""
) -> Tuple[plt.Figure, Axes]:
    """Create a matplotlib figure with Monumental branding.

    Args:
        figsize: Figure size in inches (width, height).
        dark_mode: Whether to use dark mode styling (default: False).
        title: Optional main figure title.

    Returns:
        Tuple of (figure, axes).

    Example:
        >>> fig, ax = create_figure(figsize=(10, 6), dark_mode=True, title="Results")
    """
    if dark_mode:
        fig, ax = plt.subplots(figsize=figsize, facecolor=MONUMENTAL_DARK_BLUE)
        if title:
            fig.suptitle(title, fontsize=14, fontweight="bold", color=MONUMENTAL_CREAM)
    else:
        fig, ax = plt.subplots(figsize=figsize)
        if title:
            fig.suptitle(title, fontsize=14, fontweight="bold")

    return fig, ax


def save_figure(
    fig: plt.Figure, filepath: Path, dpi: int = 300, bbox_inches: str = "tight"
) -> None:
    """Save figure with consistent settings.

    Args:
        fig: Matplotlib figure to save.
        filepath: Path where to save the figure.
        dpi: Resolution in dots per inch (default: 300).
        bbox_inches: Bounding box setting (default: "tight").

    Example:
        >>> save_figure(fig, Path("results/plot.png"))
    """
    fig.savefig(filepath, dpi=dpi, bbox_inches=bbox_inches)
    print(f"Saved figure to {filepath}")
