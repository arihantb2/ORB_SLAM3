"""Shared matplotlib style for research-paper quality figures."""

import warnings
import matplotlib.pyplot as plt

warnings.filterwarnings("ignore", message="Unable to import Axes3D")


def apply_paper_style() -> None:
    """Apply publication-quality rcParams via seaborn base styles."""
    plt.style.use(["seaborn-v0_8-paper", "seaborn-v0_8-whitegrid"])
    plt.rcParams.update(
        {
            "savefig.dpi": 300,
            "figure.dpi": 100,
            "axes.titlesize": 11,
            "axes.labelsize": 10,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "legend.fontsize": 9,
            "legend.framealpha": 0.85,
            "lines.linewidth": 1.5,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )
