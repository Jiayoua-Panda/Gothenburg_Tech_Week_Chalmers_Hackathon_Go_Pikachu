"""Plot measured upper-corridor differences for the 0.8 m/s centered pilot."""

from __future__ import annotations

import gzip
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
ARTIFACTS = ROOT / "artifacts/stair-policy-switch"
COLORS = {"hybrid": "#007f86", "dwaq-fast": "#d05a28"}
LABELS = {"hybrid": "Left video: DWAQ to V0", "dwaq-fast": "Right video: DWAQ faster"}


def moving_mean(values: np.ndarray, window: int = 11) -> np.ndarray:
    left = window // 2
    padded = np.pad(values, (left, window - left - 1), mode="edge")
    return np.convolve(padded, np.ones(window) / window, mode="valid")


def load_segment(mode: str) -> dict[str, np.ndarray]:
    path = ARTIFACTS / f"{mode}_cmd80cmps_center-trace.json.gz"
    with gzip.open(path, "rt", encoding="utf-8") as source:
        trace = json.load(source)
    segment = [point for point in trace if 7.0 <= point["x"] <= 10.4]
    return {key: np.array([point[key] for point in segment]) for key in ("x", "y", "z", "t")}


def main() -> None:
    plt.rcParams.update({"font.size": 11, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    fig.subplots_adjust(left=0.12, right=0.97, top=0.88, bottom=0.12, hspace=0.34)
    for mode in ("hybrid", "dwaq-fast"):
        segment = load_segment(mode)
        x, y, z, t = (segment[key] for key in ("x", "y", "z", "t"))
        speed = np.diff(x) / np.diff(t)
        color = COLORS[mode]
        axes[0].plot(x[1:], speed, color=color, alpha=0.18, linewidth=0.7)
        axes[0].plot(x[1:], moving_mean(speed), color=color, linewidth=2.2,
                     label=LABELS[mode])
        height_cm = (z - z.mean()) * 100
        axes[1].plot(x, height_cm, color=color, alpha=0.18, linewidth=0.7)
        axes[1].plot(x, moving_mean(height_cm), color=color, linewidth=2.2)
        axes[2].plot(x, y * 100, color=color, linewidth=1.9,
                     label=LABELS[mode])

    labels = ("Forward speed (m/s)", "Pelvis height from run mean (cm)", "Lateral offset from center (cm)")
    for axis, label in zip(axes, labels):
        axis.set_ylabel(label)
        axis.grid(alpha=0.2)
    axes[0].set_title("Mean / SD: hybrid 0.735 / 0.032; DWAQ faster 0.713 / 0.059 m/s", loc="left", fontsize=11)
    axes[1].set_title("Height SD: hybrid 0.322 cm; DWAQ faster 0.384 cm", loc="left", fontsize=11)
    axes[2].set_title("Maximum absolute lateral offset: hybrid 6.41 cm; DWAQ faster 9.20 cm", loc="left", fontsize=11)
    axes[2].axhline(0, color="#777777", linewidth=0.8, linestyle="--")
    axes[2].set_xlim(7.0, 10.4)
    axes[2].set_xlabel("Pelvis position along upper corridor, x (m)")
    fig.suptitle("Post-stair walking, 0.8 m/s command", y=0.99, fontsize=18, weight="bold")
    fig.legend(*axes[0].get_legend_handles_labels(), loc="upper center", bbox_to_anchor=(0.5, 0.953),
               ncol=2, frameon=False, fontsize=11)
    fig.text(0.5, 0.02,
             "Same centered start and 15 cm stairs. Thin curves are raw 50 Hz samples; bold curves are 11-sample moving means.",
             ha="center", fontsize=9, color="#555555")
    output = ARTIFACTS / "upper_corridor_metrics_cmd80cmps.png"
    fig.savefig(output, dpi=160, facecolor="white")
    print(output)


if __name__ == "__main__":
    main()
