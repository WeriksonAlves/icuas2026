#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# scripts/analyze_temporal_stability.py
"""CLI front-end for temporal stability analysis of segmented planes."""

from __future__ import annotations

import argparse
import json
from typing import Optional

from planes.temporal_analysis import (
    TemporalAnalysisConfig,
    run_temporal_analysis,
)


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Temporal stability analysis for segmented planes.",
    )
    parser.add_argument(
        "--input_dir",
        required=True,
        help="Directory containing planes_XXXXX.json files.",
    )
    parser.add_argument(
        "--out_dir",
        required=True,
        help="Output directory for summary JSON and plots.",
    )
    parser.add_argument(
        "--no_plots",
        action="store_true",
        help="Disable plot generation.",
    )
    return parser


def _fmt(value: Optional[float]) -> str:
    """Pretty-format optional float values."""
    if value is None or (isinstance(value, float) and not value == value):
        # None or NaN
        return "n/a"
    return f"{value:.4f}"


def main() -> None:
    parser = _build_arg_parser()
    args = parser.parse_args()

    cfg = TemporalAnalysisConfig.from_cli_args(args)
    cfg.out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Input dir: {cfg.input_dir}")
    print(f"[INFO] Output dir: {cfg.out_dir}")
    print(f"[INFO] Plots enabled: {cfg.make_plots}")

    summary = run_temporal_analysis(cfg)

    # Human-readable logs
    series_dict = summary.get("series", {})
    for name, metrics in series_dict.items():
        num_samples = metrics.get("num_samples", 0)
        mean_a1 = metrics.get("mean_angle_to_first_deg")
        max_da = metrics.get("max_delta_angle_deg")

        print(
            f"[INFO] {name}: "
            f"samples={num_samples}, "
            f"mean_angle_to_first={_fmt(mean_a1)} deg, "
            f"max_delta_angle={_fmt(max_da)} deg"
        )

    # Save summary JSON
    summary_path = cfg.out_dir / "temporal_stability_summary.json"
    with summary_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print(f"[INFO] Summary written to: {summary_path}")


if __name__ == "__main__":
    main()
