#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import json
from pathlib import Path
from dataclasses import asdict

from planes.track_confidence import compute_track_confidence


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Compute confidence score for plane tracks."
    )
    parser.add_argument(
        "--tracks",
        required=True,
        help="plane_tracks.json file",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output JSON file for confidence metrics",
    )
    return parser.parse_args()


def main():
    args = _parse_args()

    tracks_path = Path(args.tracks).resolve()
    output_path = Path(args.output).resolve()

    with tracks_path.open("r", encoding="utf-8") as f:
        tracks_data = json.load(f)

    total_frames = len(tracks_data.get("frames", {}))

    metrics = compute_track_confidence(
        tracks_data=tracks_data,
        total_frames=total_frames,
    )

    metrics_sorted = sorted(
        metrics,
        key=lambda m: m.confidence_score,
        reverse=True,
    )

    payload = {
        "tracks_file": str(tracks_path),
        "total_frames": total_frames,
        "tracks_confidence": [asdict(m) for m in metrics_sorted],
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    print(f"[INFO] Confidence metrics written to: {output_path}")


if __name__ == "__main__":
    main()
