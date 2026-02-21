# Planes Processing Pipeline

This document describes the internal **planes** pipeline used in this
repository to segment, track, classify and analyze planar structures in 3D
point clouds.

The pipeline is implemented as a small library under `src/planes/` and a set
of CLI scripts under `scripts/`. Each script is a **thin wrapper** around a
`run_*` function in the corresponding module.

---

## 1. Overview

The planes pipeline is composed of five main stages:

1. **Segmentation**  
   Detect multiple planes per frame using Open3D RANSAC and store per-plane
   metrics and coarse per-frame classification (floor/table/wall).

2. **Tracking**  
   Track planes over time across frames, building temporal tracks for
   geometrically compatible planes.

3. **Stable classification**  
   Assign *global* semantic labels (floor, table, main wall) to tracks and
   propagate these labels back to per-frame detections.

4. **Recoloring**  
   Recolor raw point clouds using the stable semantic labels (floor/table/wall)
   with fixed RGB colors.

5. **Temporal stability analysis**  
   Analyze the temporal stability of semantic planes across the sequence
   (angles, offsets, centroids, inlier ratios), and optionally generate plots.

All heavy logic (geometry, data structures, algorithms) lives inside
`src/planes/`. The scripts in `scripts/` only handle **argparse, logging and
file I/O**.

---

## 2. Directory layout (relevant to planes)

Typical layout in this repository:

```text
configs/
data/
  zed_ply/
    my_room/
      frame_00000.ply
      frame_00001.ply
      ...
  mcaps/
docs/
notebooks/
output/
scripts/
  segment_planes_open3d.py
  track_planes_temporally.py
  classify_tracks_stably.py
  recolor_by_stable_class.py
  analyze_temporal_stability.py
src/
  planes/
    __init__.py
    geometry.py
    segmentation.py
    tracking.py
    classification.py
    recolor.py
    temporal_analysis.py
```

> **Note:**
> The pipeline itself is agnostic to the actual dataset name. It only expects:
>
> * input PLY frames: `frame_XXXXX.ply`, or
> * per-frame JSONs: `planes_XXXXX.json`.

---

## 3. Modules and responsibilities

### 3.1. `planes.geometry`

Generic geometric helpers:

* `safe_acos(x: float) -> float`
  Clamped `acos` for numerical robustness.
* `stabilize_normal(normal: np.ndarray) -> np.ndarray`
  Enforce a deterministic orientation convention for normals.
* `refit_plane_svd(points: np.ndarray) -> (normal, d)`
  Refits a plane in Hessian form using SVD.
* `point_plane_distances(points, normal, d) -> np.ndarray`
  Signed distances from points to a plane.
* `plane_extent_on_basis(points, normal) -> (dx, dy)`
  Planar extents on an orthonormal basis aligned with the plane.

Used by: segmentation, tracking, classification, temporal analysis.

---

### 3.2. `planes.segmentation` + `scripts/segment_planes_open3d.py`

**Goal:** multi-plane segmentation per frame using Open3D RANSAC.

Key types and functions:

* `SegConfig`
  Holds parameters such as:

  * `input_path: Path`
  * `out_dir: Path`
  * `preset: str` (`"indoor"` or `"outdoor"`)
  * `max_planes`, `min_inliers`
  * `distance_threshold`, `ransac_n`, `num_iterations`
  * `sor_nb_neighbors`, `sor_std_ratio`, `seed`

* `segment_planes(pcd, cfg, rng) -> (List[PlaneMetrics], colored_pcd)`
  Iterative plane extraction using RANSAC on the remaining points.

* `classify_floor_and_wall(planes: List[PlaneMetrics]) -> Dict`
  Coarse classification of floor-like, table-like and wall-like planes inside
  each frame.

* `process_frame(frame_path, frame_idx, cfg) -> Dict`
  Load, filter, segment, classify and write:

  * `colored_planes_XXXXX.ply`
  * `planes_XXXXX.json`

* `run_segmentation(cfg: SegConfig) -> Dict`
  High-level entry point: segment **all** frames in `cfg.input_path` and
  write a global `summary.json`.

**CLI script:**

```bash
python scripts/segment_planes_open3d.py \
  --input data/zed_ply/my_room \
  --out_dir output/planes/my_room \
  --preset indoor \
  --max_planes 4 \
  --min_inliers 1500
```

Outputs (per frame):

* `colored_planes_00000.ply`
* `planes_00000.json`

Global:

* `summary.json`

---

### 3.3. `planes.tracking` + `scripts/track_planes_temporally.py`

**Goal:** track planes across frames and build temporal tracks.

Key types and functions:

* `PlaneDetection`
  Single plane detection in a given frame.

* `PlaneTrack`
  Temporal track with exponential moving average for:

  * normal, offset `d` and centroid.

* `TrackerConfig`
  Parameters for association:

  * `max_angle_deg`, `max_centroid_dist`, `max_d_diff`
  * `max_frame_gap`, `min_track_length`
  * `ema_alpha`
  * cost weights: `w_angle`, `w_centroid`, `w_d`.

* `PlaneTracker`
  Greedy tracker using geometric compatibility and weighted cost.

* `load_detections(input_dir) -> Dict[int, List[PlaneDetection]]`
  Reads `planes_XXXXX.json` and groups detections per frame.

* `run_tracking(input_dir: Path, cfg: TrackerConfig) -> Dict`
  Main entry point:

  * runs the tracker over all frames,
  * returns a summary dict ready to be written as `plane_tracks.json`.

**CLI script:**

```bash
python scripts/track_planes_temporally.py \
  --input_dir output/planes/my_room \
  --output output/planes/my_room/plane_tracks.json \
  --max_angle_deg 15 \
  --max_centroid_dist 0.5 \
  --max_d_diff 0.7 \
  --min_track_length 3
```

Output:

* `plane_tracks.json`
  Contains the list of tracks and a frame-wise index mapping frames to
  `(track_id, local_id)` pairs.

---

### 3.4. `planes.classification` + `scripts/classify_tracks_stably.py`

**Goal:** assign *stable* semantic labels to tracks and propagate them
per frame.

Key types and functions:

* `ClassificationConfig`

  * `vertical_axis: (vx, vy, vz)`
  * `horiz_angle_max`
  * `min_table_height_diff`
  * `vertical_band_center`
  * `vertical_band_half_width`

* `run_classification(tracks_path: Path, cfg: ClassificationConfig) -> Dict`
  Reads `plane_tracks.json`, computes per-track metrics and selects:

  * `floor_track_id`
  * `table_track_id`
  * `main_wall_track_id`
    Then builds a per-frame map
    `"frame_idx" -> {floor, table, main_wall}` with `(track_id, plane_id)`.

**CLI script:**

```bash
python scripts/classify_tracks_stably.py \
  --tracks output/planes/my_room/plane_tracks.json \
  --output output/planes/my_room/classification_stable.json \
  --vertical_axis 1 0 0 \
  --horiz_angle_max 35.0 \
  --min_table_height_diff 0.20
```

Output:

* `classification_stable.json`
  Contains:

  * `selected_tracks` (IDs for floor/table/main_wall),
  * `tracks_summary` (per-track cached metrics),
  * `frames` (per-frame semantic assignments).

---

### 3.5. `planes.recolor` + `scripts/recolor_by_stable_class.py`

**Goal:** recolor raw point clouds according to stable semantic classes.

Key types and functions:

* `RecolorConfig`

  * `input_dir: Path` (with `frame_XXXXX.ply`),
  * `classification_path: Path` (`classification_stable.json`),
  * `out_dir: Path`,
  * `distance_threshold: float` (max point-plane distance).

* `run_recolor(cfg: RecolorConfig) -> Dict`
  For each frame:

  * load `frame_XXXXX.ply`,
  * read semantic roles from `classification_stable.json`,
  * reconstruct plane parameters for each track (floor/table/main_wall),
  * assign each point to the closest semantic plane within `distance_threshold`,
  * write `stable_semantic_XXXXX.ply`.

**CLI script:**

```bash
python scripts/recolor_by_stable_class.py \
  --input_dir data/zed_ply/my_room \
  --classification output/planes/my_room/classification_stable.json \
  --out_dir output/planes/my_room/semantic \
  --distance_threshold 0.04
```

Outputs:

* `stable_semantic_00000.ply`, `stable_semantic_00001.ply`, ...

Additionally, the script writes an optional summary:

* `recolor_summary.json`
  with counts of processed frames, missing input frames, etc.

---

### 3.6. `planes.temporal_analysis` + `scripts/analyze_temporal_stability.py`

**Goal:** analyze temporal stability of semantic planes.

Key types and functions:

* `PlaneTimeSeries`
  Holds time series for a class (frames, normals, offsets, centroids,
  inlier ratios).

* `TemporalStabilityMetrics`
  Aggregated metrics:

  * angle to first frame (mean, std, max),
  * per-frame angular/offset/centroid deltas,
  * inlier ratio statistics.

* `TemporalAnalysisConfig`

  * `input_dir: Path` (with `planes_XXXXX.json`),
  * `out_dir: Path` (for plots and summary),
  * `make_plots: bool`.

* `run_temporal_analysis(cfg: TemporalAnalysisConfig) -> Dict`
  Builds time series for:

  * Floor,
  * Table/Bed,
  * Main wall,
    computes metrics, optionally generates plots, and returns a summary
    dictionary.

**CLI script:**

```bash
python scripts/analyze_temporal_stability.py \
  --input_dir output/planes/my_room \
  --out_dir output/planes/my_room/stability
```

Outputs:

* `temporal_stability_summary.json`
* Plots:

  * `floor_orientation_offset.png`, `floor_deltas.png`,
  * `table_bed_orientation_offset.png`, `table_bed_deltas.png`,
  * `main_wall_orientation_offset.png`, `main_wall_deltas.png`
    (file names may vary slightly depending on slugified names).

---

## 4. End-to-end pipeline example

Assuming:

* input PLY frames in `data/zed_ply/my_room/`,
* outputs in `output/planes/my_room/`:

```bash
# 1) Segment planes per frame
python scripts/segment_planes_open3d.py \
  --input data/zed_ply/my_room \
  --out_dir output/planes/my_room \
  --preset indoor

# 2) Track planes temporally
python scripts/track_planes_temporally.py \
  --input_dir output/planes/my_room \
  --output output/planes/my_room/plane_tracks.json

# 3) Classify tracks stably (floor/table/main_wall)
python scripts/classify_tracks_stably.py \
  --tracks output/planes/my_room/plane_tracks.json \
  --output output/planes/my_room/classification_stable.json \
  --vertical_axis 1 0 0

# 4) Recolor original frames with semantic classes
python scripts/recolor_by_stable_class.py \
  --input_dir data/zed_ply/my_room \
  --classification output/planes/my_room/classification_stable.json \
  --out_dir output/planes/my_room/semantic

# 5) Analyze temporal stability
python scripts/analyze_temporal_stability.py \
  --input_dir output/planes/my_room \
  --out_dir output/planes/my_room/stability
```

---

## 5. Using the library API from Python

All CLI scripts are thin wrappers. You can directly call the underlying
functions from Python code or notebooks, for example:

```python
from pathlib import Path

from planes.segmentation import SegConfig, run_segmentation
from planes.tracking import TrackerConfig, run_tracking
from planes.classification import ClassificationConfig, run_classification
from planes.recolor import RecolorConfig, run_recolor
from planes.temporal_analysis import TemporalAnalysisConfig, run_temporal_analysis

# 1) Segmentation
seg_cfg = SegConfig(
    input_path=Path("data/zed_ply/my_room"),
    out_dir=Path("output/planes/my_room"),
    preset="indoor",
    max_planes=4,
    min_inliers=1500,
    distance_threshold=0.03,
    ransac_n=3,
    num_iterations=2000,
    sor_nb_neighbors=20,
    sor_std_ratio=2.0,
    seed=7,
)
seg_summary = run_segmentation(seg_cfg)

# 2) Tracking
trk_cfg = TrackerConfig(
    max_angle_deg=15.0,
    max_centroid_dist=0.5,
    max_d_diff=0.7,
    max_frame_gap=5,
    min_track_length=3,
    ema_alpha=0.8,
)
trk_summary = run_tracking(
    Path("output/planes/my_room"),
    trk_cfg,
)

# 3) Stable classification
cls_cfg = ClassificationConfig(
    vertical_axis=(1.0, 0.0, 0.0),
    horiz_angle_max=35.0,
    min_table_height_diff=0.20,
    vertical_band_center=90.0,
    vertical_band_half_width=30.0,
)
cls_payload = run_classification(
    Path("output/planes/my_room/plane_tracks.json"),
    cls_cfg,
)

# 4) Recolor
rec_cfg = RecolorConfig(
    input_dir=Path("data/zed_ply/my_room"),
    classification_path=Path("output/planes/my_room/classification_stable.json"),
    out_dir=Path("output/planes/my_room/semantic"),
    distance_threshold=0.04,
)
rec_summary = run_recolor(rec_cfg)

# 5) Temporal stability
tmp_cfg = TemporalAnalysisConfig(
    input_dir=Path("output/planes/my_room"),
    out_dir=Path("output/planes/my_room/stability"),
    make_plots=True,
)
tmp_summary = run_temporal_analysis(tmp_cfg)
```

This enables tight integration with notebooks and other processing
pipelines without going through CLI wrappers.



## 6. Exporting PointCloud2 to PLY (ROS 2 / MCAP workflow)

The planes pipeline operates on PLY frames (`frame_XXXXX.ply`).  
These frames are typically generated from a ROS 2 `PointCloud2` topic,
either:

- **Live**, directly from a running sensor node (e.g., ZED2i), or
- **Offline**, by replaying a recorded `.mcap` or `.db3` bag file.

The export step converts `sensor_msgs/msg/PointCloud2` into Open3D
`PointCloud` objects and writes them as `.ply` files.

---

### 6.1. Live export from a running ROS 2 system

If a sensor is publishing a `PointCloud2` topic (e.g. `/zed2i/point_cloud/cloud_registered`):

```bash
ros2 topic list
```

Then run:

```bash
python scripts/export_pointcloud2_to_ply.py \
  --topic /zed2i/point_cloud/cloud_registered \
  --out_dir data/zed_ply/my_room \
  --max_frames 50 \
  --every_k 10 \
  --voxel 0.02
```

Parameters:

* `--topic` : PointCloud2 topic name.
* `--out_dir` : Directory where `frame_XXXXX.ply` files will be written.
* `--max_frames` : Number of frames to export.
* `--every_k` : Save one frame every K received messages.
* `--voxel` : Optional voxel size (meters) for downsampling.
* `--keep_nans` : Disable NaN/Inf filtering.

Output:

```
data/zed_ply/my_room/
  frame_00000.ply
  frame_00001.ply
  ...
```

These frames can then be used as input to the segmentation stage.

---

### 6.2. Offline export from MCAP / rosbag2

If you have a recorded bag (e.g. `.mcap`):

```bash
ros2 bag info recording.mcap
```

Replay it:

```bash
ros2 bag play recording.mcap
```

In another terminal, run the exporter:

```bash
python scripts/export_pointcloud2_to_ply.py \
  --topic /zed/zed_node/point_cloud/cloud_registered \
  --out_dir data/zed_ply/my_room \
  --max_frames 50 \
  --every_k 10
  --voxel 0.03
```

The exporter subscribes to the replayed topic and writes PLY frames.

---

### 6.3. Internal architecture

The exporter is implemented in:

```
src/planes/ros_export.py
```

Key components:

* `ExportConfig` – configuration dataclass.
* `PointCloudExporter(Node)` – ROS 2 node.
* `run_export(cfg)` – high-level entry point.
* Zero-copy conversion using `np.frombuffer`.
* Structured dtype caching per message layout.
* Optional voxel downsampling via Open3D.

The CLI script:

```
scripts/export_pointcloud2_to_ply.py
```

is a thin wrapper around `run_export`.

---

### 6.4. Typical full pipeline (ROS → PLY → planes)

```bash
# 1) Play bag (or run live sensor)
ros2 bag play recording.mcap

# 2) Export PLY frames
python scripts/export_pointcloud2_to_ply.py \
  --topic /zed2i/point_cloud/cloud_registered \
  --out_dir data/zed_ply/my_room

# 3) Run planes pipeline
python scripts/segment_planes_open3d.py \
  --input data/zed_ply/my_room \
  --out_dir output/planes/my_room \
  --preset indoor
```

This separates:

* ROS acquisition,
* dataset materialization (PLY frames),
* geometric processing (planes pipeline).

```
