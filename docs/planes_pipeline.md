# Planes Processing Pipeline

This document explains the **planes pipeline** used in this repository to:

1. Segment planar structures in 3D point clouds.
2. Track planes over time.
3. Assign stable semantic labels (floor / table / main wall).
4. Recolor point clouds with fixed semantic colors.
5. Analyze temporal stability of these planes.

The pipeline is implemented as:

* A small library under `src/planes/`
* A set of CLI scripts under `scripts/`

Each script is a **thin wrapper** around a `run_*` function in the corresponding module (for example, `planes.segmentation.run_segmentation`).

---

## 1. High-level overview

The pipeline has five main stages:

1. **Segmentation (per frame)**
   Multi-plane detection using Open3D RANSAC. For each frame:

   * Detect multiple planes.
   * Compute per-plane metrics (normal, offset, extent, inliers, etc.).
   * Optionally assign a *coarse* per-frame semantic class (floor / table / wall).

2. **Temporal tracking**
   Associate geometrically compatible planes across frames and build **tracks** (time series of the same physical plane).

3. **Stable semantic classification**
   Use plane tracks to assign **global semantic labels** (floor, table/bed, main wall).
   These labels are then propagated back to each frame.

4. **Recoloring**
   Recolor raw point clouds using the **stable semantic labels**, with fixed RGB colors per class (e.g., floor always red, table always green, wall always blue).

5. **Temporal stability analysis**
   Analyze **how stable** the main semantic planes are over time:

   * Orientation drift (angles).
   * Offset variations (`d`).
   * Centroid variations.
   * Inlier ratios.
     Optionally generate plots.

All heavy logic (geometry, data structures, algorithms) lives inside `src/planes/`.
The scripts in `scripts/` handle only:

* `argparse`
* Logging
* File I/O

---

## 2. Data flow and directory layout

A typical repository layout (relevant to planes):

```text
configs/
data/
  bags/
    outdoor_01/
    ...
  mcaps/
    lab_01/
    ...
  ply/
    zed_outdoor_01/
      frame_00000.ply
      frame_00001.ply
      ...
    livox_outdoor_01/
      frame_00000.ply
      frame_00001.ply
      ...
    zed_lab_01/
      frame_00000.ply
      ...
docs/
output/
  planes/
    zed_outdoor_01/
      ply_stable_semantic/
        planes_00000.ply
        planes_00001.ply
        ...
      ply_colored/
        planes_00000.ply
        planes_00001.ply
        ...
      json/
        planes_00000.json
        planes_00001.json
        ...
      plane_tracks.json
      track_confidence.json
      classification_stable.json
      temporal_stability_summary.json
      summary.json
scripts/
  export_pointcloud2_to_ply.py
  segment_planes_open3d.py
  track_planes_temporally.py
  compute_track_confidence.py
  classify_tracks_stably.py
  recolor_by_stable_class.py
  analyze_temporal_stability.py
  run_planes_pipeline.py
src/
  planes/
    __init__.py
    ros_export.py
    geometry.py
    segmentation.py
    tracking.py
    classification.py
    recolor.py
    temporal_analysis.py
```

### Naming conventions

* **Input frames** (from ROS / bags):
  `frame_XXXXX.ply` in `data/ply/<experiment_name>/`.

* **Segmentation outputs** (per frame) in `output/planes/<base_id>_p<max_planes>/`:

  * `ply_colored/planes_XXXXX.ply` — per-plane colored PLY (RANSAC labels).
  * `json/planes_XXXXX.json` — per-frame JSON with plane parameters and metrics.
  * `summary.json` — global summary for the entire sequence.

* **Tracking & classification** in the same `planes` directory:

  * `plane_tracks.json` — temporal tracks of planes.
  * `track_confidence.json` — confidence metrics per track.
  * `classification_stable.json` — stable semantic assignment (floor/table/wall).

* **Recoloring outputs**:

  * Typically written either directly into `output/planes/...` or into a subfolder (e.g. `semantic/`), depending on the CLI arguments.
  * Files like `stable_semantic_XXXXX.ply`.

* **Temporal stability analysis outputs**:

  * `temporal_stability_summary.json`
  * Optional plots (`*_orientation_offset.png`, `*_deltas.png`, etc.)

> **Note:**
> The pipeline itself is **agnostic** to the dataset name. It only assumes a consistent naming pattern for:
>
> * PLY frames: `frame_XXXXX.ply`
> * Per-frame plane JSONs: `planes_XXXXX.json` (either in the root of a directory or inside `json/`).

---

## 3. Working with ROS1 and ROS2 bags

The pipeline works with point clouds originally recorded as:

* **ROS1 (`.bag`, e.g. Noetic)**
* **ROS2 rosbag2 (sqlite3, `.db3`)**
* **ROS2 rosbag2 (MCAP, `.mcap`)**

Internal processing is **ROS2-oriented**:

* We export `sensor_msgs/msg/PointCloud2` to PLY, then run the planes pipeline on the PLY dataset.
* ROS1 bags must be converted to ROS2-compatible rosbag2 before export.

### 3.1. Converting ROS1 `.bag` → ROS2 (sqlite3) with `rosbags`

We use the `rosbags` Python library for offline conversion (no ROS1 environment required).

**Step 1 — Install `rosbags`:**

```bash
pip install "rosbags>=0.9.11"
```

**Step 2 — Convert ROS1 `.bag` to ROS2 rosbag2 (sqlite3):**

```bash
rosbags-convert \
  --src flight_0.bag \
  --dst flight_0_ros2 \
  --dst-storage sqlite3 \
  --dst-version 5
```

This produces:

```text
flight_0_ros2/
  metadata.yaml
  flight_0_ros2_0.db3
```

**Step 3 — Validate the converted bag:**

```bash
source /opt/ros/humble/setup.bash
ros2 bag info flight_0_ros2
```

If no error appears, the bag is ready for ROS2 playback and PLY export.

### 3.2. Using ROS2 rosbag2 `.db3`

If your dataset is already in ROS2 sqlite3 format, no conversion is required.

To replay:

```bash
ros2 bag play flight_0_ros2 --clock
```

Then, in another terminal (from the project root and inside the venv), you can export PLY frames (see Section 4).

### 3.3. Using ROS2 rosbag2 (MCAP, `.mcap`)

If the bag is stored in MCAP format:

```text
flight_0_mcap/
  metadata.yaml
  flight_0_mcap_0.mcap
```

Playback is identical:

```bash
ros2 bag play flight_0_mcap --clock --loop
```

Again, the export step is the same — the exporter subscribes to a `PointCloud2` topic while the bag is playing.

### 3.4. Summary of supported input bag types

| Input Type   | Requires Conversion | Tool              |
| ------------ | ------------------- | ----------------- |
| ROS1 `.bag`  | ✅ Yes               | `rosbags-convert` |
| ROS2 `.db3`  | ❌ No                | Direct use        |
| ROS2 `.mcap` | ❌ No                | Direct use        |

### 3.5. Common bag-related issues

**Corrupted ROS1 bag**

If conversion fails with:

```text
Bag index looks damaged
```

You must reindex the bag using ROS1:

```bash
rosbag reindex corrupted_file.bag
```

Then retry:

```bash
rosbags-convert ...
```

---

## 4. Exporting `PointCloud2` to PLY (ROS2 / MCAP workflow)

The planes pipeline operates on PLY frames:

```text
frame_00000.ply
frame_00001.ply
...
```

These frames are typically generated from a ROS2 `sensor_msgs/msg/PointCloud2` topic, either:

* **Live** — directly from a running sensor node (e.g. ZED2i, Livox), or
* **Offline** — by replaying a recorded `.db3` or `.mcap` rosbag2.

The exporter converts `PointCloud2` messages to Open3D `PointCloud` objects and writes them as `.ply`.

### 4.1. Live export from a running ROS2 system

If a sensor is publishing a `PointCloud2` topic, for example:

```bash
ros2 topic list
# ...
# /zed/zed_node/point_cloud/cloud_registered
# /zed2i/zed_node/point_cloud/cloud_registered
# /livox/lidar
```

You can run:

```bash
python scripts/export_pointcloud2_to_ply.py \
  --topic /zed/zed_node/point_cloud/cloud_registered \
  --out_dir data/ply/zed_outdoor_01 \
  --max_frames 75 \
  --every_k 5 \
  --voxel 0.02
```

Parameters:

* `--topic` : `PointCloud2` topic name.
* `--out_dir` : Output directory for `frame_XXXXX.ply`.
* `--max_frames` : Maximum number of frames to export.
* `--every_k` : Save one frame every K received messages.
* `--voxel` : Optional voxel size (meters) for downsampling.
* `--keep_nans` : If set, disable NaN/Inf filtering.

Output:

```text
data/ply/zed_outdoor_01/
  frame_00000.ply
  frame_00001.ply
  ...
```

These PLYs are the input for the segmentation stage.

### 4.2. Offline export from rosbag2 (`.db3` / `.mcap`)

For a recorded bag (e.g. MCAP):

```bash
ros2 bag info recording.mcap
```

Replay it:

```bash
ros2 bag play recording.mcap --clock
```

In another terminal:

```bash
python scripts/export_pointcloud2_to_ply.py \
  --topic /zed/zed_node/point_cloud/cloud_registered \
  --out_dir data/ply/zed_outdoor_01 \
  --max_frames 50 \
  --every_k 10 \
  --voxel 0.03
```

The exporter subscribes to the topic being replayed and writes the PLY frames.

### 4.3. Internal architecture of the exporter

The exporter lives in:

```text
src/planes/ros_export.py
```

Key components:

* `ExportConfig` – configuration dataclass.
* `PointCloudExporter(Node)` – ROS2 node that subscribes to `PointCloud2`.
* `run_export(cfg)` – high-level entry point.
* Efficient conversion using `np.frombuffer` and cached dtypes.
* Optional voxel downsampling via Open3D.

The CLI script:

```text
scripts/export_pointcloud2_to_ply.py
```

is a thin wrapper around `run_export`.

### 4.4. Typical ROS → PLY → planes workflow

```bash
# 1) Play bag (or start live sensor)
ros2 bag play recording.mcap --clock  # or .db3

# 2) Export PLY frames
python scripts/export_pointcloud2_to_ply.py \
  --topic /zed/zed_node/point_cloud/cloud_registered \
  --out_dir data/ply/zed_outdoor_01

# 3) Run plane segmentation and subsequent stages
python scripts/segment_planes_open3d.py \
  --input data/ply/zed_outdoor_01 \
  --out_dir output/planes/my_room_01_p8 \
  --preset indoor
```

This clearly separates:

1. ROS acquisition,
2. Dataset materialization (PLY),
3. Geometric processing (planes pipeline).

---

## 5. Modules and responsibilities

This section describes each module in `src/planes/` and the corresponding CLI script(s).

### 5.1. `planes.geometry`

Generic geometric utilities, reused across the pipeline:

* `safe_acos(x: float) -> float`
  `acos` with clamping to avoid NaNs from slight numerical drift.

* `stabilize_normal(normal: np.ndarray) -> np.ndarray`
  Enforce a deterministic normal orientation convention.

* `refit_plane_svd(points: np.ndarray) -> (normal, d)`
  Refits a plane in Hessian form (`n^T x + d = 0`) using SVD.

* `point_plane_distances(points, normal, d) -> np.ndarray`
  Signed distances from points to a plane.

* `plane_extent_on_basis(points, normal) -> (dx, dy)`
  Planar extents on an orthonormal basis aligned with the plane.

Used by: segmentation, tracking, classification, and temporal analysis.

---

### 5.2. `planes.segmentation` + `scripts/segment_planes_open3d.py`

**Goal:** multi-plane segmentation per frame using Open3D RANSAC.

Key types and functions:

* `SegConfig` – configuration dataclass. Main fields:

  * `input_path: Path`
  * `out_dir: Path`
  * `preset: str` (`"indoor"` or `"outdoor"`)
  * `max_planes`, `min_inliers`
  * `distance_threshold`, `ransac_n`, `num_iterations`
  * `sor_nb_neighbors`, `sor_std_ratio`, `seed`

* `segment_planes(pcd, cfg, rng) -> (List[PlaneMetrics], colored_pcd)`
  Iteratively extracts planes with RANSAC from the remaining points.

* `classify_floor_and_wall(planes: List[PlaneMetrics]) -> Dict`
  Lightweight per-frame classification of candidate floor/table/wall planes.

* `process_frame(frame_path, frame_idx, cfg) -> Dict`
  For a single `frame_XXXXX.ply`:

  * Load and optionally filter.
  * Segment planes.
  * Compute metrics and coarse classes.
  * Write:

    * `ply_colored/planes_XXXXX.ply`
    * `json/planes_XXXXX.json`

* `run_segmentation(cfg: SegConfig) -> Dict`
  Process all frames in `cfg.input_path` and write `summary.json` at the root of `cfg.out_dir`.

**CLI usage:**

```bash
python scripts/segment_planes_open3d.py \
  --input data/ply/zed_outdoor_01 \
  --out_dir output/planes/my_room_01_p8 \
  --preset indoor \
  --max_planes 4 \
  --min_inliers 1500
```

Outputs:

* Per-frame:

  * `ply_colored/planes_00000.ply`
  * `json/planes_00000.json`
* Global:

  * `summary.json`

---

### 5.3. `planes.tracking` + `scripts/track_planes_temporally.py`

**Goal:** track planes across frames and build temporal plane tracks.

Key types and functions:

* `PlaneDetection`
  Represents a single plane detection in a given frame.

* `PlaneTrack`
  Accumulates a time series for one physical plane, with exponential moving averages for:

  * plane normal,
  * offset `d`,
  * centroid.

* `TrackerConfig`
  Defines association thresholds and weights:

  * `max_angle_deg`, `max_centroid_dist`, `max_d_diff`
  * `max_frame_gap`, `min_track_length`
  * `ema_alpha`
  * `w_angle`, `w_centroid`, `w_d` (cost weights)

* `PlaneTracker`
  Greedy tracker that associates planes based on geometric compatibility and weighted cost.

* `load_detections(input_dir) -> Dict[int, List[PlaneDetection]]`
  Reads `planes_XXXXX.json` and groups detections per frame.
  Supports both layouts:

  * `input_dir/planes_XXXXX.json`
  * `input_dir/json/planes_XXXXX.json`

* `run_tracking(input_dir: Path, cfg: TrackerConfig) -> Dict`
  Runs the tracker over all frames and returns a summary dict that is written as `plane_tracks.json`.

**CLI usage:**

```bash
python scripts/track_planes_temporally.py \
  --input_dir output/planes/my_room_01_p8 \
  --output output/planes/my_room_01_p8/plane_tracks.json \
  --max_angle_deg 15 \
  --max_centroid_dist 0.5 \
  --max_d_diff 0.7 \
  --min_track_length 3
```

Output:

* `plane_tracks.json`
  Contains the list of tracks and a frame-wise index mapping from frame indices to `(track_id, local_plane_id)` pairs.

---

### 5.4. `planes.classification` + `scripts/classify_tracks_stably.py`

**Goal:** assign **stable semantic labels** to tracks and propagate them per frame.

Key elements:

* `ClassificationConfig`, including parameters such as:

  * `vertical_axis: (vx, vy, vz)` — sensor-specific gravity direction.
  * `horiz_angle_max` — max angle for considering a plane “horizontal”.
  * `min_table_height_diff` — minimum height difference between floor and table.
  * `vertical_band_center`, `vertical_band_half_width` — controls which planes can be considered vertical.

* `run_classification(tracks_path: Path, cfg: ClassificationConfig) -> Dict`
  Reads `plane_tracks.json`, computes per-track metrics, and selects:

  * `floor_track_id`
  * `table_track_id`
  * `main_wall_track_id`

  It then builds a per-frame map:

  ```json
  "frames": {
    "0": { "floor": {...}, "table": {...}, "main_wall": {...} },
    "1": { ... },
    ...
  }
  ```

**CLI usage:**

```bash
python scripts/classify_tracks_stably.py \
  --tracks output/planes/my_room_01_p8/plane_tracks.json \
  --output output/planes/my_room_01_p8/classification_stable.json \
  --vertical_axis 1 0 0 \
  --horiz_angle_max 35.0 \
  --min_table_height_diff 0.20
```

Output:

* `classification_stable.json`, containing:

  * `selected_tracks` (IDs for floor / table / main wall),
  * `tracks_summary` (cached per-track metrics),
  * `frames` (per-frame semantic assignments).

---

### 5.5. `planes.recolor` + `scripts/recolor_by_stable_class.py`

**Goal:** recolor raw point clouds according to stable semantic classes.

Key elements:

* `RecolorConfig`:

  * `input_dir: Path` (directory with `frame_XXXXX.ply`),
  * `classification_path: Path` (`classification_stable.json`),
  * `out_dir: Path`,
  * `distance_threshold: float` (max distance from a semantic plane).

* `run_recolor(cfg: RecolorConfig) -> Dict`
  For each frame:

  1. Load `frame_XXXXX.ply`.
  2. Retrieve the semantic roles (floor / table / wall) from `classification_stable.json`.
  3. Reconstruct plane parameters for each semantic track.
  4. Assign each point to the closest semantic plane within `distance_threshold`.
  5. Assign a fixed RGB color per class.
  6. Save:

     * `stable_semantic_XXXXX.ply` (or into a subfolder, depending on `out_dir`).

**CLI usage:**

```bash
python scripts/recolor_by_stable_class.py \
  --input_dir data/ply/zed_outdoor_01 \
  --classification output/planes/my_room_01_p8/classification_stable.json \
  --out_dir output/planes/my_room_01_p8/semantic \
  --distance_threshold 0.04
```

Outputs:

* `semantic/stable_semantic_00000.ply`, `semantic/stable_semantic_00001.ply`, …

Optional summary:

* `recolor_summary.json` — counts processed frames, missing frames, etc.

---

### 5.6. `planes.temporal_analysis` + `scripts/analyze_temporal_stability.py`

**Goal:** analyze the **temporal stability** of semantic planes.

Key elements:

* `PlaneTimeSeries`
  Time series for one semantic class (frames, normals, offsets, centroids, inlier ratios).

* `TemporalStabilityMetrics`
  Aggregated metrics such as:

  * Mean / std / max angle to the first frame,
  * Per-frame angular / offset / centroid deltas,
  * Inlier ratio statistics.

* `TemporalAnalysisConfig`:

  * `input_dir: Path` (directory with `planes_XXXXX.json`, usually `.../json/`),
  * `out_dir: Path`,
  * `make_plots: bool`.

* `run_temporal_analysis(cfg: TemporalAnalysisConfig) -> Dict`
  Builds time series for:

  * Floor,
  * Table/Bed,
  * Main wall,
    computes metrics and optionally generates plots.

**CLI usage:**

```bash
python scripts/analyze_temporal_stability.py \
  --input_dir output/planes/my_room_01_p8/json \
  --out_dir output/planes/my_room_01_p8/stability
```

Outputs:

* `temporal_stability_summary.json`
* Plots such as:

  * `floor_orientation_offset.png`, `floor_deltas.png`
  * `table_bed_orientation_offset.png`, `table_bed_deltas.png`
  * `main_wall_orientation_offset.png`, `main_wall_deltas.png`
    (exact names may vary slightly).

---

## 6. End-to-end examples

### 6.1. Manual step-by-step pipeline

Assuming:

* PLY frames in `data/ply/zed_outdoor_01/`
* Outputs in `output/planes/my_room_01_p8/`

```bash
# 1) Segment planes per frame
python scripts/segment_planes_open3d.py \
  --input data/ply/zed_outdoor_01 \
  --out_dir output/planes/my_room_01_p8 \
  --preset indoor

# 2) Track planes temporally
python scripts/track_planes_temporally.py \
  --input_dir output/planes/my_room_01_p8 \
  --output output/planes/my_room_01_p8/plane_tracks.json

# 3) Compute track confidence (optional but recommended)
python scripts/compute_track_confidence.py \
  --tracks output/planes/my_room_01_p8/plane_tracks.json \
  --output output/planes/my_room_01_p8/track_confidence.json

# 4) Classify tracks stably (floor/table/main_wall)
python scripts/classify_tracks_stably.py \
  --tracks output/planes/my_room_01_p8/plane_tracks.json \
  --confidence output/planes/my_room_01_p8/track_confidence.json \
  --output output/planes/my_room_01_p8/classification_stable.json \
  --vertical_axis 1 0 0

# 5) Recolor original frames with semantic classes
python scripts/recolor_by_stable_class.py \
  --input_dir data/ply/zed_outdoor_01 \
  --classification output/planes/my_room_01_p8/classification_stable.json \
  --out_dir output/planes/my_room_01_p8/semantic \
  --distance_threshold 0.04

# 6) Analyze temporal stability (using the JSON planes)
python scripts/analyze_temporal_stability.py \
  --input_dir output/planes/my_room_01_p8/json \
  --out_dir output/planes/my_room_01_p8/stability
```

---

## 7. Orchestrating the pipeline with `run_planes_pipeline.py`

For convenience, there is a single orchestrator script:

```text
scripts/run_planes_pipeline.py
```

Given an experiment folder under `data/ply/` (for example, `home_02_T1_90_05_02`), it:

1. Runs plane segmentation (`segment_planes_open3d.py`).
2. Runs temporal tracking (`track_planes_temporally.py`).
3. Computes track confidence (`compute_track_confidence.py`).
4. Classifies tracks stably (`classify_tracks_stably.py`).
5. Recolors PLY frames (`recolor_by_stable_class.py`).
6. Optionally analyzes temporal stability (`analyze_temporal_stability.py`).

It assumes:

* PLY frames are already exported at:
  `data/ply/<experiment_name>/`
* The **base id** used for outputs is composed of the first three components of the experiment name, e.g.:

  ```text
  experiment_name = "home_02_T1_90_05_02"
  base_id = "home_02_T1"
  planes_dir = output/planes/home_02_T1_p8
  ```

### 7.1. Basic usage

```bash
python scripts/run_planes_pipeline.py \
  --experiment home_02_T1_90_05_02 \
  --max_planes 8
```

Optional arguments:

* `--topic` — only documented for traceability (export topic), not used internally.
* `--no_temporal_analysis` — skip the temporal stability analysis step.

Internally, the script:

* Calls `derive_paths(...)` to build:

  * `ply_dir` (input PLY directory),
  * `planes_dir` (output directory for planes),
  * `json_planes_dir` (usually `planes_dir / "json"`),
  * `tracks_path`, `confidence_path`, `classification_path`.
* Runs the CLI scripts via `subprocess.run`, preserving the same behavior as when they are called manually.

This enables tight integration with notebooks and other pipelines: you can either run each stage manually or use `run_planes_pipeline.py` as a one-command entry point for an entire experiment.

---

This concludes the overview of the planes processing pipeline.
