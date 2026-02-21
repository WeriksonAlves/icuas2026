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
