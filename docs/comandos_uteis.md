python -c "import open3d as o3d; p=o3d.io.read_point_cloud('data/output/zed_planes_indoor/colored_planes_00000.ply'); o3d.visualization.draw_geometries([p])"



ros2 run zed2i_data_acquisition zed_reconstruction_node --ros-args --params-file \
  ~/ufv/ros2_zed_ws/src/zed2i_data_acquisition/config/zed_reconstruction_config.yaml



/home/werikson/ufv/ros2_zed_ws/src/zed2i_data_acquisition/config

---

Editar o ~/ufv/ros2_zed_ws/src/zed2i_data_acquisition/config/zed_reconstruction_config.yaml e executar
ros2 run zed2i_data_acquisition zed_reconstruction_node --ros-args --params-file \
  ~/ufv/ros2_zed_ws/src/zed2i_data_acquisition/config/zed_reconstruction_config.yaml


ros2 bag play data/mcaps/reconstruction_experiment_20251214_122302/zed2i_recording/zed2i_recording_0.mcap --loop

python - <<'PY'
import open3d as o3d

raw = o3d.io.read_point_cloud("data/zed_ply/my_room/frame_00049.ply")
# seg = o3d.io.read_point_cloud("output/planes/my_room/colored_planes_00049.ply")
seg = o3d.io.read_point_cloud("output/planes/my_room/semantic/stable_semantic_00049.ply")

print("raw points:", len(raw.points))
print("seg points:", len(seg.points))

o3d.visualization.draw_geometries([raw], window_name="RAW")
o3d.visualization.draw_geometries([seg], window_name="SEGMENTED")
PY


python scripts/analyze_temporal_stability.py  --input_dir outputs/my_room_01_p4i2500d004_3  --out_dir outputs/my_room_01_stability


python scripts/track_planes_temporally.py  --input_dir outputs/my_room_01_p4i2500d004_3  --output outputs/my_room_01_plane_tracks.json


python scripts/classify_tracks_stably.py  --tracks outputs/my_room_01_plane_tracks.json  --output outputs/my_room_01_classification_stable.json  --vertical_axis 1 0 0


python scripts/recolor_by_stable_class.py  --input_dir data/zed_ply/my_room_01  --classification outputs/my_room_01_classification_stable.json  --out_dir outputs/my_room_01_stable_semantic  --distance_threshold 0.04


# 1) Tracking
python scripts/track_planes_temporally.py \
  --input_dir output/planes/my_room \
  --output output/planes/plane_tracks.json

# 2) Confidence
python scripts/compute_track_confidence.py \
  --tracks output/planes/my_room/plane_tracks.json \
  --output output/planes/my_room/track_confidence.json

# 3) Classificação estável (agora chamando o módulo em src/planes)
python scripts/classify_tracks_stably.py \
  --tracks output/planes/my_room/plane_tracks.json \
  --confidence output/planes/my_room/track_confidence.json \
  --output output/planes/my_room/classification_stable_2.json \
  --vertical_axis 1 0 0 \
  --min_confidence 0.3

# 4) Recolorização semântica (como já está hoje)
python scripts/recolor_by_stable_class.py \
  --input_dir data/zed_ply/my_room \
  --classification output/planes/my_room/classification_stable_2.json \
  --out_dir output/planes/my_room/stable_semantic_2 \
  --distance_threshold 0.04


python scripts/run_planes_pipeline.py --experiment home_02_T2_75_05_02 --max_planes 8