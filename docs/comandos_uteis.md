python -c "import open3d as o3d; p=o3d.io.read_point_cloud('data/output/zed_planes_indoor/colored_planes_00000.ply'); o3d.visualization.draw_geometries([p])"



ros2 run zed2i_data_acquisition zed_reconstruction_node --ros-args --params-file \
  ~/ufv/ros2_zed_ws/src/zed2i_data_acquisition/config/zed_reconstruction_config.yaml



/home/werikson/ufv/ros2_zed_ws/src/zed2i_data_acquisition/config

---

Editar o ~/ufv/ros2_zed_ws/src/zed2i_data_acquisition/config/zed_reconstruction_config.yaml e executar
ros2 run zed2i_data_acquisition zed_reconstruction_node --ros-args --params-file \
  ~/ufv/ros2_zed_ws/src/zed2i_data_acquisition/config/zed_reconstruction_config.yaml


ros2 bag play data/mcaps/reconstruction_experiment_20251214_122302/zed2i_recording/zed2i_recording_0.mcap --loop

python export_pointcloud2_to_ply.py   --topic /zed/zed_node/point_cloud/cloud_registered   --out_dir data/zed_ply   --max_frames 50   --every_k 10   --voxel 0.02

python scripts/segment_planes_open3d.py   --input data/zed_ply/my_room_01/   --out_dir outputs/my_room_01_p4i2500d004   --preset indoor   --max_planes 4   --min_inliers 2500   --distance_threshold 0.04

python -c "import open3d as o3d; p=o3d.io.read_point_cloud('output/my_room_01_p1/colored_planes_00000.ply'); o3d.visualization.draw_geometries([p])"






python - <<'PY'
import open3d as o3d

raw = o3d.io.read_point_cloud("data/zed_ply/my_room_01/frame_00044.ply")
seg = o3d.io.read_point_cloud("outputs/my_room_01_p4i2500d004/colored_planes_00044.ply")

print("raw points:", len(raw.points))
print("seg points:", len(seg.points))

o3d.visualization.draw_geometries([raw], window_name="RAW")
o3d.visualization.draw_geometries([seg], window_name="SEGMENTED")
PY


