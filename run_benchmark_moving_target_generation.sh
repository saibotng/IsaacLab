python /home/innovation-hacking/luebbet/dev/IsaacLab/scripts/tng/utils/gen_benchmark_moving_target.py \
  --name benchmark_visuals_dynamic_side_shoulder \
  --dim 0.175 \
  --resolution 8 \
  --seed 21 \
  --threshold 0.1 \
  --out_dir /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/ \
  --joint_randomization_range 10.0 \
  --table_height_randomization_range 0.05 \
  --objects_yaw_randomization_range 30.0 \
  --camera_main "CAMERA_SIDE_POSE" \
  --camera_secondary "CAMERA_SHOULDER_POSE" \
  --randomize_camera_poses


