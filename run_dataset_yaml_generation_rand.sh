python /home/innovation-hacking/luebbet/dev/IsaacLab/scripts/tng/utils/gen_dataset_yaml_rand.py \
  --name visuals_shoulder_front_static \
  --dim 0.175 \
  --num-random-episodes 300 \
  --threshold 0.1 \
  --seed 42 \
  --out_dir /home/innovation-hacking/luebbet/dev/IsaacLab/tng_datasets/ \
  --joint_randomization_range 15.0 \
  --table_height_randomization_range 0.075 \
  --objects_yaw_randomization_range 40.0 \
  --calibration_episodes \
  --camera_main "CAMERA_SHOULDER_POSE" \
  --camera_secondary "CAMERA_FRONT_POSE" \




