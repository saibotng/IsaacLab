dataset_yaml_paths=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_datasets/kinematics_combined.yaml
)
parallel_envs=16
for dataset_yaml_path in "${dataset_yaml_paths[@]}"; do
    dataset_name=$(basename $dataset_yaml_path .yaml)
    working_dir=/home/innovation-hacking/luebbet/dev/important_datasets/pipeline/$dataset_name

    mkdir -p $working_dir
    /home/innovation-hacking/luebbet/venvs/isaaclab_luebbet/bin/python /home/innovation-hacking/luebbet/dev/IsaacLab/scripts/tng/utils/visualize_dataset_distr.py \
        --in $dataset_yaml_path \
        --pad 0.05 \
        --out-dir $working_dir
    /home/innovation-hacking/luebbet/venvs/isaaclab_luebbet/bin/python /home/innovation-hacking/luebbet/dev/IsaacLab/scripts/tng/ur5_pick_and_place_sm.py \
        --headless \
        --enable_cameras \
        --blackwell \
        --from_yaml $dataset_yaml_path \
        --recorder_dir $working_dir \
        --num_envs $parallel_envs
    /home/innovation-hacking/luebbet/venvs/lerobot_editable/bin/python /home/innovation-hacking/luebbet/dev/lerobot/src/tng/convert_hdf5_to_lerobot.py \
        --dataset-root $working_dir \
        --dataset-name "luebbet/$dataset_name" \
        --hdf5-files datagen_recordings.hdf5 \
        --push-to-hub \
        --commit-message "first commit" \
        --compute-stats 
    /home/innovation-hacking/luebbet/venvs/isaaclab_luebbet/bin/python /home/innovation-hacking/luebbet/dev/IsaacLab/scripts/tng/utils/push_to_aws.py \
        $working_dir \
        s3://tng-luebbe-master-arbeit/Datasets/$dataset_name

    echo "Finished dataset: $dataset_name"
    echo "----------------------------------------"
done
  