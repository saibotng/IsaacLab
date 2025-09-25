#!/bin/bash

# Usage: ./automate_single_experiment.sh <experiment_name> <dataset_name> <data_config> <max_steps> <runs_per_benchmark> [benchmark1] [benchmark2] ...
# Example: ./automate_single_experiment.sh dataset_size_30_10k base_dataset_30 "tng_ur5_AbsJointState_DeltaJointAction_2Cams" 10000 3 benchmark_easy_default_cams.yaml benchmark_hard_default_cams.yaml

if [ $# -lt 5 ]; then
    echo "Usage: $0 <experiment_name> <dataset_name> <data_config> <max_steps> <runs_per_benchmark> [benchmark1] [benchmark2] ..."
    echo "Example: $0 dataset_size_30_10k base_dataset_30 \"tng_ur5_AbsJointState_DeltaJointAction_2Cams\" 10000 3 benchmark_easy_default_cams.yaml benchmark_hard_default_cams.yaml"
    exit 1
fi

experiment_name="$1"
dataset_name="$2"
data_config="$3"
max_steps="$4"
runs_per_benchmark="$5"

# Parse benchmark files from remaining arguments
benchmarks=()
shift 5  # Remove the first 5 arguments
for benchmark in "$@"; do
    # If it's just a filename, prepend the full path
    if [[ "$benchmark" != /* ]]; then
        benchmark="/home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/$benchmark"
    fi
    benchmarks+=("$benchmark")
done

# Default benchmarks if none provided
if [ ${#benchmarks[@]} -eq 0 ]; then
    benchmarks=(
        /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_easy_default_cams.yaml
        /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_hard_default_cams.yaml
    )
fi

set -euo pipefail


dataset_dir=/home/innovation-hacking/luebbet/dev/important_datasets/pipeline/$dataset_name/
simple_trajectory_dataset_dir=/home/innovation-hacking/luebbet/dev/important_datasets/pipeline/simple_trajectory/
batch_size=16
save_steps=1000000
parallel_envs=10
experiment_dir=/home/innovation-hacking/luebbet/experiments/pipeline/$experiment_name
simple_trajectory_dataset_name=$(basename $simple_trajectory_dataset_dir)
output_dir=$experiment_dir/models_$data_config


mkdir -p $experiment_dir
mkdir -p "$experiment_dir/data_visualization"
cp "$dataset_dir"/*.png "$experiment_dir"/data_visualization/
cp "$dataset_dir"/*.yaml "$experiment_dir"/data_visualization/
/home/innovation-hacking/luebbet/venvs/combined_robots_luebbet/bin/python /home/innovation-hacking/luebbet/dev/Isaac-GR00T/scripts/gr00t_finetune.py \
    --dataset-path $dataset_dir/lerobot_luebbet_$dataset_name \
    --output-dir $output_dir \
    --max-steps $max_steps \
    --data-config $data_config \
    --embodiment-tag new_embodiment \
    --video-backend torchvision_av \
    --batch-size $batch_size \
    --no-tune-llm \
    --no-tune-visual \
    --tune-projector \
    --tune-diffusion-model \
    --report-to tensorboard \
    --save-steps $save_steps \

echo "Looking for checkpoints in: $output_dir"

for checkpoint_dir in "$output_dir"/checkpoint*/; do
    if [ -d "$checkpoint_dir" ]; then
        echo "Found checkpoint directory: $checkpoint_dir"
        /home/innovation-hacking/luebbet/venvs/combined_robots_luebbet/bin/python /home/innovation-hacking/luebbet/dev/Isaac-GR00T/scripts/eval_policy.py \
        --plot \
        --dataset_path $simple_trajectory_dataset_dir/lerobot_luebbet_$simple_trajectory_dataset_name \
        --model_path $checkpoint_dir \
        --embodiment_tag new_embodiment \
        --data_config $data_config \
        --video_backend torchvision_av \
        --modality_keys delta_robot_arm delta_gripper \
        --save_plot_path $checkpoint_dir/plot_ood \
        --trajs 1
        /home/innovation-hacking/luebbet/venvs/combined_robots_luebbet/bin/python /home/innovation-hacking/luebbet/dev/Isaac-GR00T/scripts/eval_policy.py \
        --plot \
        --dataset-path $dataset_dir/lerobot_luebbet_$dataset_name \
        --model_path $checkpoint_dir \
        --embodiment_tag new_embodiment \
        --data_config $data_config \
        --video_backend torchvision_av \
        --modality_keys delta_robot_arm delta_gripper \
        --save_plot_path $checkpoint_dir/plot_id \
        --trajs 1
    fi
done


# Run benchmarks using the dedicated benchmark script



# Call the separate benchmark script
# Set record_inference to false by default (change to true if you want to record)
record_inference="false"
/home/innovation-hacking/luebbet/dev/IsaacLab/automate_benchmark_runs.sh \
    "$runs_per_benchmark" \
    "$experiment_dir" \
    "$record_inference" \
    "$data_config" \
    "${benchmarks[@]}"

for benchmark_yaml_path in "${benchmarks[@]}"; do
    benchmark_name=$(basename $benchmark_yaml_path .yaml)
    /home/innovation-hacking/luebbet/venvs/isaaclab_luebbet/bin/python /home/innovation-hacking/luebbet/dev/IsaacLab/scripts/tng/utils/summarize_benchmarks.py \
        --benchmark_name $benchmark_name \
        --experiment_dir $experiment_dir \
        --visualize 
done

#/home/innovation-hacking/luebbet/venvs/isaaclab_luebbet/bin/python /home/innovation-hacking/luebbet/dev/IsaacLab/scripts/tng/utils/push_to_aws.py \
#    $experiment_dir \
#    s3://tng-luebbe-master-arbeit/Experiments/new_pipeline/$experiment_name