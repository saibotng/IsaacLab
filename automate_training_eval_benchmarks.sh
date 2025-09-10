set -euo pipefail
experiment_name=simple_test
dataset_dir=/home/innovation-hacking/luebbet/dev/important_datasets/pipeline/simple_trajectory/
simple_trajectory_dataset_dir=/home/innovation-hacking/luebbet/dev/important_datasets/pipeline/simple_trajectory/
data_config="tng_ur5_AbsJointAndAbsTCPState_DeltaJointAction_2Cams"
max_steps=3
batch_size=16
save_steps=1000
parallel_envs=10
benchmarks=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_easy_default_cams_small.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_hard_default_cams_small.yaml
)


experiment_dir=/home/innovation-hacking/luebbet/models/pipeline/$experiment_name
dataset_name=$(basename $dataset_dir)
simple_trajectory_dataset_name=$(basename $simple_trajectory_dataset_dir)
SERVER_PID=""
start_server() {
  /home/innovation-hacking/luebbet/venvs/combined_robots_luebbet/bin/python \
    /home/innovation-hacking/luebbet/dev/Isaac-GR00T/scripts/inference_service.py \
      --model-path "$last_checkpoint_dir" \
      --embodiment_tag new_embodiment \
      --data_config "$data_config" \
      --server >/dev/null 2>&1 &
  SERVER_PID=$!
  echo "Started inference server with PID: $SERVER_PID"
}

stop_server() {
  if [[ -n "${SERVER_PID-}" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
    echo "Stopped inference server with PID: $SERVER_PID"
    kill "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
  unset SERVER_PID || true
}

trap stop_server EXIT INT TERM

mkdir -p $experiment_dir
mkdir -p "$experiment_dir/data_visualization"
cp "$dataset_dir"/*.png "$experiment_dir"/data_visualization/
output_dir=$experiment_dir/models_$data_config
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
    --save-steps $save_steps

output_dir=$experiment_dir/models_$data_config
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
        --save_plot_path $checkpoint_dir/plot \
        --trajs 1
    fi
done

last_checkpoint_dir=$(ls -td "$output_dir"/checkpoint*/ | head -n 1)
for benchmark_yaml_path in "${benchmarks[@]}"; do
    benchmark_name=$(basename $benchmark_yaml_path .yaml)
    unique_benchmark_name=${benchmark_name}_from_${experiment_name}
    bench_dir=$experiment_dir/eval_${benchmark_name}
    mkdir -p $bench_dir
    echo "Evaluating benchmark: $benchmark_name using checkpoint: $last_checkpoint_dir"
    echo "Saving results to: $bench_dir"
    start_server
    /home/innovation-hacking/luebbet/venvs/isaaclab_luebbet/bin/python /home/innovation-hacking/luebbet/dev/IsaacLab/scripts/tng/ur5_pick_and_place_eval.py \
        --headless \
        --enable_cameras \
        --blackwell \
        --from_yaml $benchmark_yaml_path \
        --recorder_dir $bench_dir \
        --num_envs $parallel_envs
    stop_server
    /home/innovation-hacking/luebbet/venvs/lerobot_editable/bin/python /home/innovation-hacking/luebbet/dev/lerobot/src/tng/convert_hdf5_to_lerobot.py \
        --dataset-root $bench_dir \
        --dataset-name "luebbet/$unique_benchmark_name" \
        --hdf5-files datagen_recordings.hdf5 datagen_recordings_failed.hdf5 \
        --push-to-hub \
        --commit-message "first commit" \
        --compute-stats 
    echo "Finished benchmark: $benchmark_name"
    echo "----------------------------------------"
done

/home/innovation-hacking/luebbet/venvs/isaaclab_luebbet/bin/python /home/innovation-hacking/luebbet/dev/IsaacLab/scripts/tng/utils/push_to_aws.py \
    $experiment_dir \
    s3://tng-luebbe-master-arbeit/Experiments/$experiment_name

trap - EXIT INT TERM



