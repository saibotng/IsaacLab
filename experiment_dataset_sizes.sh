set -euo pipefail
experiment_name=dataset_size_30_10k
dataset_dir=/home/innovation-hacking/luebbet/dev/important_datasets/pipeline/base_dataset_30/
simple_trajectory_dataset_dir=/home/innovation-hacking/luebbet/dev/important_datasets/pipeline/simple_trajectory/
data_config="tng_ur5_AbsJointState_DeltaJointAction_2Cams"
max_steps=10000
batch_size=16
save_steps=20000
parallel_envs=10
benchmarks=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_easy_default_cams.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_hard_default_cams.yaml
)


experiment_dir=/home/innovation-hacking/luebbet/models/pipeline/$experiment_name
dataset_name=$(basename $dataset_dir)
simple_trajectory_dataset_name=$(basename $simple_trajectory_dataset_dir)
output_dir=$experiment_dir/models_$data_config
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


echo "Looking for checkpoints in: $output_dir"


last_checkpoint_dir=$(ls -d "$output_dir"/checkpoint-* | sort -V | tail -n 1)
last_checkpoint_name=$(basename "$last_checkpoint_dir")
for benchmark_yaml_path in "${benchmarks[@]}"; do
    benchmark_name=$(basename $benchmark_yaml_path .yaml)
    unique_benchmark_name=${benchmark_name}_from_${experiment_name}
    bench_dir=$experiment_dir/eval_3_${benchmark_name}_from_${last_checkpoint_name}
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
    echo "Finished benchmark: $benchmark_name"
    echo "----------------------------------------"
done



experiment_name=dataset_size_100_10k
dataset_dir=/home/innovation-hacking/luebbet/dev/important_datasets/pipeline/base_dataset_100/
experiment_dir=/home/innovation-hacking/luebbet/models/pipeline/$experiment_name
dataset_name=$(basename $dataset_dir)
simple_trajectory_dataset_name=$(basename $simple_trajectory_dataset_dir)
output_dir=$experiment_dir/models_$data_config


last_checkpoint_dir=$(ls -d "$output_dir"/checkpoint-* | sort -V | tail -n 1)
last_checkpoint_name=$(basename "$last_checkpoint_dir")
for benchmark_yaml_path in "${benchmarks[@]}"; do
    benchmark_name=$(basename $benchmark_yaml_path .yaml)
    unique_benchmark_name=${benchmark_name}_from_${experiment_name}
    bench_dir=$experiment_dir/eval_3_${benchmark_name}_from_${last_checkpoint_name}
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
    echo "Finished benchmark: $benchmark_name"
    echo "----------------------------------------"
done



experiment_name=dataset_size_300_10k
dataset_dir=/home/innovation-hacking/luebbet/dev/important_datasets/pipeline/base_dataset_300/
experiment_dir=/home/innovation-hacking/luebbet/models/pipeline/$experiment_name
dataset_name=$(basename $dataset_dir)
simple_trajectory_dataset_name=$(basename $simple_trajectory_dataset_dir)
output_dir=$experiment_dir/models_$data_config

last_checkpoint_dir=$(ls -d "$output_dir"/checkpoint-* | sort -V | tail -n 1)
last_checkpoint_name=$(basename "$last_checkpoint_dir")
for benchmark_yaml_path in "${benchmarks[@]}"; do
    benchmark_name=$(basename $benchmark_yaml_path .yaml)
    unique_benchmark_name=${benchmark_name}_from_${experiment_name}
    bench_dir=$experiment_dir/eval_3_${benchmark_name}_from_${last_checkpoint_name}
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
    echo "Finished benchmark: $benchmark_name"
    echo "----------------------------------------"
done

experiment_name=dataset_size_30_20k
dataset_dir=/home/innovation-hacking/luebbet/dev/important_datasets/pipeline/base_dataset_30/
simple_trajectory_dataset_dir=/home/innovation-hacking/luebbet/dev/important_datasets/pipeline/simple_trajectory/
data_config="tng_ur5_AbsJointState_DeltaJointAction_2Cams"
max_steps=20000
batch_size=16
save_steps=20000
parallel_envs=10
benchmarks=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_easy_default_cams.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_hard_default_cams.yaml
)


experiment_dir=/home/innovation-hacking/luebbet/models/pipeline/$experiment_name
dataset_name=$(basename $dataset_dir)
simple_trajectory_dataset_name=$(basename $simple_trajectory_dataset_dir)
output_dir=$experiment_dir/models_$data_config
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


last_checkpoint_dir=$(ls -d "$output_dir"/checkpoint-* | sort -V | tail -n 1)
last_checkpoint_name=$(basename "$last_checkpoint_dir")
for benchmark_yaml_path in "${benchmarks[@]}"; do
    benchmark_name=$(basename $benchmark_yaml_path .yaml)
    unique_benchmark_name=${benchmark_name}_from_${experiment_name}
    bench_dir=$experiment_dir/eval_3_${benchmark_name}_from_${last_checkpoint_name}
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
    echo "Finished benchmark: $benchmark_name"
    echo "----------------------------------------"
done



experiment_name=dataset_size_100_20k
dataset_dir=/home/innovation-hacking/luebbet/dev/important_datasets/pipeline/base_dataset_100/
experiment_dir=/home/innovation-hacking/luebbet/models/pipeline/$experiment_name
dataset_name=$(basename $dataset_dir)
simple_trajectory_dataset_name=$(basename $simple_trajectory_dataset_dir)
output_dir=$experiment_dir/models_$data_config


last_checkpoint_dir=$(ls -d "$output_dir"/checkpoint-* | sort -V | tail -n 1)
last_checkpoint_name=$(basename "$last_checkpoint_dir")
for benchmark_yaml_path in "${benchmarks[@]}"; do
    benchmark_name=$(basename $benchmark_yaml_path .yaml)
    unique_benchmark_name=${benchmark_name}_from_${experiment_name}
    bench_dir=$experiment_dir/eval_3_${benchmark_name}_from_${last_checkpoint_name}
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
    echo "Finished benchmark: $benchmark_name"
    echo "----------------------------------------"
done



experiment_name=dataset_size_300_20k
dataset_dir=/home/innovation-hacking/luebbet/dev/important_datasets/pipeline/base_dataset_300/
experiment_dir=/home/innovation-hacking/luebbet/models/pipeline/$experiment_name
dataset_name=$(basename $dataset_dir)
simple_trajectory_dataset_name=$(basename $simple_trajectory_dataset_dir)
output_dir=$experiment_dir/models_$data_config



last_checkpoint_dir=$(ls -d "$output_dir"/checkpoint-* | sort -V | tail -n 1)
last_checkpoint_name=$(basename "$last_checkpoint_dir")
for benchmark_yaml_path in "${benchmarks[@]}"; do
    benchmark_name=$(basename $benchmark_yaml_path .yaml)
    unique_benchmark_name=${benchmark_name}_from_${experiment_name}
    bench_dir=$experiment_dir/eval_3_${benchmark_name}_from_${last_checkpoint_name}
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
    echo "Finished benchmark: $benchmark_name"
    echo "----------------------------------------"
done









trap - EXIT INT TERM



