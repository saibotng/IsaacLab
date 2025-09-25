#!/bin/bash

# Usage: ./automate_benchmark_runs.sh <runs_per_benchmark> <experiment_dir> <record_inference> <data_config> [benchmark1] [benchmark2] ...
# Example: ./automate_benchmark_runs.sh 3 "/path/to/experiment" true "tng_ur5_AbsJointState_DeltaJointAction_2Cams" benchmark_easy.yaml benchmark_hard.yaml

if [ $# -lt 4 ]; then
    echo "Usage: $0 <runs_per_benchmark> <experiment_dir> <record_inference> <data_config> [benchmark1] [benchmark2] ..."
    echo "Example: $0 3 \"/path/to/experiment\" true \"tng_ur5_AbsJointState_DeltaJointAction_2Cams\" benchmark_easy.yaml benchmark_hard.yaml"
    exit 1
fi

runs_per_benchmark="$1"
experiment_dir="$2"
record_inference="$3"
data_config="$4"

# Parse benchmark files from remaining arguments
benchmarks=()
shift 4  # Remove the first 4 arguments
for benchmark in "$@"; do
    benchmarks+=("$benchmark")
done

# Validate that we have at least one benchmark
if [ ${#benchmarks[@]} -eq 0 ]; then
    echo "Error: At least one benchmark file must be provided"
    exit 1
fi

# Find the models directory and last checkpoint
models_dir="$experiment_dir/models_$data_config"
if [ ! -d "$models_dir" ]; then
    echo "Error: Models directory does not exist: $models_dir"
    exit 1
fi

last_checkpoint_dir=$(ls -d "$models_dir"/checkpoint-* | sort -V | tail -n 1)
if [ ! -d "$last_checkpoint_dir" ]; then
    echo "Error: No checkpoint directories found in: $models_dir"
    exit 1
fi

set -euo pipefail

# Server management functions
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

checkpoint_name=$(basename "$last_checkpoint_dir")
parallel_envs=10

echo "Running benchmarks with checkpoint: $last_checkpoint_dir"

# Main benchmark execution loop
for ((i=1; i<=runs_per_benchmark; i++)); do
    echo "Starting benchmark run $i of $runs_per_benchmark"
    for benchmark_yaml_path in "${benchmarks[@]}"; do
        benchmark_name=$(basename "$benchmark_yaml_path" .yaml)
        bench_dir="$experiment_dir/eval_${i}_${benchmark_name}_from_${checkpoint_name}"
        mkdir -p "$bench_dir"
        
        # Set recorder_dir based on record_inference flag for this specific run
        if [[ "$record_inference" == "true" ]]; then
            recorder_dir="$bench_dir"
        else
            recorder_dir="NoRecording"
        fi
        
        echo "Evaluating benchmark: $benchmark_name using checkpoint: $last_checkpoint_dir"
        echo "Saving results to: $bench_dir"
        echo "Recording inference: $record_inference (recorder_dir: $recorder_dir)"
        
        start_server
        /home/innovation-hacking/luebbet/venvs/isaaclab_luebbet/bin/python /home/innovation-hacking/luebbet/dev/IsaacLab/scripts/tng/ur5_pick_and_place_eval.py \
            --headless \
            --enable_cameras \
            --blackwell \
            --from_yaml "$benchmark_yaml_path" \
            --recorder_dir "$recorder_dir" \
            --log_dir "$bench_dir" \
            --num_envs $parallel_envs
        stop_server
        
        echo "Finished benchmark: $benchmark_name using checkpoint: $last_checkpoint_dir run $i of $runs_per_benchmark"
        echo "Results saved in: $bench_dir"
        echo "----------------------------------------"
    done
done

echo "All benchmark runs completed successfully!"
