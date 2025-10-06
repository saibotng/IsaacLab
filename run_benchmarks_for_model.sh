model_dir=/home/innovation-hacking/luebbet/experiments/pipeline/visuals_front_dynamic/
data_config="tng_ur5_AbsJointState_DeltaJointAction_2Cams"
benchmarks=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_visuals_static_front_side.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_visuals_dynamic_front_side.yaml
)   

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_benchmark_runs.sh \
    3 \
    $model_dir \
    false \
    "$data_config" \
    "${benchmarks[@]}"

