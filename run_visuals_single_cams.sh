data_config="tng_ur5_AbsJointState_DeltaJointAction_2Cams"
runs_per_benchmark=3
max_train_steps=20000
benchmarks_side=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_visuals_static_side_shoulder.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_visuals_dynamic_side_shoulder.yaml
)
benchmarks_shoulder=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_visuals_static_shoulder_front.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_visuals_dynamic_shoulder_front.yaml
)
benchmarks_front=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_visuals_static_front_side.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_visuals_dynamic_front_side.yaml
)

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "visuals_dynamic_front" \
    "visuals_dynamic_front_side" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks_front[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "visuals_dynamic_side" \
    "visuals_dynamic_side_shoulder" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks_side[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "visuals_dynamic_shoulder" \
    "visuals_dynamic_shoulder_front" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks_shoulder[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "visuals_static_front" \
    "visuals_static_front_side" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks_front[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "visuals_static_side" \
    "visuals_static_side_shoulder" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks_side[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "visuals_static_shoulder" \
    "visuals_static_shoulder_front" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks_shoulder[@]}"


