data_config="tng_ur5_AbsJointState_DeltaJointAction_3Cams"
runs_per_benchmark=3
max_train_steps=20000

benchmark_front_side=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_visuals_static_front_side.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_visuals_dynamic_front_side.yaml
)
benchmarks_side_shoulder=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_visuals_static_side_shoulder.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_visuals_dynamic_side_shoulder.yaml
)
benchmarks_shoulder_front=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_visuals_static_shoulder_front.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_visuals_dynamic_shoulder_front.yaml
)


/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "visuals_front_side_dynamic" \
    "visuals_front_side_dynamic" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmark_front_side[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "visuals_side_shoulder_dynamic" \
    "visuals_side_shoulder_dynamic" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks_side_shoulder[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "visuals_shoulder_front_dynamic" \
    "visuals_shoulder_front_dynamic" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks_shoulder_front[@]}"
