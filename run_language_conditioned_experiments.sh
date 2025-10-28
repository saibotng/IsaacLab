data_config="tng_ur5_AbsJointState_DeltaJointAction_2Cams"
runs_per_benchmark=3
max_train_steps=20000
benchmarks=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_language_no_distractors_all_colors.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_language_no_distractors_limited_colors.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_language_with_distractors_all_colors.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_language_with_distractors_limited_colors.yaml
)


/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "language_no_distractors_limited_colors" \
    "language_no_distractors_limited_colors" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "language_no_distractors_all_colors" \
    "language_no_distractors_all_colors" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "language_with_distractors_limited_colors" \
    "language_with_distractors_limited_colors" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "language_with_distractors_all_colors" \
    "language_with_distractors_all_colors" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"