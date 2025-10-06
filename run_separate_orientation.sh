data_config="tng_ur5_AbsJointState_DeltaJointAction_2Cams"
runs_per_benchmark=3
max_train_steps=20000
benchmarks=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_easy_default_cams.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_hard_default_cams.yaml
)

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "kinematics_combined_experiments" \
    "kinematics_combined_experiments" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"
