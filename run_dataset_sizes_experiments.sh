data_config="tng_ur5_AbsJointState_DeltaJointAction_2Cams"
runs_per_benchmark=3
benchmarks=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_easy_default_cams.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_hard_default_cams.yaml
)

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "dataset_size_30" \
    "base_dataset_30" \
    "$data_config" \
    2000 \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"