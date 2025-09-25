dataset_name="base_dataset_100"
data_config="tng_ur5_AbsJointState_DeltaJointAction_2Cams"
runs_per_benchmark=3
benchmarks=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_easy_default_cams.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_hard_default_cams.yaml
)

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "train_steps_2k" \
    "$dataset_name" \
    "$data_config" \
    2000 \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "train_steps_5k" \
    "$dataset_name" \
    "$data_config" \
    5000 \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "train_steps_10k" \
    "$dataset_name" \
    "$data_config" \
    10000 \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "train_steps_20k" \
    "$dataset_name" \
    "$data_config" \
    20000 \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "train_steps_40k" \
    "$dataset_name" \
    "$data_config" \
    40000 \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "train_steps_80k" \
    "$dataset_name" \
    "$data_config" \
    80000 \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "train_steps_160k" \
    "$dataset_name" \
    "$data_config" \
    160000 \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"