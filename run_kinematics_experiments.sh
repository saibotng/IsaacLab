data_config="tng_ur5_AbsJointState_DeltaJointAction_2Cams"
runs_per_benchmark=3
max_train_steps=20000
benchmarks=(
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_kinematics_easy.yaml
    /home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks/benchmark_kinematics_hard.yaml
)

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "kinematics_calibration" \
    "kinematics_calibration" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "kinematics_increased_dim" \
    "kinematics_increased_dim" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "kinematics_starting_pose" \
    "kinematics_starting_pose" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "kinematics_table_height" \
    "kinematics_table_height" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "kinematics_yaw" \
    "kinematics_yaw" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "kinematics_base" \
    "kinematics_base" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "kinematics_yaw_improved_control" \
    "kinematics_yaw_improved_control" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"

/home/innovation-hacking/luebbet/dev/IsaacLab/automate_single_experiment.sh \
    "kinematics_combined_findings" \
    "kinematics_combined_findings" \
    "$data_config" \
    "$max_train_steps" \
    "$runs_per_benchmark" \
    "${benchmarks[@]}"
