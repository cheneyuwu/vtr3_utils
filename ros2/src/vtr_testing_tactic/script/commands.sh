## Define the following environment variables VTH=VTR Testing Honeycomb
export VTHROOT=${VTRSRC}_utils/ros2/src/vtr_testing_tactic
export VTHCONFIG=${VTHROOT}/config
export VTHDATA=${VTRDATA}
export VTHRESULT=${VTRTEMP}/testing/multienv
export DATASET=${VTHDATA}/utias_20211230_multiple_terrain/<parkinglot, ...>
mkdir -p ${VTHRESULT}

source ${VTRSRC}_utils/ros2/install/setup.bash

## --prefix 'gdb -ex run --args'

# parkinglot static
ODO_INPUT=rosbag2_2021_11_01-18_05_58
LOC_INPUT=rosbag2_2021_11_01-18_10_03
# parkinglot with you
ODO_INPUT=rosbag2_2022_01_09-19_00_08
LOC_INPUT=rosbag2_2022_01_09-18_55_25
# backyard
ODO_INPUT=rosbag2_2021_12_30-14_31_27
LOC_INPUT=rosbag2_2021_12_30-14_37_51
# dome inside
ODO_INPUT=rosbag2_2022_01_30-22_34_29
LOC_INPUT=rosbag2_2022_01_30-22_36_16



## Perform odometry on a sequence directly
ros2 run vtr_testing_tactic vtr_testing_tactic_odometry_direct  \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT} \
  -p odo_dir:=${DATASET}/${ODO_INPUT}

## Perform localization on a sequence directly (with a specified point map version)
cp -r ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}  ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}.bak
rm -r ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}
mkdir -p ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}
cp -r ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/*  ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}

ros2 run vtr_testing_tactic vtr_testing_tactic_localization_direct \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p odo_dir:=${DATASET}/${ODO_INPUT} \
  -p loc_dir:=${DATASET}/${LOC_INPUT}

## Perform localization + planning on a sequence directly (with a specified point map version)
rm -r ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}
mkdir -p ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}
cp -r ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/*  ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}
ros2 run vtr_testing_tactic vtr_testing_tactic_localization_planning_direct \
  --ros-args -p use_sim_time:=true -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p odo_dir:=${DATASET}/${ODO_INPUT} \
  -p loc_dir:=${DATASET}/${LOC_INPUT}



## Perform offline tasks
ros2 run vtr_testing_tactic vtr_testing_tactic_intra_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}

ros2 run vtr_testing_tactic vtr_testing_tactic_dynamic_detection \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}

ros2 run vtr_testing_tactic vtr_testing_tactic_inter_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}

ros2 run vtr_testing_tactic vtr_testing_tactic_terrain_assessment \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}

ros2 run vtr_testing_tactic vtr_testing_tactic_intra_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p run_id:=1

ros2 run vtr_testing_tactic vtr_testing_tactic_change_detection \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p run_id:=1