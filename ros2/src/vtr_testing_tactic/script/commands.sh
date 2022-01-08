## Define the following environment variables VTH=VTR Testing Honeycomb
export VTHROOT=${VTRSRC}_utils/ros2/src/vtr_testing_tactic
export VTHCONFIG=${VTHROOT}/config
export VTHDATA=${VTRDATA}
export VTHRESULT=${VTRTEMP}/testing/multienv
export DATASET=${VTHDATA}/utias_20211230_multiple_terrain
mkdir -p ${VTHRESULT}

source ${VTRSRC}_utils/ros2/install/setup.bash

## Perform odometry on a sequence directly
ODO_INPUT=rosbag2_2021_11_01-18_05_58
#        package            executable                                      namespace      parameter file                          data directory (output dir)       input directory
ros2 run vtr_testing_tactic vtr_testing_tactic_odometry_direct  \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT} \
  -p odo_dir:=${DATASET}/${ODO_INPUT}

## Perform localization on a sequence directly (with a specified point map version)
ODO_INPUT=rosbag2_2021_11_01-18_05_58
LOC_INPUT=rosbag2_2021_11_01-18_10_03

cp -r ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}  ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}.bak
rm -r ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}
mkdir -p ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}
cp -r ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/*  ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}

ros2 run vtr_testing_tactic vtr_testing_tactic_localization_direct \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p odo_dir:=${DATASET}/${ODO_INPUT} \
  -p loc_dir:=${DATASET}/${LOC_INPUT}

## use this after copied
rm -r ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}
mkdir -p ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}
cp -r ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/*  ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}
ros2 run vtr_testing_tactic vtr_testing_tactic_localization_planning_direct \
  --ros-args -p use_sim_time:=true -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p odo_dir:=${DATASET}/${ODO_INPUT} \
  -p loc_dir:=${DATASET}/${LOC_INPUT}

## --prefix 'gdb -ex run --args'

## Perform offline tasks
ODO_INPUT=rosbag2_2021_11_06-11_45_39
LOC_INPUT=rosbag2_2021_11_01-18_05_58
ros2 run vtr_testing_tactic vtr_testing_tactic_intra_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}
ros2 run vtr_testing_tactic vtr_testing_tactic_dynamic_detection \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}
ros2 run vtr_testing_tactic vtr_testing_tactic_inter_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}



ODO_INPUT=rosbag2_2021_11_06-11_45_39
LOC_INPUT=rosbag2_2021_11_01-18_05_58
ros2 run vtr_testing_tactic vtr_testing_tactic_intra_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p run_id:=1

ros2 run vtr_testing_tactic vtr_testing_tactic_change_detection \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb_v2.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p run_id:=1