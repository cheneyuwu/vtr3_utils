## Define the following environment variables VTH=VTR Testing Honeycomb
export VTHROOT=${VTRSRC}/extra/src/vtr_testing_tactic
export VTHCONFIG=${VTHROOT}/config
export VTHDATA=${VTRDATA}
export VTHRESULT=${VTRTEMP}/testing/lidar/tactic.exp2
export DATASET=${VTHDATA}/utias_20211101_parkinglot_shorter_sequence
mkdir -p ${VTHRESULT}

## Perform odometry on a sequence directly
ODO_INPUT=rosbag2_2021_11_06-11_45_39
#        package            executable                                      namespace      parameter file                          data directory (output dir)       input directory
ros2 run vtr_testing_tactic vtr_testing_tactic_odometry_direct  \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb \
  -p odo_dir:=${DATASET}/${ODO_INPUT}

## Perform localization on a sequence directly (with a specified point map version)
ODO_INPUT=rosbag2_2021_11_06-11_45_39
LOC_INPUT=rosbag2_2021_11_01-18_05_58

cp -r ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb \
  ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb_v1

rm -r ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}
mkdir -p ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}
cp -r ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb \
  ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}

ros2 run vtr_testing_tactic vtr_testing_tactic_localization_direct \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}/honeycomb \
  -p odo_dir:=${DATASET}/${ODO_INPUT} \
  -p loc_dir:=${DATASET}/${LOC_INPUT}

## Perform offline tasks
ODO_INPUT=rosbag2_2021_11_06-11_45_39
LOC_INPUT=rosbag2_2021_11_01-18_05_58
ros2 run vtr_testing_tactic vtr_testing_tactic_intra_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb
ros2 run vtr_testing_tactic vtr_testing_tactic_dynamic_detection \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb
ros2 run vtr_testing_tactic vtr_testing_tactic_inter_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb



ODO_INPUT=rosbag2_2021_11_06-11_45_39
LOC_INPUT=rosbag2_2021_11_01-18_05_58
ros2 run vtr_testing_tactic vtr_testing_tactic_intra_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}/honeycomb \
  -p run_id:=1

ros2 run vtr_testing_tactic vtr_testing_tactic_change_detection \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}/honeycomb \
  -p run_id:=1