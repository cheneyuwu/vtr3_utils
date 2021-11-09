# generate ground truth of robot poses of one parking lot sequence
# this uses motion prior + gps measurements + side-slip costs to get orientation information

DATASET=${VTRDATA}/utias_20211101_parkinglot_shorter_sequence

# parking lot full sequences
SEQUENCES=(
  rosbag2_2021_11_01-18_05_58  # ground truth ok
  rosbag2_2021_11_01-18_10_03  # ground truth ok
  rosbag2_2021_11_01-18_14_04  # ground truth ok
  rosbag2_2021_11_01-18_18_34  # ground truth ok
  rosbag2_2021_11_06-11_41_33  # almost empty parking lot run first -> ground truth ok
  rosbag2_2021_11_06-11_45_39  # almost empty run second -> ground truth ok
  rosbag2_2021_11_06-11_58_53  # east side first -> ground truth ok (sometimes got eigen decompose error)
  rosbag2_2021_11_06-12_07_03  # east side second -> ok
  # rosbag2_2021_11_06-12_14_10  # east side third -> constantly got eigen decompose error, redo this one
  # rosbag2_2021_11_06-12_28_42  # east side 4 - stop in between due to zeus, gps so bad, redo this
  rosbag2_2021_11_06-19_57_56
  # rosbag2_2021_11_06-20_01_44
  rosbag2_2021_11_06-20_05_35
  # rosbag2_2021_11_09-09_33_49  # bad gps
  rosbag2_2021_11_09-09_39_16
  rosbag2_2021_11_09-09_43_03
)
# # short staight line from parking lot to pickup and receiving
# SEQUENCES=(
#   rosbag2_2021_11_06-20_13_42
#   # rosbag2_2021_11_06-20_25_22
#   rosbag2_2021_11_06-20_29_52
#   rosbag2_2021_11_06-20_34_11
#   rosbag2_2021_11_06-20_38_08
#   rosbag2_2021_11_06-20_41_24
#   rosbag2_2021_11_06-20_48_00
# )

for SEQUENCE in "${SEQUENCES[@]}"; do
  ros2 run parkinglot_dataset parkinglot_dataset_ground_truth --ros-args -p dataset:=${DATASET}/${SEQUENCE}
done

# generate error plots for parking lot datasets
# path input is the odometry path (top level)

python parkinglot_compute_localization_error.py \
  --dataset ${DATASET} \
  --path ${VTRTEMP}/testing/lidar/honeycomb.exp/${ODO_INPUT}