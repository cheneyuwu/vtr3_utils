# generate ground truth of robot poses of one parking lot sequence
# this uses motion prior + gps measurements + side-slip costs to get orientation information
ros2 run parkinglot_dataset parkinglot_dataset_ground_truth --ros-args -p dataset:=${VTRDATA}/utias_20211101_parkinglot_shorter_sequence/rosbag2_2021_11_01-18_18_34/

# generate error plots for parking lot datasets
# path input is the odometry path (top level)
python parkinglot_compute_localization_error.py --path /ext0/ASRL/temp/testing/lidar/honeycomb.exp/rosbag2_2021_11_01-18_05_58/