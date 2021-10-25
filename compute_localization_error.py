import os
import os.path as osp
import argparse
import numpy as np
import numpy.linalg as npla
import matplotlib
import matplotlib.pyplot as plt
from datetime import datetime

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


from pyboreas import BoreasDataset
from pylgmath import se3op

matplotlib.use("Agg")  # matplotlib.use("TkAgg")


class BagFileParser():

  def __init__(self, bag_file):
    try:
      self.conn = sqlite3.connect(bag_file)
    except Exception as e:
      print(e)
      print('could not connect')
      raise Exception('could not connect')

    self.cursor = self.conn.cursor()

    table_names = self.cursor.execute("SELECT name FROM sqlite_master WHERE type='table';").fetchall()

    ## create a message (id, topic, type) map
    topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
    print(topics_data)
    topics_data = [(1, "localization_result", "vtr_msgs/msg/LocalizationResult")]  # need to add this type info manually
    print(topics_data)
    self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
    self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
    self.topic_msg_message = {name_of: get_message(type_of) for id_of, name_of, type_of in topics_data}

  def __del__(self):
    self.conn.close()

  # Return messages as list of tuples [(timestamp0, message0), (timestamp1, message1), ...]
  def get_bag_messages(self, topic_name):

    topic_id = self.topic_id[topic_name]

    # Get from the db
    rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()

    # Deserialise all and timestamp them
    return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]


def plot_error(fig, errors):
  plot_number = 611
  fig.set_size_inches(8, 12)
  fig.subplots_adjust(left=0.16, right=0.95, bottom=0.1, top=0.95, wspace=0.7, hspace=0.6)

  labels = ['x', 'y', 'z']
  for i in range(3):
    ax = fig.add_subplot(plot_number + i)
    ax.plot(errors[:, i], '-', linewidth=1.0)
    ax.set_xlabel(r"$t$ [$s$]")
    ax.set_ylabel(r"$\hat{r}_x - r_x$ [$m$]".replace("x", labels[i]))
  for i in range(3):
    ax = fig.add_subplot(plot_number + 3 + i)
    ax.plot(errors[:, i+3], '-', linewidth=1.0)
    ax.set_xlabel(r"$t$ [$s$]")
    ax.set_ylabel(r"$\hat{\theta}_x - \theta_x$ [$rad$]".replace("x", labels[i]))

## GLOBAL SETTINGS
ROOT = osp.join(os.getenv('VTRDATA'), 'boreas/sequences')
SEQUENCES = [
    ["boreas-2020-12-01-13-26"],
    ["boreas-2020-12-18-13-44"],
    ["boreas-2021-01-15-12-17"],
    ["boreas-2021-03-02-13-38"],
    ["boreas-2021-04-15-18-55"],
]

## OPTIONS
SEQUENCE = 0

def main(data_dir):
  data_dir = osp.normpath(data_dir)
  root_dir = osp.dirname(data_dir)
  odo_input = osp.basename(data_dir)
  loc_inputs = [i for i in os.listdir(data_dir) if i != odo_input]
  loc_inputs.sort()
  print("Root Directory:", root_dir)
  print("Odometry:", odo_input)
  print("Localization:", loc_inputs)

  # dataset directory and necessary sequences to load
  dataset_root = osp.join(os.getenv('VTRDATA'), 'boreas/sequences')
  dataset_seqs = [[x] for x in [odo_input, *loc_inputs]]
  print("Dataset Root:", dataset_root)
  print("Dataset Sequences:", dataset_seqs)
  dataset = BoreasDataset(dataset_root, dataset_seqs)

  # Ground truth is provided w.r.t sensor, so we set sensor to vehicle
  # transform to identity
  yfwd2xfwd = np.array([
      [0, 1, 0, 0],
      [-1, 0, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1],
  ])
  T_robot_lidar = yfwd2xfwd @ dataset.sequences[SEQUENCE].calib.T_applanix_lidar  ## Note: use a single calibration data
  # T_robot_lidar = dataset.sequences[SEQUENCE].calib.T_applanix_lidar
  T_lidar_robot = npla.inv(T_robot_lidar)

  # build dictionary
  ground_truth_poses = { int(int(frame.timestamp * 1e9) / 1e6): frame.pose @ T_lidar_robot for frame in dataset.lidar_frames }

  print("Loaded number of poses: ", len(ground_truth_poses))

  for fid, loc_input in enumerate(loc_inputs):
    result_dir = osp.join(root_dir, odo_input, loc_input, "boreas/graph/run_000001/data")

    # get bag file
    bag_file = '{0}/{1}/{1}_0.db3'.format(osp.abspath(result_dir), "localization_result")

    parser = BagFileParser(bag_file)
    messages = parser.get_bag_messages("localization_result")

    errors = np.empty((len(messages), 6))
    for i, message in enumerate(messages):
      if not int(message[1].timestamp / 1e6) in ground_truth_poses.keys():
        print("WARNING: time stamp not found 1: ", int(message[1].timestamp / 1e6))
        continue
      if not int(message[1].vertex_timestamp / 1e6) in ground_truth_poses.keys():
        print("WARNING: time stamp not found 2: ", int(message[1].vertex_timestamp / 1e6))
        continue

      robot_timestamp = int(message[1].timestamp / 1e6)
      vertex_timestamp = int(message[1].vertex_timestamp / 1e6)
      T_robot_vertex_vec = np.array(message[1].t_robot_vertex.xi)

      # inv(T_enu_robot) @ T_enu_vertex = T_robot_vertex
      T_robot_vertex = npla.inv(ground_truth_poses[robot_timestamp]) @ ground_truth_poses[vertex_timestamp]
      T_robot_vertex_vec_gt = se3op.tran2vec(T_robot_vertex).flatten()

      # compute error
      errors[i, :] = T_robot_vertex_vec_gt - T_robot_vertex_vec

    print(np.mean(np.abs(errors), axis=0))

    fig = plt.figure(fid)
    plot_error(fig, errors)
    os.makedirs(odo_input, exist_ok=True)
    fig.savefig(osp.join(odo_input, loc_input+'.png'))

  # plt.show()




if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  main(args.path)