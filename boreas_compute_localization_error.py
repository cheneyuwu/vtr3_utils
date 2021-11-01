import os
import os.path as osp
import argparse
from typing import List, Tuple
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
      print('Could not connect: ', e)
      raise Exception('could not connect')

    self.cursor = self.conn.cursor()

    table_names = self.cursor.execute("SELECT name FROM sqlite_master WHERE type='table';").fetchall()

    ## create a message (id, topic, type) map
    topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()

    self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
    self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
    self.topic_msg_message = {name_of: get_message(type_of) for id_of, name_of, type_of in topics_data}

  def __del__(self):
    self.conn.close()

  # Return messages as list of tuples [(timestamp0, message0), (timestamp1, message1), ...]
  def get_bag_messages(self, topic_name):
    topic_id = self.topic_id[topic_name]
    rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
    return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]


def plot_error(fig, errors):
  # switch to absolute error
  errors = np.abs(errors)

  plot_number = 611
  fig.set_size_inches(8, 12)
  fig.subplots_adjust(left=0.16, right=0.95, bottom=0.1, top=0.93, wspace=0.7, hspace=0.7)

  labels = ['x', 'y', 'z', 'x', 'y', 'z']
  for i in range(3):
    ax = fig.add_subplot(plot_number + i)
    # plot the errors
    ax.plot(errors[:, i], '-b', linewidth=1.0)
    # plot the mean
    ax.plot(np.ones_like(errors[:, i]) * np.mean(errors[:, i]), '-r', linewidth=2.0)
    ax.set_title("mean: " + str(np.mean(errors[:, i])) + ", var: " + str(np.var(errors[:, i])))
    ax.set_xlabel(r"frame")
    ax.set_ylabel(r"$|\hat{r}_x - r_x|$ [$m$]".replace("x", labels[i]))
    ax.set_ylim([0, 1])
  for i in range(3, 6):
    ax = fig.add_subplot(plot_number + i)
    # plot the errors
    ax.plot(errors[:, i], '-b', linewidth=1.0)
    # plot the mean
    ax.plot(np.ones_like(errors[:, i]) * np.mean(errors[:, i]), '-r', linewidth=2.0)
    ax.set_title("mean: " + str(np.mean(errors[:, i])) + ", var: " + str(np.var(errors[:, i])))
    ax.set_xlabel(r"frame")
    ax.set_ylabel(r"$|\hat{\theta}_x - \theta_x|$ [$rad$]".replace("x", labels[i]))
    ax.set_ylim([0, 0.1])

def plot_error_box(fig, errors: List[Tuple[str, np.ndarray]], proc_func = lambda x: x, ylabel=r"$\hat{tran}_dir - tran_dir$ [$unit$]"):
  plot_number = 611
  fig.set_size_inches(2 + 1.4 * len(errors), 12)
  fig.subplots_adjust(left=0.16, right=0.95, bottom=0.1, top=0.93, wspace=0.7, hspace=0.7)

  labels = ['x', 'y', 'z', 'x', 'y', 'z']
  for i in range(3):
    ax = fig.add_subplot(plot_number + i)
    # plot the errors
    ax.boxplot([proc_func(error[:, i]) for _, error in errors])
    ax.set_xticklabels([x for x, _ in errors], rotation=-25)
    ax.set_ylabel(ylabel.replace(r"tran", r"r").replace(r"unit", r"m").replace(r"dir", labels[i]))
    # ax.set_ylim([-1, 1])
  for i in range(3, 6):
    ax = fig.add_subplot(plot_number + i)
    # plot the errors
    ax.boxplot([proc_func(error[:, i]) for _, error in errors])
    ax.set_xticklabels([x for x, _ in errors], rotation=-25)
    ax.set_ylabel(ylabel.replace(r"tran", r"\theta").replace(r"unit", r"rad").replace(r"dir", labels[i]))
    # ax.set_ylim([-0.1, 0.1])

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

  # generate ground truth pose dictionary
  ground_truth_poses = dict()
  for sequence in dataset.sequences:
    # Ground truth is provided w.r.t sensor, so we set sensor to vehicle
    # transform to identity
    yfwd2xfwd = np.array([
        [0, 1, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    T_robot_lidar = yfwd2xfwd @ sequence.calib.T_applanix_lidar
    # T_robot_lidar = sequence.calib.T_applanix_lidar
    T_lidar_robot = npla.inv(T_robot_lidar)

    # build dictionary
    precision = 1e7  # divide by this number to ensure always find the timestamp
    ground_truth_poses.update({ int(int(frame.timestamp * 1e9) / precision): frame.pose @ T_lidar_robot for frame in sequence.lidar_frames })

  print("Loaded number of poses: ", len(ground_truth_poses))

  date2trial_map = dict()  # dict with key=loc_input, value=[(trial, error), ...]
  trial2date_map = dict()  # dict with key=trial, value=[(loc_input, error), ...]
  for i, loc_input in enumerate(loc_inputs):
    loc_dir = osp.join(root_dir, odo_input, loc_input)
    trials = list(os.listdir(loc_dir))
    trials.sort()

    date2trial_map[loc_input] = list()

    for j, trial in enumerate(trials):
      result_dir = osp.join(loc_dir, trial, "graph/run_000001/data")
      if not osp.exists(result_dir):
        continue
      print("Looking at result directory:", result_dir)

      # get bag file
      bag_file = '{0}/{1}/{1}_0.db3'.format(osp.abspath(result_dir), "localization_result")

      parser = BagFileParser(bag_file)
      messages = parser.get_bag_messages("localization_result")

      errors = np.empty((len(messages), 6))
      for i, message in enumerate(messages):
        if not int(message[1].timestamp / precision) in ground_truth_poses.keys():
          print("WARNING: time stamp not found 1: ", int(message[1].timestamp / precision))
          continue
        if not int(message[1].vertex_timestamp / precision) in ground_truth_poses.keys():
          print("WARNING: time stamp not found 2: ", int(message[1].vertex_timestamp / precision))
          continue

        robot_timestamp = int(message[1].timestamp / precision)
        vertex_timestamp = int(message[1].vertex_timestamp / precision)

        T_robot_vertex_vec = np.array(message[1].t_robot_vertex.xi)[..., None]
        T_robot_vertex = se3op.vec2tran(T_robot_vertex_vec)
        # inv(T_enu_robot) @ T_enu_vertex = T_robot_vertex
        T_vertex_robot_gt = npla.inv(ground_truth_poses[vertex_timestamp]) @ ground_truth_poses[robot_timestamp]
        # compute error
        errors[i, :] = se3op.tran2vec(T_robot_vertex @ T_vertex_robot_gt).flatten()



        # TODO remove the following
        # T_robot_vertex_vec = np.array(message[1].t_robot_vertex.xi)
        # # inv(T_enu_robot) @ T_enu_vertex = T_robot_vertex
        # T_robot_vertex = npla.inv(ground_truth_poses[robot_timestamp]) @ ground_truth_poses[vertex_timestamp]
        # T_robot_vertex_vec_gt = se3op.tran2vec(T_robot_vertex).flatten()
        # # compute error
        # errors[i, :] = T_robot_vertex_vec_gt - T_robot_vertex_vec

      print(np.mean(np.abs(errors), axis=0))

      #
      date2trial_map[loc_input].append((trial.replace("boreas.", ""), errors))
      if trial not in trial2date_map.keys():
        trial2date_map[trial] = list()
      trial2date_map[trial].append((loc_input.replace("boreas-", ""), errors))


      fig = plt.figure()
      plot_error(fig, errors)
      fig.suptitle(odo_input + " <- " + loc_input + " : " + trial, fontsize=16)
      os.makedirs(osp.join(odo_input, loc_input), exist_ok=True)
      fig.savefig(osp.join(odo_input, loc_input, trial+'.png'))

  # plot box plot based on the two maps
  os.makedirs(odo_input, exist_ok=True)

  for k, v in date2trial_map.items():
    # error
    fig = plt.figure()
    plot_error_box(fig, v)
    fig.suptitle(odo_input + " <- " + k, fontsize=16)
    fig.savefig(osp.join(odo_input, k+'_box.png'))
    # np.abs(error)
    fig = plt.figure()
    plot_error_box(fig, v, np.abs, r"$|\hat{tran}_dir - tran_dir|$ [$unit$]")
    fig.suptitle(odo_input + " <- " + k, fontsize=16)
    fig.savefig(osp.join(odo_input, k+'_abs_box.png'))

  for k, v in trial2date_map.items():
    # error
    fig = plt.figure()
    plot_error_box(fig, v)
    fig.suptitle(odo_input + " point map version:" + k.replace("boreas.", ""), fontsize=16)
    fig.savefig(osp.join(odo_input, k+'_box.png'))
    # np.abs(err)
    fig = plt.figure()
    plot_error_box(fig, v, np.abs, r"$|\hat{tran}_dir - tran_dir|$ [$unit$]")
    fig.suptitle(odo_input + " point map version:" + k.replace("boreas.", ""), fontsize=16)
    fig.savefig(osp.join(odo_input, k+'_abs_box.png'))


  # plt.show()




if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  main(args.path)