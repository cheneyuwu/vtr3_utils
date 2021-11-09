import os
import os.path as osp
import argparse
from typing import List, Tuple
import numpy as np
np.set_printoptions(linewidth=120,suppress=True)
import numpy.linalg as npla
import matplotlib
import matplotlib.pyplot as plt
from datetime import datetime

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from pyboreas import BoreasDataset
from pylgmath import se3op

matplotlib.use("TkAgg")  # matplotlib.use("TkAgg")


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


def get_robot_poses(root: str, sequences: List[str]):

  # hardcoded transforms (no need be cause we are already in robot frame)
  # T_robot_gps = np.array([[1, 0, 0, 0.6], [0, 1, 0, 0], [0, 0, 1, 0.52], [0, 0, 0, 1]])
  # T_robot_lidar = np.array([[1, 0, 0, 0.06], [0, 1, 0, 0], [0, 0, 1, 1.45], [0, 0, 0, 1]])

  time_T_global_robot = dict()
  for seq in sequences:
    filename = osp.join(root, seq, "robot_poses.txt")
    if not osp.exists(filename):
      continue

    time_xi_k0 = np.loadtxt(filename, delimiter=' ')
    timestamps = time_xi_k0[:, 0]
    T_global_robot = time_xi_k0[:, 1:].reshape((-1, 4, 4))
    for i in range(len(timestamps)):
      time_T_global_robot[int(timestamps[i])] = T_global_robot[i]

  return time_T_global_robot


def plot_error(fig, errors):
  # switch to absolute error
  errors = np.abs(errors)

  plot_number = 311
  fig.set_size_inches(10, 10)
  fig.subplots_adjust(left=0.16, right=0.95, bottom=0.1, top=0.93, wspace=0.7, hspace=0.7)

  labels = ['x', 'y', 'z', 'x', 'y', 'z']
  for i in range(2):
    ax = fig.add_subplot(plot_number + i)
    # plot the errors
    ax.plot(errors[:, i], '-b', linewidth=1.0)
    # plot the mean
    ax.plot(np.ones_like(errors[:, i]) * np.mean(errors[:, i]), '-r', linewidth=2.0)
    ax.set_title("mean: " + str(np.mean(errors[:, i])) + ", var: " + str(np.var(errors[:, i])))
    ax.set_xlabel(r"frame")
    ax.set_ylabel(r"$|\hat{r}_x - r_x|$ [$m$]".replace("x", labels[i]))
    ax.set_ylim([0, 0.2])
  for i in range(5, 6):
    ax = fig.add_subplot(plot_number + i - 3)
    # plot the errors
    ax.plot(errors[:, i], '-b', linewidth=1.0)
    # plot the mean
    ax.plot(np.ones_like(errors[:, i]) * np.mean(errors[:, i]), '-r', linewidth=2.0)
    ax.set_title("mean: " + str(np.mean(errors[:, i])) + ", var: " + str(np.var(errors[:, i])))
    ax.set_xlabel(r"frame")
    ax.set_ylabel(r"$|\hat{\theta}_x - \theta_x|$ [$rad$]".replace("x", labels[i]))
    ax.set_ylim([0, 0.1])


def plot_error_box(fig,
                   errors: List[Tuple[str, np.ndarray]],
                   proc_func=lambda x: x,
                   ylabel=r"$\hat{tran}_dir - tran_dir$ [$unit$]"):
  plot_number = 311
  fig.set_size_inches(5 + 1.4 * len(errors), 12)
  fig.subplots_adjust(left=0.16, right=0.95, bottom=0.1, top=0.93, wspace=0.7, hspace=0.7)

  labels = ['x', 'y', 'z', 'x', 'y', 'z']
  for i in range(2):
    ax = fig.add_subplot(plot_number + i)
    # plot the errors
    ax.boxplot([proc_func(error[:, i]) for _, error in errors])
    ax.set_xticklabels([x for x, _ in errors], rotation=-25)
    ax.set_ylabel(ylabel.replace(r"tran", r"r").replace(r"unit", r"m").replace(r"dir", labels[i]))
    # ax.set_ylim([0.0, 0.25])
  for i in range(5, 6):
    ax = fig.add_subplot(plot_number + i - 3)
    # plot the errors
    ax.boxplot([proc_func(error[:, i]) for _, error in errors])
    ax.set_xticklabels([x for x, _ in errors], rotation=-25)
    ax.set_ylabel(ylabel.replace(r"tran", r"\theta").replace(r"unit", r"rad").replace(r"dir", labels[i]))
    # ax.set_ylim([-0.1, 0.1])


def main(dataset_dir, data_dir):
  data_dir = osp.normpath(data_dir)
  root_dir = osp.dirname(data_dir)
  odo_input = osp.basename(data_dir)
  loc_inputs = [i for i in os.listdir(data_dir) if i != odo_input]
  loc_inputs.sort()
  print("Root Directory:", root_dir)
  print("Odometry:", odo_input)
  print("Localization:", loc_inputs)

  # dataset directory and necessary sequences to load
  dataset_root = osp.abspath(osp.expanduser(osp.expandvars(dataset_dir)))
  dataset_seqs = [odo_input, *loc_inputs]
  print("Dataset Root:", dataset_root)
  print("Dataset Sequences:", dataset_seqs)
  ground_truth_poses = get_robot_poses(dataset_root, dataset_seqs)

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
      T_global_vertex = np.empty((len(messages), 4, 4))
      T_global_robot_list = np.empty((len(messages), 4, 4))
      T_global_robot_gt_list = np.empty((len(messages), 4, 4))
      for i, message in enumerate(messages):
        if not int(message[1].timestamp) in ground_truth_poses.keys():
          print("WARNING: time stamp not found (loc run): ", int(message[1].timestamp))
          continue
        if not int(message[1].vertex_timestamp) in ground_truth_poses.keys():
          print("WARNING: time stamp not found (map run): ", int(message[1].vertex_timestamp))
          continue

        robot_timestamp = int(message[1].timestamp)
        vertex_timestamp = int(message[1].vertex_timestamp)

        T_robot_vertex_vec = np.array(message[1].t_robot_vertex.xi)[..., None]
        T_robot_vertex = se3op.vec2tran(T_robot_vertex_vec)

        T_vertex_robot = npla.inv(T_robot_vertex)
        # print("estimated:", se3op.tran2vec(T_vertex_robot).flatten())

        # inv(T_enu_vertex) @ T_enu_robot = T_vertex_robot
        T_vertex_robot_gt = npla.inv(ground_truth_poses[vertex_timestamp]) @ ground_truth_poses[robot_timestamp]
        # print("ground truth:", se3op.tran2vec(T_vertex_robot_gt).flatten())

        # compute error
        errors[i, :] = se3op.tran2vec(T_robot_vertex @ T_vertex_robot_gt).flatten()

        T_global_vertex[i, :, :] = ground_truth_poses[vertex_timestamp]
        T_global_robot_list[i, :, :] = ground_truth_poses[vertex_timestamp] @ T_vertex_robot
        T_global_robot_gt_list[i, :, :] = ground_truth_poses[vertex_timestamp] @ T_vertex_robot_gt

      print(np.mean(np.abs(errors), axis=0))

      #
      date2trial_map[loc_input].append((trial.replace("rosbag2_", ""), errors))
      if trial not in trial2date_map.keys():
        trial2date_map[trial] = list()
      trial2date_map[trial].append((loc_input.replace("rosbag2_", ""), errors))

      fig = plt.figure()
      plot_error(fig, errors)
      fig.suptitle(odo_input + " <- " + loc_input + " : " + trial, fontsize=16)
      os.makedirs(osp.join(odo_input, loc_input), exist_ok=True)
      fig.savefig(osp.join(odo_input, loc_input, trial + '.png'))

      fig = plt.figure(figsize=(5, 10))
      ax = fig.add_subplot(111)
      ax.plot(T_global_vertex[:, 0, 3], T_global_vertex[:, 1, 3], 'b.', label='keyframe ground truth')
      ax.plot(T_global_robot_list[:, 0, 3], T_global_robot_list[:, 1, 3], 'r.', label='vehicle estimated')
      ax.plot(T_global_robot_gt_list[:, 0, 3], T_global_robot_gt_list[:, 1, 3], 'g.', label='vehicle ground truth')
      ax.legend(loc='lower left')
      ax.set_xlabel(r"x [m]")
      ax.set_ylabel(r"y [m]")
      ax.set_aspect('equal')
      fig.suptitle(odo_input + " <- \n" + loc_input + " : " + trial, fontsize=12)
      os.makedirs(osp.join(odo_input, loc_input), exist_ok=True)
      fig.savefig(osp.join(odo_input, loc_input, trial + '_path.png'))
      # plt.show()

  # plot box plot based on the two maps
  os.makedirs(odo_input, exist_ok=True)

  for k, v in date2trial_map.items():
    # error
    fig = plt.figure()
    plot_error_box(fig, v)
    fig.suptitle(odo_input + " <- " + k, fontsize=16)
    fig.savefig(osp.join(odo_input, k + '_box.png'))
    # np.abs(error)
    fig = plt.figure()
    plot_error_box(fig, v, np.abs, r"$|\hat{tran}_dir - tran_dir|$ [$unit$]")
    fig.suptitle(odo_input + " <- " + k, fontsize=16)
    fig.savefig(osp.join(odo_input, k + '_abs_box.png'))

  for k, v in trial2date_map.items():
    # error
    fig = plt.figure()
    plot_error_box(fig, v)
    fig.suptitle(odo_input + " point map version:" + k.replace("boreas.", ""), fontsize=16)
    fig.savefig(osp.join(odo_input, k + '_box.png'))
    # np.abs(err)
    fig = plt.figure()
    plot_error_box(fig, v, np.abs, r"$|\hat{tran}_dir - tran_dir|$ [$unit$]")
    fig.suptitle(odo_input + " point map version:" + k.replace("boreas.", ""), fontsize=16)
    fig.savefig(osp.join(odo_input, k + '_abs_box.png'))

  # plt.show()


if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--dataset', default=os.getcwd(), type=str, help='path to dataset (default: os.getcwd())')
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  main(args.dataset, args.path)