import os
import os.path as osp
import argparse
from typing import List, Tuple
import numpy as np
import numpy.linalg as npla
import csv

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from pylgmath import se3op

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
  odo_inputs = [i for i in os.listdir(data_dir)]
  print(odo_inputs)

  T_r_applanix = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

  for _, odo_input in enumerate(odo_inputs):
    odo_dir = osp.join(data_dir, odo_input)

    result_dir = osp.join(odo_dir, "boreas/graph/run_000000/data")
    if not osp.exists(result_dir):
      continue
    print("Looking at result directory:", result_dir)

    # get bag file
    bag_file = '{0}/{1}/{1}_0.db3'.format(osp.abspath(result_dir), "odometry_result")

    parser = BagFileParser(bag_file)
    messages = parser.get_bag_messages("odometry_result")

    result = []
    for _, message in enumerate(messages):
      timestamp = int(message[1].timestamp / 1e3)
      T_w_r_vec = np.array(message[1].t_world_robot.xi)[..., None]
      T_w_r = se3op.vec2tran(T_w_r_vec)
      T_w_a = T_w_r @ T_r_applanix
      T_a_w_res = npla.inv(T_w_a).flatten().tolist()[:12]
      result.append([timestamp] + T_a_w_res)

    with open(osp.join(data_dir, odo_dir+".txt"), "+w") as file:
      writer = csv.writer(file, delimiter=' ')
      writer.writerows(result)


if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  main(args.path)