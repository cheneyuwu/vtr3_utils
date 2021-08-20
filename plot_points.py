import os
import os.path as osp
import argparse
import datetime
import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

import ros2_numpy as rnp

import numpy as np
import numpy.linalg as npla

matplotlib.use("TkAgg")


def cart2pol(pts: np.ndarray) -> np.ndarray:
  """Converts points from cartesian to polar.
  Args:
    pts (np.ndarray): points in cartesian coordinate [x, y, z]
  Returns:
    np.ndarray: points in polar coordinate [rho, theta, phi]
  """
  rho = npla.norm(pts, axis=-1, keepdims=True)
  phi = np.arctan2(pts[..., 1:2], pts[..., 0:1])
  theta = np.arctan2(np.sqrt(pts[..., 0:1]**2 + pts[..., 1:2]**2), pts[..., 2:3])
  print(rho.shape, phi.shape, theta.shape)
  return np.concatenate((rho, theta, phi), axis=-1)


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


def load_points(data_dir):

  rosbag_dirs = [name for name in os.listdir(data_dir) if osp.isdir(osp.join(data_dir, name))]
  rosbag_dirs.sort()

  for i in range(len(rosbag_dirs)):
    bag_file = '{0}/{1}/{1}_0.db3'.format(data_dir, rosbag_dirs[i])

    try:
      parser = BagFileParser(bag_file)
      messages = parser.get_bag_messages("/points")
    except Exception as e:
      print("Could not open rosbag for run {}".format(rosbag_dirs[i]))
      continue

    print("Loaded ros bag: {}".format(rosbag_dirs[i]))

    # fig = plt.figure(i)
    # ax = fig.add_subplot(projection='3d')
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    # ax.set_xlim([-40, 40])
    # ax.set_ylim([-40, 40])
    # ax.set_zlim([-10, 10])

    for j in range(len(messages)):

      points_msg = messages[j]

      print(points_msg[0])
      print(points_msg[1].header)
      print(points_msg[1].height, points_msg[1].width, points_msg[1].is_dense, points_msg[1].is_bigendian)
      print(points_msg[1].point_step, points_msg[1].row_step)
      for field in points_msg[1].fields:
        print(field)

      data = rnp.numpify(points_msg[1])

      # # for j in range(len(data['x'])):
      # #   ax.scatter(data['x'][j], data['y'][j], data['z'][j], s=1, c='b')

      # #   plt.draw()
      # #   plt.pause(0.0001)

      filter = np.where(data['beam_side'])

      filtered_pitch = data['pitch'][filter]
      
      num_line = 0
      prev_pitch = 100
      for a in filtered_pitch:
        if prev_pitch < -50 and a > -50:
          num_line += 1
        prev_pitch = a
      input("Number of lines is {}".format(num_line))

      # # side
      # fig = plt.figure(i)
      # ax = fig.add_subplot(511)
      # ax.plot(np.arange(data['beam_side'][filter].shape[0]), data['beam_side'][filter])
      # ax.set_ylabel("beam_side")

      # ax = fig.add_subplot(512)
      # ax.plot(np.arange(data['yaw'][filter].shape[0]), data['yaw'][filter])
      # ax.set_ylabel("yaw")

      # ax = fig.add_subplot(513)
      # ax.plot(np.arange(data['pitch'][filter].shape[0]), data['pitch'][filter])
      # ax.set_ylabel("pitch")

      # ax = fig.add_subplot(514)
      # ax.plot(np.arange(data['range'][filter].shape[0]), data['range'][filter])
      # ax.set_ylabel("range")

      # ax = fig.add_subplot(515)
      # ax.plot(np.arange(data['y'][filter].shape[0]), data['y'][filter])
      # ax.set_ylabel("y")

      # points = np.stack([data['x'], data['y'], data['z']], axis=-1)

      # # polpts = cart2pol(points[..., :3])
      # # print(polpts.shape)

      # # ax = fig.add_subplot(413)
      # # ax.plot(np.arange(polpts.shape[0]), polpts[:, 2])

      # # for i in range(1, polpts.shape[0]):
      # #   if polpts[i, 2] - polpts[i - 1, 2] > np.pi:
      # #     polpts[i, 2] -= 2 * np.pi
      # #   elif polpts[i, 2] - polpts[i - 1, 2] < -np.pi:
      # #     polpts[i, 2] += 2 * np.pi

      # # ax = fig.add_subplot(414)
      # # ax.plot(np.arange(polpts.shape[0]), polpts[:, 2])
      plt.show()
      # fig.clear()


if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  load_points(args.path)
