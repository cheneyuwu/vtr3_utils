import os
import os.path as osp
import argparse
import datetime
import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from pyproj import Proj

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

import sensor_msgs.point_cloud2 as pc2

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

    for j in range(len(messages)):

      points_msg = messages[j]

      print(points_msg[0])
      print(points_msg[1].header)
      print(points_msg[1].height, points_msg[1].width, points_msg[1].is_dense, points_msg[1].is_bigendian)
      for field in points_msg[1].fields:
        print(field)
      print(points_msg[1].point_step)
      print(points_msg[1].row_step)

      return


if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  load_points(args.path)
