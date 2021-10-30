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


class BagFileParser():

  def __init__(self, bag_file):
    try:
      self.conn = sqlite3.connect(bag_file)
    except Exception as e:
      print('Could not connect: ', e)
      raise Exception('could not connect')

    self.cursor = self.conn.cursor()

    table_names = self.cursor.execute("SELECT name FROM sqlite_master WHERE type='table';").fetchall()

    ## create a message type map
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

def load_gps(bag_file, projection, start_gps_coord, start_xy_coord):

  gps_poses = {"timestamp": [], "latitude": [], "longitude": [], "altitude": [], "cov": [], "x": [], "y": []}
  gps_good_segments = {'x': [], 'y': []}

  try:
    parser = BagFileParser(bag_file)
    messages = parser.get_bag_messages("/fix")
  except Exception as e:
    print("Could not open rosbag: {}".format(osp.dirname(bag_file)))
    return gps_poses, gps_good_segments, True

  print("Loaded ros bag: {}".format(osp.dirname(bag_file)))

  # Collecting segements along a path that have good gps so we can plot a
  # list of good segmants and ignore the bad data.
  segment_ind = -1
  prev_vert_bad_gps = True

  num_large_covariance = 0

  for gps_msg in messages:

    # Check if covariance is good. Up to 0.1 is ok, I've chosen slightly
    # stricter setting here.
    covariance_is_large = gps_msg[1].position_covariance[0] >= 0.05
    if covariance_is_large:
      num_large_covariance += 1
      prev_vert_bad_gps = True
      continue
    else:
      # Start a new segment of good GPS data that can be plotted.
      if prev_vert_bad_gps:
        gps_good_segments["x"].append([])
        gps_good_segments["y"].append([])
        segment_ind += 1
      prev_vert_bad_gps = False

    # Projecting GPS so we can plot in meters.
    x, y = projection(gps_msg[1].longitude, gps_msg[1].latitude)
    # x, y = gps_msg[1].longitude, gps_msg[1].latitude

    if not (len(start_gps_coord) and len(start_xy_coord)):
      # Want to plot with the start of the path at (0, 0)
      start_gps_coord.extend([gps_msg[1].latitude, gps_msg[1].longitude, gps_msg[1].altitude])
      start_xy_coord.extend([x, y])

    gps_poses["timestamp"].append(gps_msg[0])
    gps_poses["latitude"].append(gps_msg[1].latitude - start_gps_coord[0])
    gps_poses["longitude"].append(gps_msg[1].longitude - start_gps_coord[1])
    gps_poses["altitude"].append(gps_msg[1].altitude - start_gps_coord[2])
    gps_poses["x"].append(x - start_xy_coord[0])
    gps_poses["y"].append(y - start_xy_coord[1])
    gps_poses["cov"].append(gps_msg[1].position_covariance[0])

    gps_good_segments["x"][segment_ind].append(x - start_xy_coord[0])
    gps_good_segments["y"][segment_ind].append(y - start_xy_coord[1])

  print("=> Number of segments:", len(gps_good_segments['x']))
  if num_large_covariance > 0:
    print("=> Large cov {}: {}, total: {}".format(osp.dirname(bag_file), num_large_covariance, len(messages)))

  return gps_poses, gps_good_segments, False # bool(num_large_covariance > 0)


def load_gps_poses(data_dir):

  rosbag_dirs = [name for name in os.listdir(data_dir) if osp.isdir(osp.join(data_dir, name))]
  rosbag_dirs.sort()

  proj_origin = (43.7822845, -79.4661581, 169.642048)  # hardcoding for now - todo: get from ground truth CSV
  projection = Proj("+proj=etmerc +ellps=WGS84 +lat_0={0} +lon_0={1} +x_0=0 +y_0=0 +z_0={2} +k_0=1".format(
      proj_origin[0], proj_origin[1], proj_origin[2]))

  start_gps_coord = []
  start_xy_coord = []

  tot_gps_poses = []
  tot_gps_good_segments = []
  bad_runs = []

  for rosbag_dir in rosbag_dirs:
    bag_file = '{0}/{1}/{1}_0.db3'.format(data_dir, rosbag_dir)


    gps_poses, gps_good_segments, bad_run = load_gps(bag_file, projection, start_gps_coord, start_xy_coord)
    if gps_poses is None:
      continue

    tot_gps_poses.append(gps_poses)
    tot_gps_good_segments.append(gps_good_segments)
    if bad_run:
      bad_runs.append(rosbag_dir)

  print("Bad GPS runs: {}".format(bad_runs))

  return tot_gps_poses, tot_gps_good_segments, rosbag_dirs, bad_runs


def plot_data(tot_gps_poses, tot_gps_good_segments, gps_runs, bad_gps_runs, data_dir):

  # plt.figure(figsize=(22, 15))
  plot_lines = []
  labels = []

  for i in range(len(tot_gps_poses)):

    p = None
    for j in range(len(tot_gps_good_segments[i]["x"])):
      p = plt.plot(tot_gps_good_segments[i]["x"][j], tot_gps_good_segments[i]["y"][j], linewidth=2, color='C{}'.format(i))

    if p is not None:
      plot_lines.append(p[0])
      labels.append(gps_runs[i])

  plt.ylabel('y (m)', fontsize=20, weight='bold')
  plt.xlabel('x (m)', fontsize=20, weight='bold')
  plt.xticks(fontsize=20)
  plt.yticks(fontsize=20)
  plt.title('GPS ground truth, teach and repeat runs', fontsize=22, weight='bold')
  plt.legend(plot_lines, labels, fontsize=12)
  plt.show()
  # plt.savefig('{}/gps_paths.png'.format(data_dir), bbox_inches='tight', format='png')
  plt.close()


if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  tot_gps_poses, tot_gps_good_segments, gps_runs, bad_gps_runs = load_gps_poses(args.path)

  plot_data(tot_gps_poses, tot_gps_good_segments, gps_runs, bad_gps_runs, args.path)
