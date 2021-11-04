import os
import os.path as osp
import argparse
import numpy as np
np.set_printoptions(linewidth=120,suppress=False)
import numpy.linalg as npla
import matplotlib
import matplotlib.pyplot as plt
from pyproj import Proj

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from pylgmath import so3op, se3op, Transformation
from pysteam.trajectory import Time, TrajectoryInterface
from pysteam.state import TransformStateVar, VectorSpaceStateVar
from pysteam.problem import OptimizationProblem, StaticNoiseModel, L2LossFunc, WeightedLeastSquareCostTerm
from pysteam.solver import GaussNewtonSolver
from pysteam.evaluator import TransformStateEvaluator, FixedTransformEvaluator, ComposeTransformEvaluator, PointToPointErrorEval


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
    rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
    return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]

  def get_bag_timestamps(self, topic_name):
    topic_id = self.topic_id[topic_name]
    rows = self.cursor.execute("SELECT timestamp FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
    return [x for x, in rows]

  def get_bag_msgs_iter(self, topic_name):

    topic_id = self.topic_id[topic_name]
    result = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id))
    while True:
      res = result.fetchone()
      if res is not None:
        yield [res[0], deserialize_message(res[1], self.topic_msg_message[topic_name])]
      else:
        break


def generate_ground_truth(bag_file, projection, start_gps_coord, start_xy_coord):

  try:
    parser = BagFileParser(bag_file)
    messages = parser.get_bag_messages("/fix")
    lidar_timestamps = []
    iter = parser.get_bag_msgs_iter("/points")
    for _, msg in iter:
      lidar_timestamps.append(int(msg.header.stamp.sec * 1e9) + msg.header.stamp.nanosec)
  except Exception as e:
    print("Could not open rosbag: {}".format(osp.dirname(bag_file)))
    return None

  print("Loaded ros bag: {} with number of gps messages {} and number of lidar messages {}".format(
      osp.dirname(bag_file), len(messages), len(lidar_timestamps)))

  gps_poses = {
      "timestamp": [],
      "latitude": [],
      "longitude": [],
      "altitude": [],
      "cov": [],
      "x": [],
      "y": [],
      'x_seg': [],
      'y_seg': [],
      'bad_run': True
  }

  # Collecting segements along a path that have good gps so we can plot a
  # list of good segmants and ignore the bad data.
  segment_ind = -1
  prev_vert_bad_gps = True

  num_large_covariance = 0

  for gps_msg in messages:

    # Check if covariance is good. Up to 0.1 is ok, I've chosen slightly
    # stricter setting here.
    covariance_is_large = gps_msg[1].position_covariance[0] > 0.1
    if covariance_is_large:
      num_large_covariance += 1
      prev_vert_bad_gps = True
      continue
    else:
      # Start a new segment of good GPS data that can be plotted.
      if prev_vert_bad_gps:
        gps_poses["x_seg"].append([])
        gps_poses["y_seg"].append([])
        segment_ind += 1
      prev_vert_bad_gps = False

    # Projecting GPS so we can plot in meters.
    x, y = projection(gps_msg[1].longitude, gps_msg[1].latitude)
    # x, y = gps_msg[1].longitude, gps_msg[1].latitude

    if not (len(start_gps_coord) and len(start_xy_coord)):
      # Want to plot with the start of the path at (0, 0)
      start_gps_coord.extend([gps_msg[1].latitude, gps_msg[1].longitude, gps_msg[1].altitude])
      start_xy_coord.extend([x, y])
      print("WARNING: new starting position: ", start_gps_coord, start_xy_coord)

    gps_poses["timestamp"].append(gps_msg[0])
    gps_poses["latitude"].append(gps_msg[1].latitude - start_gps_coord[0])
    gps_poses["longitude"].append(gps_msg[1].longitude - start_gps_coord[1])
    gps_poses["altitude"].append(gps_msg[1].altitude - start_gps_coord[2])
    gps_poses["x"].append(x - start_xy_coord[0])
    gps_poses["y"].append(y - start_xy_coord[1])
    gps_poses["cov"].append(gps_msg[1].position_covariance)

    gps_poses["x_seg"][segment_ind].append(x - start_xy_coord[0])
    gps_poses["y_seg"][segment_ind].append(y - start_xy_coord[1])

  gps_poses["bad_run"] = False  # bool(num_large_covariance > 0)

  gps_poses["rosbag_dir"] = osp.basename(osp.dirname(bag_file))

  print("=> Number of segments:", len(gps_poses["x_seg"]))
  print("=> Large cov {}: {}, total: {}".format(osp.dirname(bag_file), num_large_covariance, len(messages)))

  if len(gps_poses['timestamp']) == 0:
    return None

  return gps_poses

def generate_ground_truth_poses(data_dir):

  rosbag_dirs = [name for name in os.listdir(data_dir) if osp.isdir(osp.join(data_dir, name))]
  rosbag_dirs.sort()
  print(rosbag_dirs)

  proj_origin = (43.78210381666667, -79.46711405, 149.765)  # hardcoding for now - todo: get from ground truth CSV
  projection = Proj("+proj=etmerc +ellps=WGS84 +lat_0={0} +lon_0={1} +x_0=0 +y_0=0 +z_0={2} +k_0=1".format(
      proj_origin[0], proj_origin[1], proj_origin[2]))

  start_gps_coord = []
  start_xy_coord = []

  # # TODO hard-coded here for now
  start_gps_coord = [43.78210381666667, -79.46711405, 149.765]
  start_xy_coord = [0.0, 0.0]

  ## single threaded version
  tot_gps_poses = []

  for rosbag_dir in rosbag_dirs:
    bag_file = '{0}/{1}/{1}_0.db3'.format(data_dir, rosbag_dir)

    gps_poses = generate_ground_truth(bag_file, projection, start_gps_coord, start_xy_coord)

    if gps_poses is None:
      continue

    tot_gps_poses.append(gps_poses)

  return tot_gps_poses


def plot_all(tot_gps_poses, data_dir):

  fig = plt.figure(figsize=(5, 10))
  ax = fig.add_subplot(111)

  plot_lines = []
  labels = []

  for i in range(len(tot_gps_poses)):

    p = None
    for j in range(len(tot_gps_poses[i]["x_seg"])):
      p = ax.plot(tot_gps_poses[i]["x_seg"][j], tot_gps_poses[i]["y_seg"][j], color='C{}'.format(i))

    if p is not None:
      plot_lines.append(p[0])
      labels.append(tot_gps_poses[i]["rosbag_dir"])

  ax.set_ylabel('y (m)', fontsize=14, weight='bold')
  ax.set_xlabel('x (m)', fontsize=14, weight='bold')
  ax.set_aspect('equal')
  # ax.set_xticks(fontsize=14)
  # ax.set_yticks(fontsize=14)
  ax.set_title('GPS ground truth, teach and repeat runs', fontsize=12, weight='bold')
  ax.legend(plot_lines, labels, fontsize=8, loc='lower left')
  fig.savefig('{}/gps_paths.png'.format(data_dir), bbox_inches='tight', format='png')
  plt.show()


if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  tot_gps_poses = generate_ground_truth_poses(args.path)

  plot_all(tot_gps_poses, args.path)