import os
import os.path as osp
import argparse
import numpy as np
import numpy.linalg as npla
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import gridspec

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


from pyboreas import BoreasDataset
from pylgmath import se3op

# matplotlib.use("TkAgg")  # matplotlib.use("TkAgg")


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
    print(topics_data)
    topics_data = [(1, "localization_result", "vtr_lidar_msgs/msg/ICPResult")]  # need to add this type info manually
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

  def get_bag_msgs_iter(self, topic_name):

    topic_id = self.topic_id[topic_name]
    result = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id))
    while True:
      res = result.fetchone()
      if res is not None:
        yield [res[0], deserialize_message(res[1], self.topic_msg_message[topic_name])]
      else:
        break



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
    ax.set_xlabel(r"$t$ [$s$]")
    ax.set_ylabel(r"$|\hat{r}_x - r_x|$ [$m$]".replace("x", labels[i]))
    ax.set_ylim([0, 1])
  for i in range(3, 6):
    ax = fig.add_subplot(plot_number + i)
    # plot the errors
    ax.plot(errors[:, i], '-b', linewidth=1.0)
    # plot the mean
    ax.plot(np.ones_like(errors[:, i]) * np.mean(errors[:, i]), '-r', linewidth=2.0)
    ax.set_title("mean: " + str(np.mean(errors[:, i])) + ", var: " + str(np.var(errors[:, i])))
    ax.set_xlabel(r"$t$ [$s$]")
    ax.set_ylabel(r"$|\hat{\theta}_x - \theta_x|$ [$rad$]".replace("x", labels[i]))
    ax.set_ylim([0, 0.1])

def main(data_dir):
  data_dir = osp.normpath(data_dir)
  root_dir = osp.dirname(data_dir)
  odo_input = osp.basename(data_dir)
  loc_inputs = [i for i in os.listdir(data_dir) if i != odo_input]
  loc_inputs.sort()
  print("Root Directory:", root_dir)
  print("Odometry:", odo_input)
  print("Localization:", loc_inputs)

  for i, loc_input in enumerate(loc_inputs):
    loc_dir = osp.join(root_dir, odo_input, loc_input)
    trials = list(os.listdir(loc_dir))
    trials.sort()

    trial_msg_iter = []
    for j, trial in enumerate(trials):
      if trial != "boreas.v2" and trial != "boreas.v1":
        continue
      result_dir = osp.join(loc_dir, trial, "graph/run_000001/data")
      if not osp.exists(result_dir):
        continue
      print("Looking at result directory:", result_dir)

      # get bag file
      bag_file = '{0}/{1}/{1}_0.db3'.format(osp.abspath(result_dir), "localization_icp_result")

      parser = BagFileParser(bag_file)
      msg_iter = parser.get_bag_msgs_iter("localization_result")
      trial_msg_iter.append((trial, msg_iter))

    fig = plt.figure()
    fig.set_size_inches(20, 5)
    gs = gridspec.GridSpec(1, 2, width_ratios=[1, 2])
    # fig.subplots_adjust(left=0.16, right=0.95, bottom=0.1, top=0.93, wspace=0.7, hspace=0.7)

    ax1 = fig.add_subplot(gs[0]); ax1.set_ylim(top=12000); bin_max1 = 30; hist_bins1 = np.linspace(0, bin_max1, 100)
    ax2 = fig.add_subplot(gs[1]); ax2.set_ylim(top=2000); bin_max2 = 10; hist_bins2 = np.linspace(0, bin_max2, 100)

    bar_containers1 = []
    bar_containers2 = []
    for trial, _ in trial_msg_iter:
      _, _, bar_container1 = ax1.hist(np.random.randn(1000), hist_bins1, lw=1, alpha=0.5, label=trial)
      bar_containers1.append(bar_container1)
      _, _, bar_container2 = ax2.hist(np.random.randn(1000), hist_bins2, lw=1, alpha=0.5, label=trial)
      bar_containers2.append(bar_container2)

    ax1.set_xlabel("planar distance to NN in point map [m], bin size 0.3m")
    ax1.set_ylabel("number of points in scan")
    ax2.set_xlabel("planar distance to NN in point map [m], bin size 0.1m")
    ax2.set_ylabel("number of points in scan")

    ax1.legend()
    ax2.legend()

    title1 = ax1.text(0.5, 0.95, "", # bbox={'facecolor':'w', 'alpha':0.5, 'pad':5},
                      transform=ax1.transAxes, ha="center")
    title2 = ax2.text(0.5, 0.95, "", # bbox={'facecolor':'w', 'alpha':0.5, 'pad':5},
                      transform=ax2.transAxes, ha="center")

    def animate(frame_msg):
        frame = frame_msg[0]
        msg = frame_msg[1]
        print("Processing frame ", frame)

        # simulate new data coming in
        for i, bar_container in enumerate(bar_containers1):
          # dist = np.sqrt(msg[i][1].nn_dists)  # point distance
          dist = msg[i][1].nn_planar_dists  # plan distance
          if (max(dist) > bin_max1):
            print("WARNING: max distance is ", max(dist))
          n, _ = np.histogram(dist, hist_bins1)
          for count, rect in zip(n, bar_container.patches):
              rect.set_height(count)
        assert msg[0][0] == msg[1][0]
        title1.set_text(u"frame: {}, timestamp: {}".format(frame, msg[0][0]))

        # simulate new data coming in
        for i, bar_container in enumerate(bar_containers2):
          # dist = np.sqrt(msg[i][1].nn_dists)  # point distance
          dist = msg[i][1].nn_planar_dists  # plan distance
          if (max(dist) > bin_max2):
            print("WARNING: max distance is ", max(dist))
          n, _ = np.histogram(dist, hist_bins2)
          for count, rect in zip(n, bar_container.patches):
              rect.set_height(count)
        assert msg[0][0] == msg[1][0]
        title2.set_text(u"frame: {}, timestamp: {}".format(frame, msg[0][0]))

        updated = [title1, title2]
        updated.extend([x for bar_container in bar_containers1 for x in bar_container.patches])
        updated.extend([x for bar_container in bar_containers2 for x in bar_container.patches])

        return updated

    ani = animation.FuncAnimation(fig, animate, enumerate(zip(*[x[1] for x in trial_msg_iter])), repeat=False, blit=True, save_count=10400, interval=200)
    # plt.show()
    ani.save(osp.join(odo_input, loc_input, "icp_error.mp4"))

    break


if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <rosbag name>/metadata.yaml
  # <rosbag name>/<rosbag name>_0.db3
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  main(args.path)