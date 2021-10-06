import sys
import time

sys.path.append("/home/yuchen/ASRL/data/boreas/boreas-devkit/python")

from boreas import BoreasDataset


class BoreasDatasetWrapper(BoreasDataset):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def get_lidar_iter(self,
                       seq=0,
                       time=False,
                       pose=False,
                       start=0,
                       stop=None):
        for i, frame in enumerate(
                self.get_seq(seq).lidar_frames[start:stop], start):
            res = [frame.load_data()]
            if pose:
                res = [frame.pose, *res]
            if time:
                res = [frame.timestamp, *res]
            yield res


dataset = BoreasDatasetWrapper(
    "/home/yuchen/ASRL/data/boreas/sequences",
    [["boreas-2020-12-01-13-26"], ["boreas-2020-12-18-13-44"]])

T_applanix_lidar = dataset.sequences[0].calib.T_applanix_lidar


# Ground truth is provided w.r.t sensor, so we set sensor to vehicle
# transform to identity
yfwd2xfwd = np.array([
    [0, 1, 0, 0],
    [-1, 0, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
])
self.T_robot_lidar = yfwd2xfwd @ T_applanix_lidar
self.T_lidar_robot = npla.inv(self.T_robot_lidar)


lidar_iter = dataset.get_lidar_iter(0, True, False)

for [timestamp, points] in lidar_iter:
    print(timestamp)
