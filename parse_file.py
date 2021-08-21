from datetime import datetime

import numpy as np


class ParseData:
    def __init__(self, dataset):
        self.load_data(dataset)

    def load_data(self, dataset, ):

        # Ground truth: [Time[s], x[m], y[m], orientation[rad]]
        self.groundtruth_data = np.loadtxt(dataset + "/ground_truth.csv", delimiter=',', skiprows=1)
        # GNSS: [Time[epoch], x[m], y[m]]
        self.gnss_data = np.loadtxt(dataset + "/gnss.csv", delimiter=',', skiprows=1)
        # Measurement: [Time[epoch], Subject#, range[m], bearing[rad]]
        self.measurement_data = np.loadtxt(dataset + "/odom.csv", delimiter=',', skiprows=1)
        # Odometry: [Time[epoch], Subject#, forward_V[m/s], angular _v[rad/s]]
        self.odometry_data = np.loadtxt(dataset + "/odom.csv", delimiter=',', skiprows=1)


        t0 = self.odometry_data[0, 0] // 1e9
        for i in range(1, len(self.odometry_data[:, 0])):
            t = datetime.utcfromtimestamp(self.odometry_data[i, 0] // 1e9) - datetime.utcfromtimestamp(t0)
            self.odometry_data[i, 0] = t.total_seconds()
        self.odometry_data[0, 0] = 0
        print(self.odometry_data[:, 0])

        t0 = self.measurement_data[0, 0] // 1e9
        for i in range(1, len(self.measurement_data[:, 0])):
            t = datetime.utcfromtimestamp(self.measurement_data[i, 0] // 1e9) - datetime.utcfromtimestamp(t0)
            self.measurement_data[i, 0] = t.total_seconds()
        self.measurement_data[0, 0] = 0
        print(self.measurement_data[:, 0])

        t0=self.groundtruth_data[0, 0] // 1e9
        for i in range(1, len(self.groundtruth_data[:, 0])):
            t = datetime.utcfromtimestamp(self.groundtruth_data[i, 0] // 1e9) - datetime.utcfromtimestamp(t0)
            self.groundtruth_data[i, 0] = t.total_seconds()
        self.groundtruth_data[0, 0] = 0
        print(self.groundtruth_data[:, 0])


if __name__ == "__main__":
    a = ParseData(
        "/Users/francesco/PycharmProjects/pythonProject1/Crover Application - Robotics Software Engineer Challenge")
