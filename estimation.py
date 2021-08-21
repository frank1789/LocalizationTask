import math

import os
import sys
import matplotlib.pyplot as plt
import numpy as np

from parse_file import ParseData

print(os.path.dirname(sys.argv[0]))
a = ParseData(os.path.dirname(sys.argv[0]))

dt = 0
# EKF data
Q = np.diag([0.5, 0.5, np.deg2rad(2.0), 0.5]) ** 2
R = np.diag([0.7, 0.7]) ** 2

# Noise params
white_noise = np.diag([0.1, np.deg2rad(30.0)]) ** 2
gnss_noise = np.diag([5, 5]) ** 2  # set from data


def input_system():
    """ compute input starting from odometric values"""
    vx = a.odometry_data[:, 2:3]  # linear velocity_y [m/s]
    vy = a.odometry_data[:, 1:2]  # linear velocity_x [m/s]
    v = np.add(vx, vy)
    v = np.true_divide(v, 2)  # combined velocity [m/s]
    yawrate = np.reshape(a.odometry_data[:, 3], (-1, 1))  # angular_z [rad/s]
    u = np.reshape([v, yawrate], (-1, 2))
    return u


def perform_observation(xTrue, xd, u):
    xTrue = car_model(xTrue, u)
    # add noise to gps x-y
    z = observation(xTrue) + gnss_noise @ np.random.randn(2, 1)
    # add noise to input
    ud = u + white_noise @ np.random.randn(2, 1)
    xd = car_model(xd, ud)
    return xTrue, z, xd, ud


def car_model(x, u):
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[dt * math.cos(x[2, 0]), 0],
                  [dt * math.sin(x[2, 0]), 0],
                  [0.0, dt],
                  [1.0, 0.0]])

    x = F @ x + B @ u
    return x


def observation(x):
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
    ])
    z = H @ x
    return z


def jacobian(x, u):
    """
    compute the Jacobian for motion model

    x_{t+1} = x_t + v*dt * cos(yaw)
    y_{t+1} = y_t + v*dt * sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}

    obtain
    dx/dyaw = -v * dt * sin(yaw)
    dx/dv = dt * cos(yaw)
    dy/dyaw = v * dt * cos(yaw)
    dy/dv = dt * sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jac = np.array([
        [1.0, 0.0, -dt * v * math.sin(yaw), dt * math.cos(yaw)],
        [0.0, 1.0, dt * v * math.cos(yaw), dt * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jac


def jacobian_observation():
    j_obs = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
    ])

    return j_obs


def perfome_estimation(qEst, pEst, z, u):
    qPred = car_model(qEst, u)
    jac = jacobian(qEst, u)
    PPred = jac @ pEst @ jac.T + Q

    #  update value
    jH = jacobian_observation()
    z_pred = observation(qPred)
    y = z_pred
    S = jH @ PPred @ jH.T + R
    K = PPred @ jH.T @ np.linalg.inv(S)
    qEst = qPred + K @ y
    pEst = (np.eye(len(qEst)) - K @ jH) @ PPred
    return qEst, pEst


if __name__ == '__main__':
    xEst = np.zeros((4, 1))  # initialize estimation as [0, 0, 0]
    # initialize x original position starting from ground_truth
    xTrue = np.add(np.zeros((4, 1)), np.array([a.groundtruth_data[0, 1:2], a.groundtruth_data[0, 2:3], [0], [0]]))
    pEst = np.eye(4)  # initial Pest [0,0,0,0]
    xDR = np.zeros((4, 1))
    # store information to plot follow
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((2, 1))

    for i in range(0, len(a.odometry_data[:, 0])):
        dt = a.odometry_data[i, 0]
        u = input_system()[i]  # extract input from odom data
        # update x True from ground truth
        xTrue = np.add(np.zeros((4, 1)), np.array([a.groundtruth_data[i, 1:2], a.groundtruth_data[i, 2:3], [0], [0]]))
        xTrue, z, xDR, ud = perform_observation(xTrue, xDR, u)
        xEst, pEst = perfome_estimation(xEst, pEst, z, ud)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.hstack((hz, z))

        # plot
        plt.cla()
        plt.plot(hz[0, :], hz[1, :], ".g")
        plt.plot(hxTrue[0, :].flatten(),
                 hxTrue[1, :].flatten(), "-b")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)
