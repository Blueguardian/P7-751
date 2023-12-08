import numpy as np
from math import cos, sin
from scipy.spatial.transform import Rotation

class Pose_tracker:
    """
    Very rough pose tracker, coded specifically for the tasks required
    Will not document
    """
    def __init__(self):

        self._x = 0
        self._y = 0
        self._z = 0

        self._x_vel = 0
        self._y_vel = 0
        self._z_vel = 0

        self._x_ang_vel = 0
        self._y_ang_vel = 0
        self._z_ang_vel = 0

        self.rot_mat = None
        self.time_prior = 0
        self.time_now = 0
        self.delta_time = 0

    def get_transform(self):
        return np.hstack([self.rot_mat, np.array([self._x, self._y, self._z]).T])

    def get_ang_vel(self):
        return np.array([self._x_ang_vel, self._y_ang_vel, self._z_ang_vel]).T

    def get_rot_vec(self):
        r = Rotation.from_matrix(self.rot_mat)
        return r.as_euler('xyz')

    def get_time(self):
        return self.delta_time

    def update_values_EFK(self, vec):
        self._x = vec[0]
        self._y = vec[1]
        self._z = vec[2]
        self._x_vel = vec[3]
        self._y_vel = vec[4]
        self._z_vel = vec[5]

        yaw = np.array(
            [[1, 0, 0], [0, cos(vec[8]), -sin(vec[8])], [0, sin(vec[8], cos(vec[8]))]])
        pitch = np.array(
            [[cos(vec[7]), 0, sin(vec[7])], [0, 1, 0], [-sin(vec[7]), 0, cos(vec[7])]])
        roll = np.array(
            [[cos(vec[6]), -sin(vec[6]), 0], [sin(vec[6]), cos(vec[6]), 0], [0, 0, 1]])

        self.rot_mat = roll * pitch * yaw

    def update_values_data(self, sensor_output):
        sensor_output = sensor_output.reshape(6, 3)
        gyro_data = sensor_output[1, :]

        self._x_ang_vel = gyro_data[0]
        self._y_ang_vel = gyro_data[1]
        self._z_ang_vel = gyro_data[2]

        self.time_prior = self.time_now
        self.time_now = sensor_output[5, 2]
        self.delta_time = self.time_now - self.time_prior
