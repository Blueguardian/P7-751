import numpy as np
from math import cos, sin
from scipy.spatial.transform import Rotation

class Pose_tracker:

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

    def get_transform(self):
        return np.hstack([self.rot_mat, np.array([self._x, self._y, self._z]).T])

    def get_ang_vel(self):
        return np.array([self._x_ang_vel, self._y_ang_vel, self._z_ang_vel]).T

    def get_rot_vec(self):
        r = Rotation.from_matrix(self.rot_mat)
        return r.as_euler('xyz')

    def interpret_sensor_output(self, sensor_output):

        sensor_output = sensor_output.reshape(6, 3)

        accel_data = sensor_output[0, :]
        gyro_data = sensor_output[1, :]
        magnet_data = sensor_output[2, :]
        alt = sensor_output[3, 2]

        self.time_now = sensor_output[5, 2]

        self._x_vel = self._x_vel + (accel_data[0]*(self.time_now-self.time_prior))
        self._y_vel = self._y_vel + (accel_data[1] * (self.time_now - self.time_prior))
        self._z_vel = self._z_vel + (accel_data[2] * (self.time_now - self.time_prior))

        self._x = self._x + (self._x_vel*(self.time_now-self.time_prior)) + 0.5*(accel_data[0]*(self.time_now-self.time_prior)**2)
        self._y = self._y + (self._y_vel*(self.time_now-self.time_prior)) + 0.5*(accel_data[1]*(self.time_now-self.time_prior)**2)
        self._z = self._z + (self._z_vel*(self.time_now-self.time_prior)) + 0.5*(accel_data[2]*(self.time_now-self.time_prior)**2)

        yaw = np.array([[1,0,0], [0, cos(magnet_data(0)), -sin(magnet_data[0])], [0, sin(magnet_data[0], cos(magnet_data[0]))]])
        pitch = np.array([[cos(magnet_data[1]), 0, sin(magnet_data[1])], [0, 1, 0], [-sin(magnet_data[1]), 0, cos(magnet_data[1])]])
        roll = np.array([[cos(magnet_data[2]), -sin(magnet_data[2]), 0], [sin(magnet_data[2]), cos(magnet_data[2]), 0], [0, 0, 1]])

        self.rot_mat = roll*pitch*yaw

        self._x_ang_vel = gyro_data[0]
        self._y_ang_vel = gyro_data[1]
        self._z_ang_vel = gyro_data[2]