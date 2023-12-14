import numpy as np
from scipy.linalg import expm
from scipy.optimize import least_squares
from math import pi, cos, sin
import time
from matplotlib import pyplot as plt

class VisionAlgorithm:

    def __init__(self):
        # Initialize static parameters for calculations

        # Focal length of Raspberry Pi V2.1 IMX219 sensor, taken from https://www.raspberrypi.com/documentation/accessories/camera.html
        self.f = 3.04 # In mm

        # Intrinsic camera matrix, obtained from testing 3x3
        self.intrinsic_matrix = np.array([[2770.28, 0, 1602.71], [0, 2765.57, 1200.56], [0,0,1.0]])

        # Projection matrix 3x3
        self.proj_mat = np.array([[(self.f / (3.68))*1920, 0, 0], [0, (self.f/(2.76))*1080, 0], [0, 0, 1]])

        self.points = np.array([[-5.5, 5, 0], [5.5, 5, 0], [-5.5, -5, 0], [5.5, -5, 0]]).reshape(-1, 3)/100 # Original points in m.


    def _reprojection_func(self, params, points, init_rot, init_pose):
        delta_T = params[:3]
        dR = (expm(np.cross(np.eye(3), np.array(params[3:6].T))))*init_rot
        T = init_pose + delta_T
        proj_points = self._project_points(self.points, T, dR)
        reprojection_error = np.linalg.norm(points - proj_points, axis=1)**2

        return reprojection_error

    def _project_points(self, points, T, R):
        transl_points = points.transpose()-T.reshape((3,1))
        rot_points = np.dot(R.transpose(), transl_points)
        homo_coords = rot_points[:3] / rot_points[2]
        proj_points = np.dot(self.proj_mat, homo_coords)
        offset_points = proj_points[:2] 
        return offset_points.transpose()

    def minimize(self, pixel_pose, R, T):
        pixel_pose = np.reshape(pixel_pose, (-1, 2))
        omega = np.array([0.1,0.1,0.1])
        vel = np.array([0.1,0.1,0.1])
        init_params = np.array([vel[0], vel[1], vel[2], omega[0], omega[1], omega[2]])
        result = least_squares(self._reprojection_func, init_params, args=(pixel_pose, R, T), ftol= 1e-2)
        dT = T + result.x[:3]
        dR = expm(np.cross(result.x[3:], np.eye((3))))*R

        return dT, dR



if __name__ == '__main__':

    vision = VisionAlgorithm()


    pixel_coords_red = np.array([938.0, 657.0])
    pixel_coords_blue = np.array([629.0, 653.0])
    pixel_coords_green = np.array([949.0, 508.0])
    pixel_coords_yellow = np.array([645.0, 504.0])

    pixel_coords = np.append((pixel_coords_red), (pixel_coords_blue, pixel_coords_green, pixel_coords_yellow))


    vec = np.array([0, pi, 0])
    yaw = np.array(
        [[1, 0, 0], [0, cos(vec[2]), -sin(vec[2])], [0, sin(vec[2]), cos(vec[2])]])
    pitch = np.array(
        [[cos(vec[1]), 0, sin(vec[1])], [0, 1, 0], [-sin(vec[1]), 0, cos(vec[1])]])
    roll = np.array(
        [[cos(vec[0]), -sin(vec[0]), 0], [sin(vec[0]), cos(vec[0]), 0], [0, 0, 1]])


    # Initial parameters:

    R = np.dot(np.dot(roll, pitch), yaw)
    T = np.array([0.0, 0.0, 0.6])

    omega = np.array([0.1,0.1,0.1])
    vel = np.array([0.1,0.1,0.1])


    # Getting the optimal parameters:


    results = vision.minimize(pixel_coords.copy(), R, T, omega, vel)
    # print("Results of minimisation:",results.x[:3], results.x[3:], "\n")


    # Testing the optimal parameters.

    dT = T + results.x[:3]
    dR = expm(np.cross(results.x[3:], np.eye((3))))*R
    # print(f"Derivative of Rotation matrix: {dR}")
    # print(f"New translation: {dT}")
    print(f"Residual of minimisation: {results.fun} \nTermination condition description: {results.message} \nSucces?: {results.success} \n")

    projected = np.array([[0,0],[0,0],[0,0],[0,0],[0,0], [0,0]])

    projected = vision._project_points(vision.points.reshape(-1, 3), dT, dR)[:4, :]
    print(f"projected points: \n{projected}")
