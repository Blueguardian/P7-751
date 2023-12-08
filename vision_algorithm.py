import numpy as np
from scipy.linalg import expm
from scipy.optimize import least_squares

class VisionAlgorithm:

    def __init__(self):
        # Initialize static parameters for calculations

        # Focal length of Raspberry Pi V2.1 IMX219 sensor, taken from https://www.raspberrypi.com/documentation/accessories/camera.html
        self.f = 3.04 # In mm

        # Intrinsic camera matrix, obtained from testing
        self.intrinsic_matrix = np.array([[2770.28, 0, 1602.71], [0, 2765.57, 1200.56], [0,0,1.0]])

        # Projection matrix
        self.proj_mat = np.array([[self.f, 0, 0], [0, self.f, 0], [0, 0, 1]])

        # Pixels limits for Raspberry Pi V2.1 IMX219 sensor, taken from https://www.raspberrypi.com/documentation/accessories/camera.html
        self.u_lim = [0, 3280]
        self.v_lim = [0, 2464]

        self.points = np.array([[20, 20, 0], [8.7, 20, 0], [20, 7, 0], [8.7, 8, 0]]).T

    def _reprojection_func(self, params, points, init_rot):

        T = params[:3].T
        dR = expm(np.cross(np.eye(3), np.array(params[3:].T)))*init_rot
        proj_points = self._project_points(self.points, T, dR)
        reprojection_error = np.linalg.norm(points.T - proj_points)
        return reprojection_error

    def _project_points(self, T, R):
        Transform = np.hstack((R, T.reshape(-1, 1)))
        proj_matrix = np.dot(self.intrinsic_matrix, Transform[:3, :])
        homo_coords = np.dot(proj_matrix, np.vstack((self.points.T, np.ones((1, self.points.shape[1])))))
        pixel_pose = (homo_coords[:2, :] / homo_coords[2, :]) * self.intrinsic_matrix
        return pixel_pose.T


    def minimize(self, pixel_pose, R, T, omega):
        pixel_pose = np.reshape(pixel_pose, (2, 4)).T
        init_params = np.array([T[0], T[1], T[2], omega[0], omega[1], omega[2]])
        result = least_squares(self._reprojection_func, init_params, args=(pixel_pose, R), method='lm')
        return result[:3], result[3:]

