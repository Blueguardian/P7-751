import numpy as np
from scipy.linalg import expm
from scipy.optimize import least_squares
from math import pi, cos, sin
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

        # Pixels limits for Raspberry Pi V2.1 IMX219 sensor, taken from https://www.raspberrypi.com/documentation/accessories/camera.html
        self.u_lim = [0, 3280]
        self.v_lim = [0, 2464]

        self.points = np.array([[-5.5, 5, 0], [5.5, 5, 0], [-5.5, -5, 0], [5.5, -5, 0], [11, 5, 0], [11, -5, 0]]).reshape(-1, 3)

    def _reprojection_func(self, params, points, init_rot, init_pose):
        pixel_coords_red = np.array([938.0, 657.0])
        pixel_coords_blue = np.array([629.0, 653.0])
        pixel_coords_green = np.array([949.0, 508.0])
        pixel_coords_yellow = np.array([645.0, 504.0])
        pixel_coords_pink = np.array([770.0 + (770.0 - 455.0), 620.0 + (770.0 - 445.0)])
        pixel_coords_fawk_green = np.array([455.0 + (770.0 - 455.0), 620.0 + (770.0 - 445.0)])
        pixel_coords = np.append((pixel_coords_red), (
        pixel_coords_blue, pixel_coords_green, pixel_coords_yellow, pixel_coords_pink, pixel_coords_fawk_green))
        pixel_coords_1 = pixel_coords.reshape(-1, 2)

        print(f"Params, T: {params[:3]}")
        delta_T = params[:3]
        print(f"T: {delta_T}")
        dR = (expm(np.cross(np.eye(3), np.array(params[3:6].T))))*init_rot
        T = init_pose + delta_T
        print(f"Delta Rotation:  {dR}")
        proj_points = self._project_points(self.points, T, dR)
        print(f"Project points:  {proj_points}")
        dif_points = points - proj_points
        reprojection_error = np.linalg.norm(dif_points, axis=1)**2
        print(f"Reprojection error:  {reprojection_error}")

        plt.plot(proj_points[:, 0], proj_points[:, 1], 'bo', pixel_coords_1[:, 0], pixel_coords_1[:, 1], 'go', )

        return reprojection_error


    def _project_points(self, points, T, R):
        print(f"Original points: {points}")
        transl_points = points.transpose().copy()-T.reshape((3,1)).copy()
        print(f"points transposed: {points.transpose()}")
        print(f"Translated points: {transl_points.transpose()}")
        rot_points = np.dot(R.transpose().copy(), transl_points.copy())
        print(f"rotated_points: {rot_points.transpose()}")
        homo_coords = rot_points[:3].copy() / rot_points[2].copy()
        print(f"Homogeneous coords: {homo_coords.transpose()}")
        proj_points = np.dot(self.proj_mat.copy(), homo_coords.copy())
        print(f"Updated pixel pose: {proj_points.transpose()}")
        offset_points = proj_points[:2].copy() #+np.array([1920/2, 1080/2]).reshape(-1, 1)
        return offset_points.transpose().copy()


    def minimize(self, pixel_pose, R, T, omega, vel):
        print(f"Before pixel pose: {pixel_pose}")
        pixel_pose = np.reshape(pixel_pose.copy(), (2, -1)).T
        init_params = np.array([vel[0].copy(), vel[1].copy(), vel[2].copy(), omega[0].copy(), omega[1].copy(), omega[2].copy()])
        print(f"init_params:  {init_params}")
        result = least_squares(self._reprojection_func, init_params, args=(pixel_pose, R, T), method='lm')
        print(result)
        return result

if __name__ == '__main__':
    vision = VisionAlgorithm()


    pixel_coords_red = np.array([938.0, 657.0])
    pixel_coords_blue = np.array([629.0, 653.0])
    pixel_coords_green = np.array([949.0, 508.0])
    pixel_coords_yellow = np.array([645.0, 504.0])
    pixel_coords_pink = np.array([770.0+(770.0-455.0), 620.0+(770.0-445.0)])
    pixel_coords_fawk_green = np.array([455.0+(770.0-455.0), 620.0+(770.0-445.0)])


    vec = np.array([0, pi, 0])

    yaw = np.array(
        [[1, 0, 0], [0, cos(vec[2]), -sin(vec[2])], [0, sin(vec[2]), cos(vec[2])]])
    pitch = np.array(
        [[cos(vec[1]), 0, sin(vec[1])], [0, 1, 0], [-sin(vec[1]), 0, cos(vec[1])]])
    roll = np.array(
        [[cos(vec[0]), -sin(vec[0]), 0], [sin(vec[0]), cos(vec[0]), 0], [0, 0, 1]])

    pixel_coords = np.append((pixel_coords_red), (pixel_coords_blue, pixel_coords_green, pixel_coords_yellow, pixel_coords_pink, pixel_coords_fawk_green))
    pixel_coords_1 = pixel_coords.reshape(-1, 2)
    plt.xlim(-1920, 1920)
    plt.ylim(-1080, 1080)
    plt.grid()
    #plt.plot(projected[:, 0], projected[:, 1], 'bo', pixel_coords[:, 0], pixel_coords[:, 1], 'go', )
    plt.plot(pixel_coords_1[:, 0], pixel_coords_1[:, 1], 'go', )
    plt.show()
    R = np.dot(np.dot(roll, pitch), yaw)
    T = np.array([0.0, 0.0, 0.6])

    omega = np.array([0.1,0.1,0.1])
    vel = np.array([0.1,0.1,0.1])


    results = vision.minimize(pixel_coords, R, T, omega, vel)

    #print(f"Output results: {results}")

    dT = T + results.x[:3]
    dR = expm(np.cross(results.x[3:], np.eye((3))))*R
    print(f"Derivative of Rotation matrix: {dR}")
    print(f"New translation: {dT}")
    print(f"Residual of minimisation: {results.fun} \nTermination condition description: {results.message} \nSucces?: {results.success} \n")

    projected = np.array([[0,0],[0,0],[0,0],[0,0],[0,0], [0,0]])

    print("Projected points: \n")
    projected = vision._project_points(vision.points.reshape(-1, 3), dT, dR)[:4, :]
    # (np.dot(vision.proj_mat, R)
    print(f"projected points: {projected}")
    pixel_coords = pixel_coords.reshape(-1, 2)[:4, :]
    print(f"Pixel coords: {pixel_coords}")

    plt.xlim(-1920, 1920)
    plt.ylim(-1080, 1080)
    plt.grid()
    plt.plot(projected[:, 0], projected[:, 1], 'bo', pixel_coords[:, 0], pixel_coords[:, 1], 'go', )
    #plt.plot(pixel_coords[:, 0], pixel_coords[:, 1], 'go', )
    plt.show()
    I = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    T = np.array([1,0,0])
    p = np.array([[1, 2, 3], [4, 5, 6]])

    POS = vision._project_points(p, T, I)