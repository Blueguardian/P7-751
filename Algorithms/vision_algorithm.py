import numpy as np
from scipy.linalg import expm
from scipy.optimize import least_squares

class VisionAlgorithm:

    def __init__(self):
        # Initialize static parameters for calculations

        # Focal length of Raspberry Pi V2.1 IMX219 sensor, taken from https://www.raspberrypi.com/documentation/accessories/camera.html
        self.f = 3.04 # In mm

        # Intrinsic camera matrix, obtained from testing 3x3
        self.intrinsic_matrix = np.array([[2770.28, 0, 1602.71], [0, 2765.57, 1200.56], [0,0,1.0]])

        # Projection matrix 3x3
        self.proj_mat = np.array([[(self.f * 3.68)/1920, 0, 0,0], [0, (self.f*2.76)/1080, 0,0], [0, 0, 1,0]])

        # Pixels limits for Raspberry Pi V2.1 IMX219 sensor, taken from https://www.raspberrypi.com/documentation/accessories/camera.html
        self.u_lim = [0, 3280]
        self.v_lim = [0, 2464]

        self.points = np.array([[20, 20, 0], [8.7, 20, 0], [20, 7, 0], [8.7, 8, 0]]).reshape(-1, 3)

    def _reprojection_func(self, params, points, init_rot):
        print(f"Params, T: {params[:3]}")
        T = params[:3]
        print(f"T: {T}")
        dR = (expm(np.cross(np.eye(3), np.array(params[3:6].T))))*R
        print(f"Delta Rotation:  {dR}")
        reprojection_error = 0
        for point in range(0, 1, self.points.shape[0]):
            proj_points = self._project_points(self.points[point], T, dR)
            print(f"Project points:  {proj_points}")
            reprojection_error += np.linalg.norm(points[point].T - proj_points) / self.points.shape[0]
            print(f"Reprojection error:  {reprojection_error}")
        return reprojection_error

    def _project_points(self, point, T, R):
        #proj_matrix = np.dot(self.proj_mat, R.T)
        #print(f"Projection matrix: {proj_matrix}")
        homo_coords = np.dot(R.transpose(), (point[:3]-T))




        # trans = np.hstack([R, T.reshape(-1,1)])
        #
        # trans = np.vstack([trans, [0,0,0,1]])
        #
        # homo_coords = self.proj_mat.dot(trans.dot(np.vstack([point.reshape(-1,1),1])))


        print(f"Homogeneous coordinates:  {homo_coords}")
        pixel_pose = homo_coords[:2]
        print(f"Updated pixel pose: {pixel_pose}")
        return pixel_pose.T


    def minimize(self, pixel_pose, R, T, omega):
        print(f"Before pixel pose: {pixel_pose}")
        pixel_pose = np.reshape(pixel_pose, (2, 4)).T
        print(f"Pixel pose:  {pixel_pose}")
        print(f"R:  {R}")
        print(f"T:  {T}")
        print(f"omega:  {omega}")
        init_params = np.array([T[0], T[1], T[2], omega[0], omega[1], omega[2]])
        print(f"init_params:  {init_params}")
        result = least_squares(self._reprojection_func, init_params, args=(pixel_pose, R))
        return result

if __name__ == '__main__':
    vision = VisionAlgorithm()


    pixel_coords_red = np.array([770, 620])
    pixel_coords_blue = np.array([455, 620])
    pixel_coords_green = np.array([776, 455])
    pixel_coords_yellow = np.array([620, 620])

    pixel_coords = np.append((pixel_coords_red), (pixel_coords_blue, pixel_coords_green, pixel_coords_yellow))
    print(pixel_coords)
    R = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    T = np.array([0.0, 0.0, 1000.0])

    omega = np.array([0.1,0.1,0.1])


    results = vision.minimize(pixel_coords, R, T, omega)

    #print(f"Output results: {results}")

    dT = results.x[:3]
    dR = R+expm(results.x[1]*np.zeros((3,3)))
    print(f"Derivative of Rotation matrix: {dR}")
    print(f"New translation: {T}")
    print(f"Residual of minimisation: {results.fun} \nTermination condition description: {results.message} \nSucces?: {results.success} \n")


    print("Projected points: \n")
    for point in range(0, vision.points.shape[1]+1):
        proj_point = vision._project_points(vision.points[point], T, dR)
        print(f"Point {point+1}: {proj_point.reshape(1, -1)}")
        # (np.dot(vision.proj_mat, R)

    I = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    T = np.array([1,0,0])
    p = np.array([1, 2, 3])

    POS = vision._project_points(p, T, I)