import numpy as np
from numpy import (cos, sin)
from scipy.spatial.transform import Rotation


class EKF():

    def initialise_ekf(self):
        X = np.array([0] * 9).reshape(-1, 1)  # Column of current state X = [x,y,z,vx,vy,vz,r,p,w].T
        U = np.array([0] * 6).reshape(-1, 1)  # Inputs of expected perturbances U = [ax,ay,az,gy_x,gy_y,gy_z].T

        # state covariance matrix. (NxN)
        self.P = np.zeros((9, 9))  # - initialized to zero for first update
        # np.fill_diagonal(self.P, [
        # 	10, 10, 10,  # Covariance state position. 	x,y,z.
        # 	4, 4, 4,  # Covariance state speed.		x_vel,y_vel,z_vel.
        # 	np.pi / 2, np.pi / 2, np.pi / 2,  # Covariance state tilt.
        # 	np.pi / 4, np.pi / 4, np.pi / 4  # Covariance state angular speed.
        # ])

        # system error matrix Q (12,12)
        # - adds 4.5 meter std dev in x and y position to state covariance
        # - adds 2 meters per second std dev in x and y velocity to state covariance
        # - these values are not optimized but work for this example
        self.Q = np.zeros((9, 9))
        np.fill_diagonal(self.Q, [
            1, 1, 1,  # Error state position.
            0.5, 0.5, 0.5,  # Error state speed.
            np.pi, np.pi, np.pi,  # Error state tilt.
            np.pi / 2, np.pi / 2, np.pi / 2  # Error state angular speed.
        ])

        self.X = X
        self.U = U

        return self.X

    def prediction_step(self, sensor_measurements, dt):
        # State transistion matrix (9,9)
        A = np.array([  # x y z	 vx vy  vz  r  p  w
            [1, 0, 0, dt, 0, 0, 0, 0, 0],  # x
            [0, 1, 0, 0, dt, 0, 0, 0, 0],  # y
            [0, 0, 1, 0, 0, dt, 0, 0, 0],  # z
            [0, 0, 0, 1, 0, 0, 0, 0, 0],  # Vx
            [0, 0, 0, 0, 1, 0, 0, 0, 0],  # Vy
            [0, 0, 0, 0, 0, 1, 0, 0, 0],  # Vz
            [0, 0, 0, 0, 0, 0, 1, 0, 0],  # Roll
            [0, 0, 0, 0, 0, 0, 0, 1, 0],  # Pitch
            [0, 0, 0, 0, 0, 0, 0, 0, 1],  # Yaw
        ])

        ang_vel = sensor_measurements[3:]
        accel = sensor_measurements[:3]
        roll, pitch, yaw = self.X[-3:, 0].copy()

        offset = Rotation.from_euler("ZYX", (yaw, pitch, roll)).apply([0, 0, -9.80665])
        accel += offset  # Removing gravity.

        U = np.append(accel, ang_vel).reshape(-1, 1)  # Inputs of expected perturbances

        # Control matrix (9,6) - Adds expected exteneral perturbances.
        B = np.array([  # ax  ay  az vr  vp  vw
            [0., 0, 0, 0, 0, 0],  # x
            [0, 0, 0, 0, 0, 0],  # y
            [0, 0, 0, 0, 0, 0],  # z
            [dt, 0, 0, 0, 0, 0],  # Vx
            [0, dt, 0, 0, 0, 0],  # Vy
            [0, 0, dt, 0, 0, 0],  # Vz
            [0, 0, 0, dt, 0, 0],  # Roll
            [0, 0, 0, 0, dt, 0],  # Pitch
            [0, 0, 0, 0, 0, dt],  # Yaw
        ])

        # Predict State Forward
        X_predict = A.dot(self.X) + B.dot(U)

        # Predict Covariance Forward
        P_predict = A.dot(self.P).dot(A.T) + self.Q

        self.X = X_predict
        self.P = P_predict

        return X_predict

    def prediction_step_2_imus(self, sensor_measurements, dt):
        # State transistion matrix (9,9)
        A = np.array([  # x y z	 vx vy  vz  r  p  w
            [1, 0, 0, dt, 0, 0, 0, 0, 0],  # x
            [0, 1, 0, 0, dt, 0, 0, 0, 0],  # y
            [0, 0, 1, 0, 0, dt, 0, 0, 0],  # z
            [0, 0, 0, 1, 0, 0, 0, 0, 0],  # Vx
            [0, 0, 0, 0, 1, 0, 0, 0, 0],  # Vy
            [0, 0, 0, 0, 0, 1, 0, 0, 0],  # Vz
            [0, 0, 0, 0, 0, 0, 1, 0, 0],  # Roll
            [0, 0, 0, 0, 0, 0, 0, 1, 0],  # Pitch
            [0, 0, 0, 0, 0, 0, 0, 0, 1],  # Yaw
        ])

        acc1 = sensor_measurements[:3]
        gyr1 = sensor_measurements[3:6]
        acc2 = sensor_measurements[6:9]
        gyr2 = sensor_measurements[9:12]

        roll, pitch, yaw = self.X[-3:, 0].copy()
        offset = Rotation.from_euler("ZYX", (yaw, pitch, roll)).apply([0, 0, -9.80665])
        acc1 += offset  # Removing gravity.
        acc2 += offset  # Removing gravity.

        U = np.append(acc1, gyr1)
        U = np.append(U, acc2)
        U = np.append(U, gyr2)
        U = U.reshape(-1, 1)  # Inputs of expected perturbances (accel1, gyro1, accel2, gyro2)

        # Control matrix (9,12) - Adds expected exteneral perturbances.
        B = np.array([  # ax1   ay1   az1   vr1   vp1   vw2   ax2   ay2   az2   vr2   vp2   vw2
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # x
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # y
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # z
            [dt / 2, 0, 0, 0, 0, 0, dt / 2, 0, 0, 0, 0, 0],  # Vx
            [0, dt / 2, 0, 0, 0, 0, 0, dt / 2, 0, 0, 0, 0],  # Vy
            [0, 0, dt / 2, 0, 0, 0, 0, 0, dt / 2, 0, 0, 0],  # Vz
            [0, 0, 0, dt / 2, 0, 0, 0, 0, 0, dt / 2, 0, 0],  # Roll
            [0, 0, 0, 0, dt / 2, 0, 0, 0, 0, 0, dt / 2, 0],  # Pitch
            [0, 0, 0, 0, 0, dt / 2, 0, 0, 0, 0, 0, dt / 2],  # Yaw
        ])

        # Predict State Forward
        X_predict = A.dot(self.X) + B.dot(U)

        # Predict Covariance Forward
        P_predict = A.dot(self.P).dot(A.T) + self.Q
        self.X = X_predict
        self.P = P_predict

        return X_predict

    def measurement_step_barometer(self, sensor_measurements):
        # self.prediction_step()

        # Predicted states to measurements transition matrix (1,9)
        H = np.array([  # x	y  z  	vx vy vz	r  p  w
            [0, 0, 1, 0, 0, 0, 0, 0, 0],  # alt
        ])

        # measurement covariance matrix
        R = 0.5

        # Compute Kalman Gain
        S = H.dot(self.P).dot(H.T) + R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))

        # Column with the current measurements from the sensors(z).
        current_z = sensor_measurements

        # Compute column with the predicted sensor measurements.
        z = self.X[2, 0].copy()

        h_small = z

        # compute the residual
        # - the difference between the state and measurement for that data time
        residual = current_z - h_small

        # Compute new estimate for state vector using the Kalman Gain
        self.X = self.X + K * residual

        # Compute new estimate for state covariance using the Kalman Gain
        self.P = self.P - K.dot(H).dot(self.P)

        return self.X

    def measurement_step_barometer_2_imus(self, sensor_measurements):
        # self.prediction_step()

        # Predicted states to measurements transition matrix (2,9)
        H = np.array([  # x	y  z  	vx vy vz	r  p  w
            [0, 0, 1, 0, 0, 0, 0, 0, 0],  # alt1
            [0, 0, 1, 0, 0, 0, 0, 0, 0],  # alt2
        ])

        # measurement covariance matrix
        R = np.zeros((2, 2))
        np.fill_diagonal(R, [
            1, 1  # covariance for barometre,	alt1, alt2.
        ])

        # Compute Kalman Gain
        S = H.dot(self.P).dot(H.T) + R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))

        # Column with the current measurements from the sensors(z).
        current_z = sensor_measurements.reshape(-1, 1)

        # Compute column with the predicted sensor measurements.
        z = self.X[2, 0].copy()

        h_small = np.array([
            [z],  # alt1
            [z],  # alt2
        ])

        # compute the residual
        # - the difference between the state and measurement for that data time
        residual = current_z - h_small

        # Compute new estimate for state vector using the Kalman Gain
        self.X = self.X + K.dot(residual)

        # Compute new estimate for state covariance using the Kalman Gain
        self.P = self.P - K.dot(H).dot(self.P)

        return self.X

    def measurement_step_vision(self, sensor_measurements):
        # Predict State Forward
        # self.prediction_step()

        # Predicted states to measurements transition matrix (16,9)
        H = np.array([  # x	y  z  vx    vy     vz	    r      p      w
            [1, 0, 0, 0, 0, 0, 0, 0, 0],  # vis_x
            [0, 1, 0, 0, 0, 0, 0, 0, 0],  # vis_y
            [0, 0, 1, 0, 0, 0, 0, 0, 0],  # vis_z
            [0, 0, 0, 0, 0, 0, 1, 0, 0],  # vis_roll
            [0, 0, 0, 0, 0, 0, 0, 1, 0],  # vis_pitch
            [0, 0, 0, 0, 0, 0, 0, 0, 1],  # vis_yaw
        ])

        # measurement covariance matrix
        R = np.zeros((6, 6))
        np.fill_diagonal(R, [
            1, 1, 1, 1, 1, 1  # covariance for barometre.
        ])

        # Compute Kalman Gain
        S = H.dot(self.P).dot(H.T) + R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))

        # Column with the current measurements from the sensors(z).
        current_z = sensor_measurements.reshape(-1, 1)

        # Compute column with the predicted sensor measurements.
        x, y, z, vx, vy, vz, r, p, w = self.X
        h_small = np.array([
            x,  # vis_x
            y,  # vis_y
            z,  # vis_z
            r,  # vis_r
            p,  # vis_p
            w,  # vis_w
        ])

        # compute the residual
        # - the difference between the state and measurement for that data time
        residual = current_z - h_small

        # Compute new estimate for state vector using the Kalman Gain
        self.X = self.X + K.dot(residual)

        # Compute new estimate for state covariance using the Kalman Gain
        self.P = self.P - K.dot(H).dot(self.P)

        return self.X

    def measurement_step_magnetometer(self, sensor_measurements):
        # self.prediction_step()

        r, p, w = self.X[-3:, 0]  # Predicted roll, pitch and yaw.
        # Get the derivative of mx, my and mz
        dmx = [.0698 * cos(r) * sin(w) - .0698 * cos(w) * sin(r) * sin(p),
               .0698 * cos(r) * cos(w) + .0698 * sin(r) * sin(p) * sin(w), -.0698 * cos(p) * sin(r)]
        dmy = [.0698 * cos(r) * cos(p) * cos(w) - .998 * cos(w) * sin(p),
               .998 * sin(p) * sin(w) - .0698 * cos(r) * cos(p) * sin(w), -.998 * cos(p) - .0698 * cos(r) * sin(p)]
        dmz = [-.0698 * cos(w) * sin(r) - .988 * cos(p) * sin(w) - .0698 * cos(r) * sin(p) * sin(w),
               -.0698 * sin(r) * sin(w) - .998 * cos(p) * cos(w) - .0698 * cos(r) * cos(w) * sin(p), 0]

        # Predicted states to measurements transition matrix (3,9)
        H = np.array([  # x	y  z  	vx vy vz	   r      p      w
            [0, 0, 0, 0, 0, 0, dmx[0], dmx[1], dmx[2]],  # mag_x
            [0, 0, 0, 0, 0, 0, dmy[0], dmy[1], dmy[2]],  # mag_y
            [0, 0, 0, 0, 0, 0, dmz[0], dmz[1], dmz[2]]  # mag_z
        ])

        # measurement covariance matrix
        R = np.zeros((3, 3))
        np.fill_diagonal(R, [
            5, 5, 5,  # covariance for magnetometer,	mx,my,mz.
        ])

        # Compute Kalman Gain
        S = H.dot(self.P).dot(H.T) + R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))

        # Column with the current measurements from the sensors(z).
        current_z = sensor_measurements.reshape(-1, 1)

        # Compute column with the predicted sensor measurements.
        mx = 0.0698 * sin(r) * sin(w) + 0.998 * cos(p) * cos(w) + 0.0698 * cos(r) * cos(w) * sin(p)
        my = 0.0698 * cos(w) * sin(r) - 0.998 * cos(p) * sin(w) - 0.0698 * cos(r) * sin(p) * sin(w)
        mz = 0.0698 * cos(p) * cos(r) - 0.998 * sin(p)

        h_small = np.array([
            [mx],  # mag_x
            [my],  # mag_y
            [mz],  # mag_z
        ])

        # compute the residual
        # - the difference between the state and measurement for that data time
        residual = current_z - h_small

        # Compute new estimate for state vector using the Kalman Gain
        self.X = self.X + K.dot(residual)

        # Compute new estimate for state covariance using the Kalman Gain
        self.P = self.P - K.dot(H).dot(self.P)

        return self.X



