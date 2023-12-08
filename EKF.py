import numpy as np
from numpy import (cos, sin)

class EKF():

	def ekf(self, updateNumber, sensed_state, sensor_measurements, dt):

		X = sensed_state.reshape(-1,1)	# Column of current state X = [x,y,z,vx,vy,vz,r,p,w].T

		accel = sensor_measurements[:3]
		ang_vel = sensor_measurements[3:6]
		U = np.append(accel,ang_vel).reshape(-1,1) # Inputs of expected perturbances

		# State transistion matrix (9,9)
		A = np.array([#  x  y  z	vx  vy  vz   r  p  w     vr  vp  vw
						[1, 0, 0,	dt, 0,  0, 	 0, 0, 0], # x
						[0, 1, 0, 	0,  dt, 0,	 0, 0, 0], # y
						[0, 0, 1, 	0,  0,  dt,  0, 0, 0], # z
						[0, 0, 0, 	1,  0,  0,	 0, 0, 0], # Vx
						[0, 0, 0, 	0,  1,  0,	 0, 0, 0], # Vy
						[0, 0, 0, 	0,  0,  1,	 0, 0, 0], # Vz
						[0, 0, 0, 	0,  0,  0,	 1, 0, 0], # Roll
						[0, 0, 0, 	0,  0,  0,	 0, 1, 0], # Pitch
						[0, 0, 0, 	0,  0,  0,	 0, 0, 1], # Yaw
					])
		
		# Control matrix (9,6) - Adds expected exteneral perturbances.
		B = np.array([ #  ax  ay  az     vr  vp  vw
						[ 0,  0,  0,	 0,  0,  0], # x
						[ 0,  0,  0,	 0,  0,  0], # y
						[ 0,  0,  0,	 0,  0,  0], # z
						[dt, 0,   0,	 0,  0,  0], # Vx 
						[ 0, dt,  0,	 0,  0,  0], # Vy
						[ 0,  0, dt,	 0,  0,  0], # Vz
						[ 0,  0,  0,	dt,  0,  0], # Roll
						[ 0,  0,  0,	 0, dt,  0], # Pitch
						[ 0,  0,  0,	 0,  0, dt], # Yaw
					])


		if updateNumber == 0: # First update.

			# state covariance matrix. (NxN)
			self.P = np.zeros((12,12))	# - initialized to zero for first update
									
			# measurement covariance matrix. (MxM)
			self.R = np.zeros((16,16))
			np.fill_diagonal(self.R,[	
										1,1,1,						# covariance for accelerometer, ax,ay,az.
										np.pi/2,np.pi/2,np.pi/2,	# covariance for gyroscope, 	gx,gy,gz.
										5,5,5,						# covariance for magnetometer,	mx,my,mz.
										1,							# covariance for barometer,		alt.
										1,1,1,						# covariance for vision pos,	x,y,z.
										np.pi/2,np.pi/2,np.pi/2		# covariance for vision RPY, 	roll, pitch, yaw.
									])

			# system error matrix
			# - initialized to zero matrix for first update. (NxN)
			self.Q = np.zeros((12,12))

			# residual and kalman gain. Kalman Gain (NxM).
			# - not computed for first update
			# - but initialized so it could be output
			self.residual = np.zeros((16,1))
			
			K = np.zeros((12,16))

			self.old_X = X
			self.old_U = U


		# Reinitialize State
		if updateNumber == 1: # Second Update

			# state covariance matrix
			# - initialized to large values
			# - more accurate position values can be used based on measurement
			#   covariance but this example does not go that far
			self.P = np.zeros((12,12))
			np.fill_diagonal(self.P,[	
										10,10,10,					# Covariance state position. 	x,y,z.
										4,4,4,						# Covariance state speed.		x_vel,y_vel,z_vel. 
										np.pi/2,np.pi/2,np.pi/2,	# Covariance state tilt.
										np.pi/4,np.pi/4,np.pi/4		# Covariance state angular speed.
									])

			# measurement covariance matrix
			# - provided by the measurment source
			self.R = self.R

			# system error matrix Q (12,12)
			# - adds 4.5 meter std dev in x and y position to state covariance
			# - adds 2 meters per second std dev in x and y velocity to state covariance
			# - these values are not optimized but work for this example
			self.Q = np.zeros((12,12))
			np.fill_diagonal(self.Q,[	
										0.1,0.1,0.1,				# Error state position. 	x,y,z.
										0.1,0.1,0.1,				# Error state speed.		x_vel,y_vel,z_vel. 
										np.pi/2,np.pi/2,np.pi/2,	# Error state tilt.
										np.pi/4,np.pi/4,np.pi/4		# Error state angular speed.
									])	
			
			# residual and kalman gain
			# - not computed for first update
			# - but initialized so it could be output
			self.residual = self.residual
			self.K = self.K

			self.old_X = X
			self.old_U = U


		if updateNumber > 1:
			# Predict State Forward
			X_predict = A.dot(self.old_X) + B.dot(self.old_U)

			# Predict Covariance Forward
			P_predict = A.dot(self.P).dot(A.T) + self.Q

			_,_,_,vx,vy,vz,r,p,w = X_predict
			mx = [.0698*cos(r)*sin(w)-.0698*cos(w)*sin(r)*sin(p), .0698*cos(r)*cos(w)+.0698*sin(r)*sin(p)*sin(w), -.0698*cos(p)*sin(r)]
			my = [.0698*cos(r)*cos(p)*cos(w)-.998*cos(w)*sin(p), .998*sin(p)*sin(w)-.0698*cos(r)*cos(p)*sin(w), -.998*cos(p)-.0698*cos(r)*sin(p)]
			mz = [-.0698*cos(w)*sin(r)-.988*cos(p)*sin(w)-.0698*cos(r)*sin(p)*sin(w), -.0698*sin(r)*sin(w)-.998*cos(p)*cos(w)-.0698*cos(r)*cos(w)*sin(p), 0]
			
			# Predicted states to measurements transition matrix (16,9)
			H = np.array([ # x	y  z	  vx    vy     vz	    r      p      w  
							[0, 0, 0,	 vx*dt,  0,    0, 		0,     0,     0,  ],	# acc_x
							[0, 0, 0, 	   0,  vy*dt,  0,		0,     0,     0,  ],	# acc_y
							[0, 0, 0, 	   0,    0,  vz*dt,   	0,     0,     0,  ], 	# acc_z
							[0, 0, 0, 	   0,    0,    0,	   r*dt,   0,     0,  ],	# gyr_x
							[0, 0, 0, 	   0,    0,    0,		0,    p*dt,   0,  ],	# gyr_y
							[0, 0, 0, 	   0,    0,    0,		0,     0,    w*dt ],	# gyr_z
							[0, 0, 0, 	   0,    0,    0,     mx[0], mx[1], mx[2] ],	# mag_x
							[0, 0, 0, 	   0,    0,    0,     my[0], my[1], my[2] ],	# mag_y
							[0, 0, 0, 	   0,    0,    0,     mz[0], mz[1], mz[2] ],	# mag_z
							[0, 0, 1, 	   0,    0,    0,       0,      0,    0   ],	# alt
							[1, 0, 0, 	   0,    0,    0,       0,      0,    0   ],	# vis_x
							[0, 1, 0, 	   0,    0,    0,       0,      0,    0   ],	# vis_y
							[0, 0, 1, 	   0,    0,    0,       0,      0,    0   ],	# vis_z
							[0, 0, 0, 	   1,    0,    0,       0,      0,    0   ],	# vis_vx
							[0, 0, 0, 	   0,    1,    0,       0,      0,    0   ],	# vis_vy
							[0, 0, 0, 	   0,    0,    1,       0,      0,    0   ],	# vis_vz
						])			

			# measurement covariance matrix
			self.R = self.R
			
			# Compute Kalman Gain
			S = H.dot(P_predict).dot(H.T) + self.R
			K = P_predict.dot(H.T).dot(np.linalg.inv(S))

			# Column with the current measurements from the sensors(z).
			current_z = sensor_measurements.reshape(-1,1)


			# Compute column with the predicted sensor measurements.
			x,y,z,vx,vy,vz,r,p,w =X_predict
			mx = 0.0698*sin(r)*sin(w) + 0.998*cos(p)*cos(w) + 0.0698*cos(r)*cos(w)*sin(p)
			my = 0.0698*cos(w)*sin(r) - 0.998*cos(p)*sin(w) - 0.0698*cos(r)*sin(p)*sin(w)
			mz = 0.0698*cos(p)*cos(r) - 0.998*sin(p)

			h_small = np.array([ 
									[vx/dt],	# acc_x
									[vy/dt],	# acc_y
									[vz/dt], 	# acc_z
									[r/dt],		# gyr_x
									[p/dt],		# gyr_y
									[w/dt],		# gyr_z
									[mx],		# mag_x
									[my],		# mag_y
									[mz],		# mag_z
									[z],		# alt
									[x],		# vis_x
									[y],		# vis_y
									[z],		# vis_z
									[vx],		# vis_vx
									[vy],		# vis_vy
									[vz],		# vis_vz
							   ])			

			# compute the residual
			# - the difference between the state and measurement for that data time
			residual = current_z - h_small

			# Compute new estimate for state vector using the Kalman Gain
			self.old_X = X_predict + K.dot(residual)

			# Compute new estimate for state covariance using the Kalman Gain
			self.P = P_predict - K.dot(H).dot(P_predict)

		return [self.old_X, self.P, K, residual]
	