import numpy as np
import matplotlib.pyplot as plt
 
np.printoptions(suppress=True)

data_imu = np.loadtxt("only_imu_best.csv", delimiter=",", dtype=np.float64)
data_imu[:,-1] -= data_imu[0,-1]

data_camera = np.loadtxt("imu_and_camera_best.csv", delimiter=",", dtype=np.float64)
data_camera[:,-1] -= data_camera[0,-1]


samples = 19+38
time = data_camera[:samples,-1]


# Initialise the subplot function using number of rows and columns 
figure, axis = plt.subplots(1,1) 

# axis.plot(time, data_imu[:samples,8], label = "Yaw (IMU)")
# axis.plot(time, data_camera[:samples,8], label = "Yaw (camera and IMU)")
# axis.plot(time, [0]*len(time), linestyle="dashed")
# axis.set_xlabel("Time [s]")
# axis.set_ylabel("Angle [rad]")
# axis.legend()
# axis.grid()
# axis.set_xlim(0,time[samples-1])

# plt.savefig('foo.png', dpi=199)   



axis.plot(time, data_imu[:samples,0], label = "X (IMU)") 
axis.plot(time, data_camera[:samples,0], label = "X (camera and IMU)")
axis.plot(time, [0]*len(time), label = "Ground truth", linestyle="dashed", linewidth=1)
axis.set_xlabel("Time [s]\n(a)")
axis.set_ylabel("Distance [m]")
axis.set_xlim(0,time[samples-1])
axis.tick_params(axis='both', which='major', labelsize=12)

# axis[1].plot(time, data_imu[:samples,1], label = "Y (IMU)") 
# axis[1].plot(time, data_camera[:samples,1], label = "Y camera and IMU") 
# axis[1].plot(time, [0]*len(time), label = "Ground truth", linestyle="dashed", linewidth=1)
# axis[1].set_xlabel("Time [s]")
# axis[1].set_ylabel("Distance [m]")
# axis[1].set_xlim(0,time[samples-1])
# axis[1].tick_params(axis='both', which='major', labelsize=12)
# axis[1].set_xlabel("Time [s]\n(b)")

# axis[2].plot(time, data_imu[:samples,2], label = "Z (IMU)") 
# axis[2].plot(time, data_camera[:samples,2], label = "Z (camera and IMU)")
# axis[2].plot(time, [-0.6]*len(time), label = "Ground truth", linestyle="dashed", linewidth=1)
# axis[2].set_xlabel("Time [s]")
# axis[2].set_ylabel("Distance [m]")
# axis[2].set_xlim(0,time[samples-1])
# axis[2].tick_params(axis='both', which='major', labelsize=12)
# axis[2].set_xlabel("Time [s]\n(c)")

# axis[1,0].plot(time, data_imu[:samples,6], label = "Roll (IMU)") 
# axis[1,0].plot(time, data_camera[:samples,6], label = "Roll (camera and IMU)")
# axis[1,0].plot(time, [0]*len(time), label = "Ground truth", linestyle="dashed", linewidth=1)
# axis[1,0].set_xlabel("Time [s]")
# axis[1,0].set_ylabel("Angle [rad]")
# axis[1,0].set_xlim(0,time[samples-1])
# axis[1,0].tick_params(axis='both', which='major', labelsize=12)
# axis[1,0].ticklabel_format(style='sci', axis='y', scilimits=(0,0))
# axis[1,0].set_xlabel("Time [s]\n(d)")

# axis[1,1].plot(time, data_imu[:samples,7], label = "Pitch (IMU)") 
# axis[1,1].plot(time, data_camera[:samples,7], label = "Pitch (camera and IMU)")
# axis[1,1].plot(time, [0]*len(time), label = "Ground truth", linestyle="dashed", linewidth=1)
# axis[1,1].set_xlabel("Time [s]")
# axis[1,1].set_ylabel("Angle [rad]")
# axis[1,1].set_xlim(0,time[samples-1])
# axis[1,1].tick_params(axis='both', which='major', labelsize=12)
# axis[1,1].ticklabel_format(style='sci', axis='y', scilimits=(0,0))
# axis[1,1].set_xlabel("Time [s]\n(e)")

# axis[1,2].plot(time, data_imu[:samples,8], label = "Yaw (IMU)") 
# axis[1,2].plot(time, data_camera[:samples,8], label = "Yaw (camera and IMU)")
# axis[1,2].plot(time, [0]*len(time), label = "Ground truth", linestyle="dashed", linewidth=1)
# axis[1,2].set_xlabel("Time [s]")
# axis[1,2].set_ylabel("Angle [rad]")
# axis[1,2].set_xlim(0,time[samples-1])
# axis[1,2].tick_params(axis='both', which='major', labelsize=12)
# axis[1,2].ticklabel_format(style='sci', axis='y', scilimits=(0,0))
# axis[1,2].set_xlabel("Time [s]\n(f)")



axis.legend()
axis.grid()

# for a in axis:
# for m in axis:
#     m.legend()
#     m.grid()

plt.show()
