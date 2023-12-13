import numpy as np
import matplotlib.pyplot as plt
 
np.printoptions(suppress=True)

data_imu = np.loadtxt("only_imu2.csv", delimiter=",", dtype=np.float64)
data_imu[:,-1] -= data_imu[0,-1]

data_camera = np.loadtxt("imu_and_camera2.csv", delimiter=",", dtype=np.float64)
data_camera[:,-1] -= data_camera[0,-1]


samples = 150
x = data_camera[:samples,-1]





# Initialise the subplot function using number of rows and columns 
figure, axis = plt.subplots(2, 1) 

axis[0].plot(x, data_imu[:samples,0], label = "x_imu") 
axis[0].plot(x, data_camera[:samples,0], label = "x_vision")
axis[0].set_title("X estimation")

axis[1].plot(x, data_imu[:samples,1], label = "y_imu") 
axis[1].plot(x, data_camera[:samples,1], label = "y_vision") 
axis[1].set_title("Y estimation")


# Combine all the operations and display 
axis[0].legend()
axis[1].legend()
axis[0].grid()
axis[1].grid()
plt.show()
