# Run this script on the laptop!

# Standard library imports
import multiprocessing, time
import numpy as np
from scipy.spatial.transform import Rotation # Used for switching between rotation representations

# System class imports
from Interfaces.GUI import Drone_GUI
from Interfaces.pose_tracker import Pose_tracker
from Algorithms.vision_algorithm import VisionAlgorithm
from Algorithms.EKF import EKF
from Communication.tcp_client import tcp_client


import matplotlib.pyplot as plt
import csv



np.set_printoptions(suppress=True)

def EKF_func(vision_pipe, com_pipe):
    """
    Extended Kalman filter process, instantiates and executes the EKF class, for sensor fusion with the remainder
    of the system. It receives inputs from both the sensors and the vision algorithm. The sensor output is received
    from the Raspberry PI through a TCP socket connection and sent through a inter-process pipeline to the EKF process
    from the communication process. Similarly, the vision output is received through a inter-process pipeline to the EKF
    process from the process related to the vision algorithm
    :param vision_pipe: The pipeline connected to the vision algorithm process
    :param com_pipe: The pipeline connected to the communication process
    """

    # Instantiate the Extended Kalman filter object
    ekf = EKF()
    ekf_state = None
    
    # Initialise time variable
    prev_time = 0


    ekf1 = EKF()
    ekf_state1 = None
    readings = np.empty((0,10))
    readings1 = np.empty((0,10))
    n_readings = 0
    just_do_it = False

    plt.ion()

    try:
        # Infinite while loop for continuous process execution
        while True:
            # If the pipeline to communication contains readable data
            if com_pipe.poll():

                # While loop ensuring that the most recent data is used for the updates
                while com_pipe.poll():
                    # Retrieve the data from the pipeline and reshape it to a column vector
                    data = com_pipe.recv()

                    if data is not None:
                        sensor_data = data[1]
                        current_t = (sensor_data[2][-1]/1000)

                        if prev_time == 0: # Initialise EKF
                            
                            ekf_state = ekf.initialise_ekf(sensor_data)
                            ekf_state1 = ekf1.initialise_ekf(sensor_data)
                            prev_time = current_t
                            
                        else:
                            dt = current_t - prev_time
                            prev_time = current_t
                    
                            # Perform the prediction update with the available data
                            ekf.prediction_step_2_imus(sensor_data, dt)
                            ekf.measurement_step_barometer_2_imus(sensor_data)
                            ekf_state = ekf.measurement_step_magnetometer(sensor_data)

                            ekf1.prediction_step_2_imus(sensor_data, dt)
                            ekf1.measurement_step_barometer_2_imus(sensor_data)
                            ekf_state1 = ekf1.measurement_step_magnetometer(sensor_data)
                        
                        just_do_it = True

                        

            # If the pipeline to the vision algorithm contains readable data
            if vision_pipe.poll() and prev_time:
                
                # While loop ensuring that the most recent data is used for the update
                while vision_pipe.poll():
                    # Retrieve the data from the pipeline and assign it to the variable
                    data = vision_pipe.recv()

                # print(data)

                # Perform the perception update with the new vision data
                ekf_state = ekf.measurement_step_vision(data)
                

            if ekf_state is not None:
                # Send the result to the control system through the communication process
                
                if just_do_it:
                    # print(ekf_state.reshape(1,-1)[0], current_t)            

                    n_readings +=1
                    readings = np.vstack([readings, np.append(ekf_state.reshape(1,-1)[0], current_t)])
                    readings1 = np.vstack([readings1, np.append(ekf_state1.reshape(1,-1)[0], current_t)])

                    plt.plot(readings[:,0], readings[:,1])
                    plt.grid()

                    plt.draw()
                    plt.pause(0.001)
                    plt.clf()


                    if n_readings % 50 == 0:
                            print("\n",n_readings, "\n")

                    if n_readings == 1000:
                        None
                        np.savetxt("imu_and_camera6.csv", readings, delimiter=",")
                        np.savetxt("only_imu6.csv", readings1, delimiter=",")

                    just_do_it = False

                    com_pipe.send(ekf_state)

    except KeyboardInterrupt:
        pass

def Vision(com_pipe, ekf_pipe):
    """
    Vision algorithm process, instantiates and executes the VisionAlgorithm class, for pose estimation using the Raspberry
    Pi Camera. It receives inputs from the Kalman filter after updates and from the image processing.
    The data from the Kalman filter output is used for initial position and orientation estimates and is continuously updated
    when the Kalman filter has updated it's states, through a pose tracker object. The Image data received from the image
    processing is postions of the markers and is retrieved through the inter-process pipeline from the communication
    process.
    The EKF-data is stored by the communication process and is retrieved from the inter-process pipeline from there. The
    data output from this process is sent to the EKF process for updates.
    :param com_pipe: The pipeline connected to the communication process
    :param ekf_pipe: The pipeline connected to the EKF process
    """
    # Instantiate the vision algorithm object
    vision = VisionAlgorithm()

    # Initialise containers for the data used by the algorithm

    # If the pipeline to communication contains readable data
    points_image = None # Image marker coordinates [x, y] in [mm]
    Transform = np.array([None, 0, 0, 0]) # Transform containing pose of camera in world coordinates

    try:
        # Infinite while loop for continuous process execution
        while True:

            if com_pipe.poll():
                # While loop ensuring that the most recent data is used for the algorithm
                while com_pipe.poll():
                    # Retrieve the data from the pipeline and reshape it to a column vector
                    data = com_pipe.recv()

                    # If the data is from the image processing
                    if data[0] == 'image':
                        points_image = data[1]

                    # If the data contains the pose from the EKF
                    if data[0] == 'pose':
                        Transform = data[1]

                # If neither of the variables are None, execute the algorithm with the new data
                if not points_image is None and Transform[0] is not None:
                    # Define T and R components
                    T = Transform[:, 3]
                    R = Transform[:4, :3].copy()

                    # Execute the minimisation of the variables
                    translation, rotation_m = vision.minimize(points_image.T, R, T)

                    # Convert the results into usable data for the EKF
                    rot_vec = Rotation.from_matrix(rotation_m).as_euler("zyx") # From rotation matrix
                    pose_vec = np.append(translation, rot_vec) # Create a column vector from the results
                    # Send the results to the EKF

                    # print(pose_vec)

                    ekf_pipe.send(pose_vec)

                    # If the pipeline to communication contains readable data
                    points_image = None # Image marker coordinates [x, y] in [mm]
                    Transform = np.array([None, 0, 0, 0]) # Transform containing pose of camera in world coordinates

                else:
                    continue
            else:
                continue
    except KeyboardInterrupt:
        pass

def Com(vision_pipe, ekf_pipe):
    """
    Communication process, initialises a TCP socket connection to the Raspberry Pi, to receive data from the sensors and
    the image processing. Furthermore, it keeps track of the last output of the EKF in a pose tracker class for the
    vision algorithm. The process opens a graphical user interface, for the user to start and stop the execution of the
    drone system. It uses the respective pipelines to send and receive data from the vision and EKF processes.
    :param vision_pipe: The pipeline connected to the vision algorithm process
    :param ekf_pipe: The pipeline connected to the EKF process
    """

    #Instantiate client object instance
    client = tcp_client(host='192.168.0.102') # Standard host IP, only change if you know what you are doing
    pose = Pose_tracker()

    try:
        while True:

            # Retrieve data from the server
            data = client.receiveData()

            # If no error occurred
            if not isinstance(data, str):

                # Seperate the origin and data
                origin = data[0]
                data = data[1]

                # If the origin of the data is from image processing
                # send the vision algorithm the data along with the
                # most recent pose and angular velocity measured.
                if origin == 'image':
                   # print("points:", data)
                    vision_pipe.send((origin, data))
                    vision_pipe.send(('pose', pose.get_transform()))

                # Else if the origin of the data is from the IMU
                # update the pose tracker with the values from the gyro and time
                # and send the data along to the EKF and vision process
                elif origin == 'IMU':
                # pose.update_values_data(data)
                    ekf_pipe.send((origin, data))
                    vision_pipe.send((origin, pose.get_transform()))

            # If the pipeline to the EKF process contains readable data
            if ekf_pipe.poll():
                
                # Initialise a container for the data
                data_ekf = None

                # While loop ensuring that the most recent data is sent
                while ekf_pipe.poll():
                    # Retrieve the data
                    data_ekf = ekf_pipe.recv()
                    
                    temp = np.array([0, 0, 0.7, 0, 0, 0, 0, 0, 0]).reshape(-1,1)
                    pose.update_values_EKF(temp)

                    # pose.update_values_EKF(data_ekf)
                    data_ekf = np.append([1, 1], data_ekf)

                    # pose.update_values_EKF(data_ekf)
                    

                # Send the final data to the control system
                status = client.sendData(data_ekf, 'EKF')

    except KeyboardInterrupt:
        pass


if __name__ == '__main__':

    # Initialise pipelines between processes for inter-process communication
    # Named as from_to_pipe (Wierd naming convention i know)
    vision_com_pipe, com_vision_pipe = multiprocessing.Pipe()
    vision_ekf_pipe, ekf_vision_pipe = multiprocessing.Pipe()
    ekf_com_pipe, com_ekf_pipe = multiprocessing.Pipe()

    # Process initialisation, targets are the above functions handling each individual process and the args are
    # the arguments for the function
    p_EKF = multiprocessing.Process(target=EKF_func,    args=(ekf_vision_pipe, ekf_com_pipe)) # EKF process
    p_vision = multiprocessing.Process(target=Vision,   args=(vision_com_pipe, vision_ekf_pipe)) # Vision algorithm process
    p_com = multiprocessing.Process(target=Com,         args=(com_vision_pipe, com_ekf_pipe)) # Communication process
    
    try:

        p_EKF.start()
        p_vision.start()
        p_com.start()
        
        # For synchronisation purposes these have to be called
        # If the processes finish simultaneously, try commenting them out.
        p_EKF.join()
        p_vision.join()
        p_com.join()
    except KeyboardInterrupt:
        pass


