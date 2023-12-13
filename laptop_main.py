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

    # Initialise containers for the inputs from sensors
    sensor_data_acc_gyro = np.zeros((7, 1))
    sensor_data_magnetometer = np.zeros((3, 1))
    sensor_data_barometer = 0

    # Initialise container for vision algorithm data
    vision_data = np.zeros((6, 1))

    # Initialise time variable
    delta_time = 0
    prev_time = 0

    # Initialise EKF
    ekf.initialise_ekf()

    ekf_state = None

    # Infinite while loop for continuous process execution
    while True:
        # If the pipeline to communication contains readable data
        if com_pipe.poll():
            # While loop ensuring that the most recent data is used for the updates
            while com_pipe.poll():
                # Retrieve the data from the pipeline and reshape it to a column vector
                data = com_pipe.recv()

            if data is not None:
                data = data[1]

                    

                sensor_data_acc_gyro1 = data[0][:]
                sensor_data_acc_gyro2 = data[1][:]
                sensor_data_acc_gyro = np.append(sensor_data_acc_gyro1, sensor_data_acc_gyro2).copy().reshape(-1,1)
                sensor_data_magnetometer = data[2][:3]
                sensor_data_barometer1 = data[2][3]
                sensor_data_barometer2 = data[2][4]
                sensor_data_barometer = np.append(sensor_data_barometer1, sensor_data_barometer2).copy().reshape(-1,1)
                current_t = (data[2][-1]/1000)

                if prev_time == 0:
                    prev_time = current_t

                delta_time = current_t - prev_time
                prev_time = current_t
        
                # Perform the prediction update with the available data
                ekf_state = ekf.prediction_step_2_imus(sensor_data_acc_gyro, delta_time)

                # Perform perception updates with magnetometer and barometer data
                #ekf_state = ekf.measurement_step_magnetometer(sensor_data_magnetometer)
                ekf_state = ekf.measurement_step_barometer_2_imus(sensor_data_barometer)


        # If the pipeline to the vision algorithm contains readable data
        if vision_pipe.poll():
            # While loop ensuring that the most recent data is used for the update
            while vision_pipe.poll():
                # Retrieve the data from the pipeline and assign it to the variable
                data = vision_pipe.recv()

            # Perform the perception update with the new vision data
            #ekf_state = ekf.measurement_step_vision(vision_data)


        if ekf_state is not None:
            # Send the result to the control system through the communication process
            print(np.round(ekf_state,3).reshape(1,-1)[0])

            com_pipe.send(ekf_state)

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
    points_image = None # Image marker coordinates [x, y] in [mm]
    Transform = None # Transform containing pose of camera in world coordinates
    omega = None # Angular velocities

    # Infinite while loop for continuous process execution
    while True:
        # If the pipeline to communication contains readable data
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

                # If the data contains angular velocities
                if data[0] == 'ang_vel':
                    omega = data[1]

            # If neither of the variables are None, execute the algorithm with the new data
            if not points_image is None and not Transform is None and not omega is None:

                # Define T and R components
                T = Transform[:, 3]
                R = Transform[:4, :3].copy()

                # Execute the minimisation of the variables
                translation, rotation_m = vision.minimize(points_image.T, R, T, omega)

                # Convert the results into usable data for the EKF
                rot_vec = Rotation.from_matrix(rotation_m) # From rotation matrix
                rot_vec = rot_vec.as_rotvec() # To rotation vector (Assumed to be rpy)
                pose_vec = np.vstack((translation, rot_vec)) # Create a column vector from the results
                # Send the results to the EKF
                ekf_pipe.send(pose_vec)
            else:
                continue
        else:
            continue

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

    # Instantiate the GUI and Pose tracker objects
    GUI = Drone_GUI()
    pose = Pose_tracker()

    # Initialise GUI variables
    execution_state = 0
    desired_altitude = 0

    # Update the GUI and retrieve relevant data and events and print the "LED"
    events, values = GUI.getinput()
    GUI.SetLED('online', 'red')

    # Infinite while loop for continuous process execution
    while True:
        # Update the GUI and retrieve relevant data and events
        events, values = GUI.getinput()

        # If the 'Abort' button is pressed stop the drone system
        # and update relevant GUI components
        if events == 'Abort':
            execution_state = 0
            GUI.SetLED('online', 'red')

        # Else if the 'Execute' button is pressed start the drone system
        # and update relevant GUI components
        elif events == 'Execute':
            execution_state = 1
            GUI.SetLED('online', 'green')

        # Else if the user has entered a desired altitude and pressed the 'Submit' button, update the
        # value sent to the control system
        elif events == 'Submit':
            desired_altitude = float(values[0])


        # Retrieve data from the server
        data = client.receiveData()

        # If no error occurred
        if not isinstance(data, str):

            # print(data[0])

            # Seperate the origin and data
            origin = data[0]
            data = data[1]

            # If the origin of the data is from image processing
            # send the vision algorithm the data along with the
            # most recent pose and angular velocity measured.
            if origin == 'image':
                vision_pipe.send((origin, data))
                vision_pipe.send(('pose', pose.get_transform()))
                vision_pipe.send(('ang_vel', pose.get_ang_vel()))

            # Else if the origin of the data is from the IMU
            # update the pose tracker with the values from the gyro and time
            # and send the data along to the EKF and vision process
            elif origin == 'IMU':
                pose.update_values_data(data)
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

                # Reshape the data to include the execution state and the desired altitude
                data_ekf = np.append([execution_state, desired_altitude], data_ekf)

                # # Update the GUI values
                # GUI.update_text('data', f"x_pos:\t{data_ekf[0]:.6f}\t\tx_vel:\t{data_ekf[3]:.6f}\t\troll:\t{data_ekf[6]:.6f}\n"
                #                         f"y_pos:\t{data_ekf[1]:.6f}\t\ty_vel:\t{data_ekf[4]:.6f}\t\tpitch:\t{data_ekf[7]:.6f}\n"
                #                         f"z_pos:\t{data_ekf[2]:.6f}\t\tz_vel:\t{data_ekf[5]:.6f}\t\tyaw:\t{data_ekf[8]:.6f}")

                # Update the pose tracker
                pose.update_values_EKF(data_ekf)

            # Send the final data to the control system
            #print(f"Execution state: {execution_state}\n Desired altitude: {desired_altitude}")
            status = client.sendData(data_ekf, 'EKF')


if __name__ == '__main__':

    # Initialise pipelines between processes for inter-process communication
    # Named as from_to_pipe (Wierd naming convention i know)
    vision_com_pipe, com_vision_pipe = multiprocessing.Pipe()
    vision_ekf_pipe, ekf_vision_pipe = multiprocessing.Pipe()
    ekf_com_pipe, com_ekf_pipe       = multiprocessing.Pipe()

    # Process initialisation, targets are the above functions handling each individual process and the args are
    # the arguments for the function
    p_EKF = multiprocessing.Process(target=EKF_func,    args=(ekf_vision_pipe, ekf_com_pipe)) # EKF process
    p_vision = multiprocessing.Process(target=Vision,   args=(vision_com_pipe, vision_ekf_pipe)) # Vision algorithm process
    p_com = multiprocessing.Process(target=Com,         args=(com_vision_pipe, com_ekf_pipe)) # Communication process
    
    p_EKF.start()
    p_vision.start()
    p_com.start()
    
    # For synchronisation purposes these have to be called
    # If the processes finish simultaneously, try commenting them out.
    p_EKF.join()
    p_vision.join()
    p_com.join()



