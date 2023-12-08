from pose_tracker import Pose_tracker
from vision_algorithm import VisionAlgorithm
from tcp_client import tcp_client
from EKF import EKF
import multiprocessing, time
import numpy as np
from scipy.spatial.transform import Rotation
def EKF_func(vision_pipe, com_pipe):

    ekf = EKF()
    sensor_data_acc_gyro = np.zeros((7, 1))
    sensor_data_magnetometer = np.zeros((3, 1))
    sensor_data_barometer = 0
    vision_data = np.zeros((6, 1))
    delta_time = 0
    ekf.initialise_ekf()

    ekf_state = None

    while True:
        if com_pipe.poll():
            while com_pipe.poll():
                data = com_pipe.recv()
                data = np.reshape(data, (-1, 1))
                sensor_data_acc_gyro[:7] = data[:7]
                sensor_data_magnetometer = data[7:10]
                sensor_data_barometer = data[14]
                delta_time = data[17] - delta_time
                sensor_data_acc_gyro[-1] = delta_time
            ekf_state = ekf.measurement_step_magnetometer(sensor_data_magnetometer)
            ekf_state = ekf.measurement_step_barometer(sensor_data_barometer)
            com_pipe.send(ekf_state)
        if vision_pipe.pol():
            while vision_pipe.poll():
                data = com_pipe.recv()
                vision_data[:] = data[:]
            ekf_state = ekf.vision_step(vision_data)
            com_pipe.send(ekf_state)
        ekf.prediction_step(sensor_data_acc_gyro)

def Vision(com_pipe, ekf_pipe):
    vision = VisionAlgorithm()

    last_origin = None
    points_image = None
    Transform = None
    omega = None

    while True:
        if com_pipe.poll():
            while com_pipe.poll():
                data = com_pipe.recv()
                if data[0] == 'image':
                    last_origin = data[0]
                    points_image = data[1]
                if data[0] == 'IMU':
                    last_origin = data[0]
                    Transform = data[1]
                if data[0] == 'ang_vel':
                    last_origin = data[0]
                    omega = data[1]
            if not points_image is None and not Transform is None and not omega is None:
                T = Transform[:, 4]
                R = Transform[:, :4]
                translation, rotation_m = vision.minimize(points_image.T, R, T, omega)

                rot_vec = Rotation.from_matrix(rotation_m)
                rot_vec = rot_vec.as_rotvec()
                pose_vec = np.vstack((translation, rot_vec))
                ekf_pipe.send(pose_vec)
            else:
                continue
        else:
            continue

def Com(vision_pipe, ekf_pipe):

    #Instantiate server object instance
    client = tcp_client(host='192.168.0.101')
    pose = Pose_tracker()
    image_data = None
    while True:
        data = client.receiveData() # Receive data
        if not isinstance(data, str):
            origin = data[0]
            data = data[1]
            if origin == 'image':
                vision_pipe.send((origin, data))
                vision_pipe.send(('pose', pose.get_transform()))
                vision_pipe.send(('ang_vel', pose.get_ang_vel()))
            elif origin == 'IMU':
                pose.interpret_sensor_output(data)
                vision_pipe.send((origin, pose.get_transform()))
                ekf_pipe.send((origin, pose.get_transform()))
        if ekf_pipe.poll():
            data_ekf = None
            while ekf_pipe.poll():
                data_ekf = ekf_pipe.recv()
            client.sendData(data_ekf, 'EKF')


if __name__ == '__main__':

    vision_com_pipe, com_vision_pipe = multiprocessing.Pipe()
    vision_ekf_pipe, ekf_vision_pipe = multiprocessing.Pipe()
    ekf_com_pipe, com_ekf_pipe = multiprocessing.Pipe()

    p_EKF = multiprocessing.Process(target=EKF_func, args=(vision_ekf_pipe, ekf_com_pipe))
    p_vision = multiprocessing.Process(target=Vision, args=(com_vision_pipe, vision_ekf_pipe))
    p_com = multiprocessing.Process(target=Com, args=(com_vision_pipe, ekf_com_pipe))

    p_EKF.join()
    p_vision.join()
    p_com.join()



