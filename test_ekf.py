from Python_Serial_Com import Teensy_comm
import time, numpy as np
from EKF import EKF
from xmlhandler import XMLhandler



# The following is an 
#example of how to use it.
if __name__ == "__main__":
    np.set_printoptions(suppress=True)

    parser = XMLhandler()

    teensy = Teensy_comm(port="COM3")
    x = np.array([0,0,0,0,0,0,0,0,0]).reshape(-1,1)
    control = np.array([0,2])

    ekf = EKF()

    prev_time = 0
    prev_step = 0
    
    timer = time.time()

    ax = 0
    n = 0

    try:


        while True:

            if time.time()-timer < 5:
                control = np.array([0,2])
            elif time.time() - timer < 10:
                control = np.array([1,0])
            else:
                timer = time.time()
                

            # Receiving data from Teensy and sending a command
            temp = parser.process_xml(np.append(control,x), "PI").decode()
            data=teensy.receive_send(temp)

            if data is not None:
                sensor_data = data[1]
                current_t = sensor_data[2][-1]/1000


                vision_pos, vision_tilt = [0,0,0], [0,0,0]

                if prev_time == 0:
                    prev_time = time.time()
                    x = ekf.initialise_ekf(sensor_data).reshape(1,-1)[0]
                    dt = 0
                else:
                    dt = time.time() - prev_time
                    prev_time = time.time()

                    x = ekf.prediction_step_2_imus(sensor_data, dt).reshape(1,-1)[0]
                    x = ekf.measurement_step_barometer_2_imus(sensor_data).reshape(1,-1)[0]
                    x = ekf.measurement_step_magnetometer(sensor_data).reshape(1,-1)[0]
                    #x = ekf.measurement_step_vision(np.append(vision_pos, vision_tilt).copy()).reshape(1,-1)[0]

                acc1 = sensor_data[0][:3]
                acc2 = sensor_data[1][:3]

                debug = (acc1 + acc2 )/ 2

                ax += debug[0]
                n += 1

                print(np.round(x[:3],3), np.round(x[6:9],3), np.round(x[3:6],3), np.round(debug, 3))
                    

                if np.abs(x[0]) > 1:
                    break

                time.sleep(0.1)

        if n:
            print(ax/n)



    except KeyboardInterrupt:
        None

