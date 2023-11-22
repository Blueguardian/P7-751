import numpy as np
import serial, time 
from xmlhandler import XMLhandler

np.set_printoptions(suppress=True)

handler = XMLhandler()
start = time.time()
mode_timer = time.time()
command = "command: blink_enabled"

with serial.Serial(port="COM3",  baudrate=115200) as arduino:
    arduino.flushInput()

    try:
        while True:
            if(arduino.in_waiting):
                line=arduino.readline().decode()[:-2] # we remove EOL characters(\r and \n)
                
                data = handler.parse_xml(line)[1]
                print(np.round(data,2), "\n")


                if(time.time()-mode_timer < 5):
                    command = "command: blink_enabled"
                elif(time.time()-mode_timer < 10):
                    command = "command: blink_disabled"
                else:
                    mode_timer = time.time()

                arduino.write(bytes(command,  'utf-8'))


                print("time:", time.time()-start)
                start = time.time()

    except KeyboardInterrupt:
        None

    