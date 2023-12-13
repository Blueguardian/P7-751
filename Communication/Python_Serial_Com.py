from xmlhandler import XMLhandler
import serial, serial.tools.list_ports, numpy as np
import time


class Teensy_comm():
    """
    Class for reading receiving the readings from the sensors and sending a command to the Teensy.

    It will always check if there is something to read without blocking the loop.
    Once it receives new data, it sends a command to the Teensy and switches back to checking for new data.
    """
    def __init__(self, port: str = None):
        self.data = None
        self.ser = None

        self.parser = XMLhandler()

        if port:
            self.init_serial_com(port, 115200)
        else:
            self.check_ports()

    @classmethod
    def check_ports(cls):
        ports = serial.tools.list_ports.comports()

        for port, desc, hwid in sorted(ports):
            print("{}: {} [{}]".format(port, desc, hwid))

        if(not ports):
            print("[!]: Couldn't find any serial port open.")

    def init_serial_com(self, port: str, baud: int):
        self.ser = serial.Serial(port=port,  baudrate=baud)
        self.ser.flush()

    def receive_send(self, command=""):
        """
        This function will always return the last data obtained through serial com.
        The way it works is:
            - Detects if there is some data to read in the bus.
            - Process this data and updates self.data with last one obtained.
            - Sends a new command to the Serial bus.

            return self.data
        """

        if(self.ser.in_waiting):

            line=self.ser.readline().decode()[:-2]      # Reading line, decoding and removing EOL characters(\r and \n).

            self.data = self.parser.parse_xml(line)[1]      # Parse line from XML to sensor data.

            #self.data = XMLhandler.parse_xml(line)[1]

            self.ser.write(bytes(command,  'utf-8'))    # Send command.


        return self.data



# The following is an example of how to use it.
if __name__ == "__main__":


    np.set_printoptions(suppress=True)

    teensy = Teensy_comm(port="COM3")

    start = time.time()
    mode_timer = time.time()

    try:
        while True:
                
            # Example of alternating command (led transitions between blinking and of each 5 seconds.
            if(time.time()-mode_timer < 5):
                command = "command: blink_enabled"
            elif(time.time()-mode_timer < 10):
                command = "command: blink_disabled"
            else:
                mode_timer = time.time()

            # Receiving data from Teensy and sending a command
            data=teensy.receive_send(command)

            if data is not None:
                print(data)


            print("time:", time.time()-start, "s\n")
            start = time.time()

    except KeyboardInterrupt:
        None

