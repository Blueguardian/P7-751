import serial
from xmlhandler import XMLhandler

class TeensyCom:

    __serial = None
    __PARSER = XMLhandler()

    def __init__(self, port='/dev/ttyS0', baudrate=9600, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS,
                 parity=serial.PARITY_NONE, timeout=0, length_param=8, format='utf-8'):
        self.__PORT = port
        self.__BAUD = baudrate
        self.__STOPBITS = stopbits
        self.__BYTESIZE = bytesize
        self.__PARITY = parity
        self.__TIMEOUT = timeout
        self.__SIZE_LEN = length_param
        self.__FORMAT = format

        self.__serial = serial.Serial(self.__PORT,
                                      self.__BAUD,
                                      self.__BYTESIZE,
                                      self.__PARITY,
                                      self.__STOPBITS,
                                      self.__TIMEOUT)

        self.__serial.reset_input_buffer()
        self.__serial.reset_output_buffer()

    def __del__(self):
        self.__serial.close()
        self.__serial.__del__()

    def __receiveLength(self):
        try:
            if not self.__serial.isOpen():
                self.__serial.open()
            if not self.__serial.writable():
                self.__serial.flushOutput()
            length = str(self.__serial.read_until(b'\r\n')).lstrip("0")
            length = eval(length)
        except serial.PortNotOpenError:
            return "PORT_NOT_OPEN ErrorCode: 1"
        else:
            return length

    def __packLength(self, data):
        length = str(len(data).zfill(self.__SIZE_LEN)).encode(self.__FORMAT)
        length = length + b'\n'
        return length

    def __packData(self, data):
        out_data = self.__PARSER.process_xml().encode(self.__FORMAT)
        length = self.__packLength(out_data)
        return [length, out_data]


    def sendData(self, data):
        try:
            if not self.__serial.isOpen():
                self.__serial.open()
            if not self.__serial.writable():
                self.__serial.flushOutput()
            self.__serial.writelines(self.__packData(data))
            self.__serial.close()
        except serial.PortNotOpenError:
            return "PORT_NOT_OPEN ErrorCode: 1"
        else:
            return None

    def receiveData(self):
        try:
            if not self.__serial.isOpen():
                self.__serial.open()
            length = self.__receiveLength().lstrip("0").decode(self.__FORMAT)
            in_data = self.__serial.read(size=eval(length))
            in_data = self.__PARSER.parse_xml(in_data)
        except Exception as e:
            return f"Exception {e} occurred"
        else:
            return in_data