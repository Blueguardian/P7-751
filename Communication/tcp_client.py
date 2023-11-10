# Designed by group 7-751, Aalborg University, 2023.
# For purpose of project, concerning tethered drone control.
# All rights reserved, can be copied under license.

import socket, math
import numpy as np
from xmlhandler import XMLhandler
from time import sleep

class tcp_client_test:
    """
    TCP client class for handling serverside message retrieval and sending of xml and message data, along with parsing
    of received xml data.
    Dependencies:
        socket and xmlhandler(included)
    Compatability:
        Unknown, possibly limited to Linux-type systems, furher testing required
    Further development:
        Create own protocol for sending and receiving
        More robust receive-methods of data and more data types
        Clean-up and optimization
    """

    # Fixed class attributes
    __PARSER = XMLhandler() #
    __serverSocket = None

    def __init__(self, host='127.0.0.1', port=15567, receive_size=65536, format='utf-8', sock_rep=socket.AF_INET,
                 sock_type=socket.SOCK_STREAM, rec_len=10):
        """
        Initialiser for TCP-client class, sets values specified for object attibutes and initialised the connection
        between the client and the server. Optional inputs can be used for changing default values.
        :param host: Host IP-address (String): Default value: '127.0.0.1' - Loopback
        :param port: Communication port (Int): Default value: 15567 - port above 1024
        :param receive_size: Size of data retrieval buffer (Int): Default value: 65536 -  64kB largest allowed for TCP
        :param format: Encoding format (String): Default value: 'utf-8': Default used in .encoding()
        :param sock_rep: Socket address family (Module socket address family type): Default value: socket.AF_INET
        :param sock_type: Socket stream type (Module socket stream type): Default value: socket.SOCK_STREAM
        :param rec_len: Length of header for incoming data
        """

        # Assign attributes parameter values
        self.__HOST = host
        self.__PORT = port
        self.__SIZE = receive_size # Not currently used
        self.__FORMAT = format
        self.__LEN_SIZE = rec_len  # Has to agree on both sides
        self.__serverSocket = socket.socket(sock_rep, sock_type)

        # Initialise socket
        self.__serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__serverSocket.connect((self.__HOST, self.__PORT))
        print(f"Connected to server at: addr: {self.__HOST}:{self.__PORT}")

    def __del__(self):
        """
        Destructor for instances of class, to ensure that sockets are closed before program exit.
        :return: None
        """
        self.__serverSocket.close()

    def __receive_length(cls):
        """
        Object method __receive_length used to obtain the length of the incoming data
        :return: int: length of incoming message or data
        """
        data = cls.__serverSocket.recv(cls.__LEN_SIZE)
        data = data.decode(cls.__FORMAT)
        length = eval(data.split('-')[1].lstrip("0"))
        return length

    def receiveMsg(self):
        """
        Object method receiveMsg used for receiving messages that does not need xml-parsing
        :return: string: message
        """
        try:
            data = self.__serverSocket.recv(self.__SIZE)
        except socket.error:
            return "SOCKET_TIMEOUT error"
        else:
            if not data:
                return "Invalid data"
            else:
                data = data.decode(self.__FORMAT)
                return data


    def receiveData(self):
        """
        Instance method, used for receiving data from the socket stream, utilises internal methods and embedded class for
        parsing information. Returns received data in a numpy array format.
        :return: ndarray: based on the type and size defined in the received XML
        """
        try:
            length = self.__receive_length()
            data = self.__serverSocket.recv(length)
        except socket.error:
            return "SOCKET_TIMEOUT error"
        else:
            if str(data.decode(self.__FORMAT)) == "" or str(data.decode(self.__FORMAT)) == '':
                return "DATA_INVALID error"
            else:
                temp = data.decode(self.__FORMAT)
                origin, in_data = self.__PARSER.parse_xml(temp)
                return origin, in_data

    def sendData(self, data, origin="None"):
        """
        Instance method, used for sending xml formatted data to the socket stream, utilises internal methods and embedded
        class for processing the data. Returns error if socket is closed or the connection timed out.
        :param data: Data to be sent to the socket stream. Currently available types for processing (Lists and tuples,
        ints, floats, complex, string and bool)
        :return: Error if socket timed out and None if successful
        """
        try:
            out_data = self.__PARSER.process_xml(data, origin)
            length = b'-' + f"{len(out_data)}".zfill(8).encode(self.__FORMAT) + b'-'
            out_data = length + out_data
            if not isinstance(out_data, bytes):
                out_data = out_data.encode(self.__FORMAT)
            checkout = self.__serverSocket.send(out_data)
        except socket.error:
            return "SOCKET_TIMEOUT error"
        else:
            return checkout


    def sendMsg(self, msg):
        """
        Instance method, used for sending messages to the client, formatted as strings. Returns error if socket timed out
        :param msg: The message to send, assumed to be a string
        :return: Error message if socket timed out, None if successful
        """
        try:
            msg = msg.encode(self.__FORMAT)
            self.__serverSocket.send(msg)
        except socket.error:
            return "SOCKET_TIMEOUT error"
        else:
            return None

"""Test setup for the classes"""
"""Receives 'ack' before continuing to ensure a fully established connection"""
"""Sends different types of data to the server for processing"""

# Instantiate client object
client = tcp_client_test(host='192.168.0.101')

# Still not tested for other types than ndarray

# "Main function" only used due to class definition above
if __name__ == "__main__":
    ack = None # Initialise ack variable
    while ack != 'ack':
        client.sendMsg('ack') # Reassign until ack == 'ack'
        ack = client.receiveMsg() # Send 'ack' to server

    # Define different data type for testing the setup
    data_fun1 = np.zeros((5, 5))
    data_fun2 = np.zeros((5, 5, 2))
    data_fun3 = np.zeros((3, 1))
    data_fun4 = np.zeros((1))
    data_fun5 = np.zeros((4, 4))
    data_fun5[0, 0] = math.cos(5)
    data_fun5[0, 1] = -math.sin(5)
    data_fun5[0, 3] = 411.0
    data_fun5[1, ] = math.sin(5)*math.cos(50)
    data_fun5[1, 1] = math.cos(5)*math.cos(50)
    data_fun5[1, 2] = -math.sin(50)
    data_fun5[1, 3] = math.cos(50)*112.0
    data_fun5[2, 0] = math.sin(5)*math.sin(50)
    data_fun5[2, 1] = math.cos(5)*math.sin(50)
    data_fun5[2, 2] = math.cos(50)
    data_fun5[2, 3] = math.cos(50)*112.0
    data_fun5[3, 3] = 1.0
    i = 0 # Iterator
    sleep(2) # Wait 2 seconds before sending all the data
    while True:
        if i < 1:
            check = client.sendData(data_fun1) # Send 2D Matrix
            print(check)
        elif i == 1:
            check = client.sendData(data_fun2) # Send 3D Matrix
            print(check)
        elif i == 2:
            check = client.sendData(data_fun3) # Send Vector
            print(check)
        elif i == 3:
            check = client.sendData(data_fun4) # Send Value
            print(check)
        elif i > 3 and i < 5:
            check = client.sendData(data_fun5) # Send transformation matrix
            print(check)
        i += 1


