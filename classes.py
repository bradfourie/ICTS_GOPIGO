from enum import Enum
from socket import *
from constants import *

class ControlParadigm(Enum):
    """
    Enum that stores the different control paradigm options
    """
    MODE_FREE_ROAMING = 0
    MODE_FOLLOW_LEFT_WALL = 1
    MODE_FOLLOW_RIGHT_WALL = 2
    MODE_USER_CONTROL = 3

class GUICommands(Enum):
    """
    Enum that stores the different commands sent from the GUI
    """
    CP_FREE_ROAMING = "FREE_ROAMING"
    CP_FOLLOW_LEFT_WALL = "LEFT_WALL"
    CP_FOLLOW_RIGHT_WALL = "RIGHT_WALL"
    CP_USER_CONTROL = "USER_CONTROL"
    BP_FORWARD = "W"
    BP_BACK = "S"
    BP_LEFT = "A"
    BP_RIGHT = "D"
    BP_STOP = "STOP"
    BP_START = "START"

class Server():
    """
    Server class for the Client-Server communication between the GUI and the GoPiGo3 robot

    Attributes:
        server_port:    stores the port to use on the server
        server_socket:  stores the socket object for the server
        client_address: stores the address of the client so that the battery voltage can be sent to the GUI
    """
    def __init__(self):
        self.server_port = SERVER_PORT
        self.server_socket = None

        self.client_address = None

    def connect(self):
        """
        connect executes code that waits for the client to connect to the server and then resends the message that
        was received from the client back to the client as confirmation of connection
        :return: None
        """
        print(" Connecting to the client...")
        self.server_socket = socket(AF_INET, SOCK_DGRAM)
        self.server_socket.bind(('', self.server_port))

        while True:
            message, self.client_address = self.server_socket.recvfrom(2048)
            self.server_socket.sendto(message.encode(),self.client_address)
            break

        print(" Finished...")


    def close_connection(self):
        """
        close_connection closes the connection to the server
        :return: None
        """
        self.server_socket.close()

    def receive_message(self):
        """
        receive_message receives the message from the client and decodes it. This decoded message is then returned by
        the method
        :return: modified_message
        """
        message, client_address = self.server_socket.recvfrom(2048)
        modified_message = message.decode()

        return modified_message

    def send_message(self, message):
        """
        send_message sends a message to the client
        :param message: The input message to be sent to the client
        :return: None
        """
        self.server_socket.sendto(message.encode(), self.client_address)
