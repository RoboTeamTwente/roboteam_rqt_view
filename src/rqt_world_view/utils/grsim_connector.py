from python_qt_binding import QtNetwork

from roboteam_utils.grSim_Packet_pb2 import grSim_Packet
from roboteam_utils.grSim_Replacement_pb2 import grSim_Replacement


class GrsimConnector:

    def __init__(self):

        self.socket = QtNetwork.QUdpSocket()

        self.default_ip = "127.0.0.1"
        self.default_port = 20011


    def place_robot(self):
        packet = grSim_Packet
        replace = grSim_Replacement

        #grSim_Packet.setReplacement(replace)
