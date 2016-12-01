import rospy

from python_qt_binding import QtNetwork

from roboteam_utils.grSim_Packet_pb2 import grSim_Packet
from roboteam_utils.grSim_Replacement_pb2 import grSim_RobotReplacement

import utils

class GrsimConnector:

    def __init__(self):
        self.socket = QtNetwork.QUdpSocket()

        self.default_ip = QtNetwork.QHostAddress("127.0.0.1")
        self.default_port = 20011


    def place_robot(self, id, is_us, x, y):
        """
        Sends a message to Grsim to place a robot at a specific position.
        Both x and y are in mm.
        """

        our_color = rospy.get_param('our_color', 'yellow')

        if our_color == "yellow":
            yellow_team = is_us
        else:
            yellow_team = not is_us

        packet = grSim_Packet()

        packet.replacement.robots.add()

        packet.replacement.robots[0].x = utils.mm_to_m(x)
        packet.replacement.robots[0].y = utils.mm_to_m(y)
        packet.replacement.robots[0].dir = 0
        packet.replacement.robots[0].id = id
        packet.replacement.robots[0].yellowteam = yellow_team

        self.socket.writeDatagram(packet.SerializeToString(), self.default_ip, self.default_port)


    def place_ball(self, x, y):
        """
        Sends a message to Grsim to place the ball at a specific position.
        Both x and y are in mm.
        """

        packet = grSim_Packet()

        packet.replacement.ball.x = utils.mm_to_m(x)
        packet.replacement.ball.y = utils.mm_to_m(y)
        packet.replacement.ball.vx = 0
        packet.replacement.ball.vy = 0

        self.socket.writeDatagram(packet.SerializeToString(), self.default_ip, self.default_port)
