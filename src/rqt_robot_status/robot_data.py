
import rospy

from roboteam_msgs import msg


class RobotType:

    TYPES = ["proto", "grsim", "arduino"]

    def __init__(self, type_str="proto"):
        self.set(type_str)

    def set(self, type_str):
        if type_str in self.TYPES:
            self.type = type_str
        else:
            self.type = "proto"

    def get(self):
        return self.type


class RobotData():

    def __init__(self, bot_id):

        self.id = bot_id

        self.is_on_vision = False

        # ---- Spatial data ----
        #
        self.pos_x = 0
        self.pos_y = 0

        self.vel_x = 0
        self.vel_y = 0

        self.angle = 0
        self.angle_w = 0

        # ---- Connection data ----

        self.acks = 0
        self.nacks = 0

        # ---- Control type ----

        self.type = RobotType()
        self.type_param_name = "robot" + str(self.id) + "/robotType"

        # ---- Current role ----

        self.role = ""


    def update_with_detection(self, detection):
        self.is_on_vision = True

        self.pos_x = detection.pos.x
        self.pos_y = detection.pos.y

        self.vel_x = detection.vel.x
        self.vel_y = detection.vel.y

        self.angle = detection.angle
        self.angle_w = detection.w


    def update_with_serial_status(self, status):
        self.acks = status.acks
        self.nacks = status.nacks


    def update_parameters_from_ros(self):

        if rospy.has_param(self.type_param_name):
            type_str = rospy.get_param(self.type_param_name)
            self.type.set(type_str)
        else:
            rospy.set_param(self.type_param_name, self.type.get())


    def update_with_role_status(self, status):
        if status.type == msg.BtDebugInfo.TYPE_ROLE:
            self.role = status.name


    def change_robot_type(self, new_type):
        self.type = new_type

        type_param_name = "robot" + str(self.id) + "/robotType"
        rospy.set_param(self.type_param_name, self.type.get())

    def clear_role_status(self):
        self.role = ""

    def set_vision_lost(self):
        self.is_on_vision = False

    def get_id(self):
        return self.id

    def get_vision_status(self):
        return self.is_on_vision

    def get_pos(self):
        return (self.pos_x, self.pos_y)

    def get_vel(self):
        return (self.vel_x, self.vel_y)

    def get_angle(self):
        return self.angle

    def get_w(self):
        return self.angle_w

    def get_connection_status(self):
        return (self.acks, self.nacks)

    def get_type(self):
        return self.type

    def get_role(self):
        return self.role
