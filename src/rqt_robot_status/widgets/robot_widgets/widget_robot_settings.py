
import os

from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from python_qt_binding import QtGui

from rqt_robot_status import robot_data


class WidgetRobotSettings(QtWidgets.QFrame):

    def __init__(self, bot):
        super(WidgetRobotSettings, self).__init__()

        self.bot_id = bot.get_id()
        self.bot_data = bot

        self.setLayout(QtWidgets.QGridLayout())
        self.setFrameStyle(QtWidgets.QFrame.Panel | QtWidgets.QFrame.Sunken)

        self.keyboard_button = QtWidgets.QPushButton("Controller")
        self.keyboard_icon = QtGui.QIcon.fromTheme("input-keyboard")
        self.keyboard_button.setIcon(self.keyboard_icon)
        self.layout().addWidget(self.keyboard_button, 0, 0)

        self.robot_type_box = QtWidgets.QComboBox()
        self.layout().addWidget(self.robot_type_box, 1, 0)

        for item in robot_data.RobotType.TYPES:
            self.robot_type_box.addItem(item)

        # ---- Connect signals ----

        self.keyboard_button.clicked.connect(self.open_keyboard_controller)
        self.robot_type_box.activated.connect(self.change_robot_type)

        # ---- /Connect signals ----


    def update(self):
        type_index = self.robot_type_box.findText(self.bot_data.type.get())
        self.robot_type_box.setCurrentIndex(type_index)


    def open_keyboard_controller(self):
        """
        Try to launch keyboard controller.
        Using `&` to launch it independedly from this program.
        """
        os.system("rosrun roboteam_input keyboard_controller -id " + str(self.bot_id) + " &")

    def change_robot_type(self, index):
        type_str = self.robot_type_box.itemText(index)
        new_type = robot_data.RobotType(type_str)
        self.bot_data.change_robot_type(new_type)
