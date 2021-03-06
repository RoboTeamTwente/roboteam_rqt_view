
from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from python_qt_binding import QtGui

import colors
from rqt_robot_status import robot_data


class WidgetAllRobotData(QtWidgets.QFrame):

    def __init__(self, general_data, config):
        super(WidgetAllRobotData, self).__init__()
        self.general_data = general_data
        self.config = config

        self.setLayout(QtWidgets.QHBoxLayout())
        self.layout().setContentsMargins(5, 5, 5, 5)

        # ---- Vision icon ----

        self.vision_status_icon = QtWidgets.QLabel()
        self.layout().addWidget(self.vision_status_icon)

        self.vision_icon = QtGui.QIcon.fromTheme("camera-web")
        self.vision_pixmap = self.vision_icon.pixmap(self.vision_icon.actualSize(QtCore.QSize(32, 32)))
        self.vision_status_icon.setPixmap(self.vision_pixmap)

        self.vision_status_label = QtWidgets.QLabel("Vision")
        self.layout().addWidget(self.vision_status_label)

        # ---- /Vision icon ----

        # ---- Type selector ----

        self.robot_type_label = QtWidgets.QLabel("Set all: ")
        self.layout().addWidget(self.robot_type_label)

        self.robot_type_box = QtWidgets.QComboBox()
        self.layout().addWidget(self.robot_type_box)

        for item in robot_data.RobotType.TYPES:
            self.robot_type_box.addItem(item)

        # ---- /Type selector ----

        # ---- Refbox status ----

        self.refbox_stage_label = QtWidgets.QLabel("")
        self.layout().addWidget(self.refbox_stage_label)

        self.refbox_command_label = QtWidgets.QLabel("")
        self.layout().addWidget(self.refbox_command_label)

        # ---- /Refbox status ----

        self.layout().addStretch(1)

        # ---- Connect signals ----

        self.robot_type_box.activated.connect(self.change_robot_types)

        # ---- /Connect signals ----


    def update(self):
        self.vision_status_icon.setVisible(self.config.is_icons_visible())
        self.vision_status_label.setVisible(self.config.is_icons_visible())
        self.refbox_stage_label.setVisible(self.config.is_referee_data_visible())
        self.refbox_command_label.setVisible(self.config.is_referee_data_visible())

        self.robot_type_label.setVisible(self.config.is_robot_settings_visible())
        self.robot_type_box.setVisible(self.config.is_robot_settings_visible())

        vision_status = self.general_data.get_vision_status()

        if vision_status:
            # Change vision icon to OK.
            self.vision_status_label.setStyleSheet(colors.OK_STYLE)
        else:
            # Change vision icon to bad.
            self.vision_status_label.setStyleSheet(colors.BAD_STYLE)

        refbox_stage = self.general_data.get_refbox_stage()
        self.refbox_stage_label.setText(refbox_stage)

        refbox_command = self.general_data.get_refbox_command()
        self.refbox_command_label.setText(refbox_command)


    def change_robot_types(self, index):
        type_str = self.robot_type_box.itemText(index)
        new_type = robot_data.RobotType(type_str)
        self.general_data.change_all_robot_types(new_type)
