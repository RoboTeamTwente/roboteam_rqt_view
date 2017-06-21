
from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from python_qt_binding import QtGui

import colors


class WidgetStatusIcons(QtWidgets.QFrame):

    def __init__(self, bot):
        super(WidgetStatusIcons, self).__init__()

        self.bot = bot

        self.setLayout(QtWidgets.QGridLayout())
        self.setFrameStyle(QtWidgets.QFrame.Panel | QtWidgets.QFrame.Sunken)

        # Vision
        self.vision_status_icon = QtWidgets.QLabel()
        self.layout().addWidget(self.vision_status_icon, 0, 0)

        self.vision_icon = QtGui.QIcon.fromTheme("camera-web")
        self.vision_pixmap = self.vision_icon.pixmap(self.vision_icon.actualSize(QtCore.QSize(32, 32)))
        self.vision_status_icon.setPixmap(self.vision_pixmap)

        self.vision_status_label = QtWidgets.QLabel("Vision")
        self.layout().addWidget(self.vision_status_label, 0, 1)

        # Connection
        self.connection_status_icon = QtWidgets.QLabel()
        self.layout().addWidget(self.connection_status_icon, 1, 0)

        self.connection_active_icon = QtGui.QIcon.fromTheme("network-wireless")
        self.connection_active_pixmap = self.connection_active_icon.pixmap(self.connection_active_icon.actualSize(QtCore.QSize(32, 32)))
        self.connection_idle_icon = QtGui.QIcon.fromTheme("network-offline")
        self.connection_idle_pixmap = self.connection_idle_icon.pixmap(self.connection_idle_icon.actualSize(QtCore.QSize(32, 32)))
        self.connection_error_icon = QtGui.QIcon.fromTheme("network-error")
        self.connection_error_pixmap = self.connection_error_icon.pixmap(self.connection_error_icon.actualSize(QtCore.QSize(32, 32)))

        self.connection_status_icon.setPixmap(self.connection_idle_pixmap)

        self.connection_status_label = QtWidgets.QLabel("?/?")
        self.layout().addWidget(self.connection_status_label, 1, 1)

        # Battery
        self.battery_status_icon = QtWidgets.QLabel()
        self.layout().addWidget(self.battery_status_icon, 2, 0)

        self.battery_icon = QtGui.QIcon.fromTheme("battery-low")
        self.battery_pixmap = self.battery_icon.pixmap(self.battery_icon.actualSize(QtCore.QSize(32, 32)))
        self.battery_status_icon.setPixmap(self.battery_pixmap)

        self.battery_status_label = QtWidgets.QLabel("...")
        self.layout().addWidget(self.battery_status_label, 2, 1)

    def update(self):
        vision_status = self.bot.get_vision_status()

        if vision_status:
            # Change vision icon to OK.
            self.vision_status_label.setStyleSheet(colors.OK_STYLE)
        else:
            # Change vision icon to bad.
            self.vision_status_label.setStyleSheet(colors.BAD_STYLE)

        (acks, nacks) = self.bot.get_connection_status()
        total_packets = acks + nacks

        self.connection_status_label.setText(str(acks) + "/" + str(total_packets))

        if total_packets == 0:
            # No packets sent.
            self.connection_status_label.setStyleSheet(colors.FADED_STYLE)
            self.connection_status_icon.setPixmap(self.connection_idle_pixmap)
        elif (float(acks) / total_packets) < 0.6:
            # Less than 60 % of packets arrive!
            self.connection_status_label.setStyleSheet(colors.BAD_STYLE)
            self.connection_status_icon.setPixmap(self.connection_error_pixmap)
        else:
            self.connection_status_label.setStyleSheet(colors.OK_STYLE)
            self.connection_status_icon.setPixmap(self.connection_active_pixmap)
