from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from python_qt_binding import QtGui

import colors
from rqt_robot_status import robot_data


class WidgetStrategyData(QtWidgets.QFrame):

    def __init__(self, general_data, config):
        super(WidgetStrategyData, self).__init__()
        self.general_data = general_data
        self.config = config

        self.setLayout(QtWidgets.QHBoxLayout())
        self.layout().setContentsMargins(5, 5, 5, 5)

        self.setFrameStyle(QtWidgets.QFrame.Box)
        self.setLineWidth(1)
