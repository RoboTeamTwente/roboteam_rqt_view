from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from python_qt_binding import QtGui

import colors
from rqt_robot_status import robot_data


class WidgetStrategyData(QtWidgets.QFrame):

    def __init__(self, strategy_data, config):
        super(WidgetStrategyData, self).__init__()
        self.strategy_data = strategy_data
        self.config = config

        self.setLayout(QtWidgets.QHBoxLayout())
        self.layout().setContentsMargins(5, 5, 5, 5)

        self.setFrameStyle(QtWidgets.QFrame.Box)
        self.setLineWidth(1)

        # ---- Widgets ----

        self.interpreted_command_label = QtWidgets.QLabel()
        self.layout().addWidget(self.interpreted_command_label)

        self.current_strategy_label = QtWidgets.QLabel()
        self.layout().addWidget(self.current_strategy_label)

        # ---- /Widgets ----

        self.layout().addStretch(1)


    def update(self):
        # ---- Apply config ----

        self.interpreted_command_label.setVisible(self.config.is_referee_data_visible())
        self.current_strategy_label.setVisible(self.config.is_robot_role_visible())

        # ---- /Apply config ----

        self.interpreted_command_label.setText(self.strategy_data.get_interpreted_command())
        self.current_strategy_label.setText(self.strategy_data.get_current_strategy())
