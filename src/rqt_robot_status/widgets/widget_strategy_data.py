from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from python_qt_binding import QtGui

import colors
from rqt_robot_status import robot_data
import widget_strategy_command_data
import widget_strategy_play_data


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

        self.id_font = QtGui.QFont("Arial", 20, QtGui.QFont.Bold)

        self.id_label = QtWidgets.QLabel("S")
        self.id_label.setFont(self.id_font)
        self.id_label.setMaximumHeight(20)
        self.layout().addWidget(self.id_label)

        self.widget_strategy_command_data = widget_strategy_command_data.WidgetStrategyCommandData(self.strategy_data)
        self.layout().addWidget(self.widget_strategy_command_data)

        self.widget_strategy_play_data = widget_strategy_play_data.WidgetStrategyPlayData(self.strategy_data)
        self.layout().addWidget(self.widget_strategy_play_data)

        # ---- /Widgets ----

        self.layout().addStretch(1)


    def update(self):
        # ---- Apply config ----

        self.widget_strategy_command_data.setVisible(self.config.is_referee_data_visible())

        # ---- /Apply config ----

        self.widget_strategy_command_data.update()
        self.widget_strategy_play_data.update()
