
from python_qt_binding import QtWidgets
from python_qt_binding import QtGui


class WidgetStrategyCommandData(QtWidgets.QFrame):

    def __init__(self, strategy_data):
        super(WidgetStrategyCommandData, self).__init__()

        self.strategy_data = strategy_data

        self.setLayout(QtWidgets.QHBoxLayout())
        self.setFrameStyle(QtWidgets.QFrame.Panel | QtWidgets.QFrame.Sunken)

        self.interpreted_command_label = QtWidgets.QLabel()
        self.layout().addWidget(self.interpreted_command_label)

        self.current_strategy_label = QtWidgets.QLabel()
        self.layout().addWidget(self.current_strategy_label)


    def update(self):
        self.interpreted_command_label.setText(self.strategy_data.get_interpreted_command())
        self.current_strategy_label.setText(self.strategy_data.get_current_strategy())
