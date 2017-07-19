from python_qt_binding import QtWidgets
from python_qt_binding import QtGui


class WidgetStrategyPlayData(QtWidgets.QFrame):

    def __init__(self, strategy_data):
        super(WidgetStrategyPlayData, self).__init__()

        self.strategy_data = strategy_data

        self.setLayout(QtWidgets.QVBoxLayout())
        self.setFrameStyle(QtWidgets.QFrame.Panel | QtWidgets.QFrame.Sunken)

        self.play_texts = []

    def update(self):
        for widget in self.play_texts:
            widget.deleteLater()

        del self.play_texts[:]

        for name, robots in self.strategy_data.plays.items():
            text = name
            for bot in robots:
                text += " " + str(bot)
            widget = QtWidgets.QLabel(text)
            self.play_texts.append(widget)
            self.layout().addWidget(widget)
