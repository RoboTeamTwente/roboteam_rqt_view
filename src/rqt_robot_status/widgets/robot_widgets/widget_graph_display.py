
from python_qt_binding import QtWidgets
from python_qt_binding import QtGui


class WidgetGraphDisplay(QtWidgets.QFrame):

    def __init__(self, bot):
        super(WidgetGraphDisplay, self).__init__()

        self.bot = bot

        self.setLayout(QtWidgets.QHBoxLayout())
        self.setFrameStyle(QtWidgets.QFrame.Panel | QtWidgets.QFrame.Sunken)


    def update(self):
        pass
