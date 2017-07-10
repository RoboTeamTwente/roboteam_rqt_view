
from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from python_qt_binding import QtGui


import robot_color_patterns


class WidgetColorPattern(QtWidgets.QFrame):

    def __init__(self, bot):
        super(WidgetColorPattern, self).__init__()

        self.bot = bot

        self.IMAGE_SIZE = 50

        self.setLayout(QtWidgets.QVBoxLayout())
        self.setFrameStyle(QtWidgets.QFrame.Panel | QtWidgets.QFrame.Sunken)

        self.image = robot_color_patterns.draw_pattern_image(self.bot.id, self.IMAGE_SIZE)

        self.image_label = QtWidgets.QLabel()
        self.layout().addWidget(self.image_label)
        self.image_label.setPixmap(QtGui.QPixmap.fromImage(self.image))
