from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from python_qt_binding import QtGui


class WidgetValueDisplay(QtWidgets.QFrame):

    def __init__(self, bot):
        super(WidgetValueDisplay, self).__init__()

        self.bot = bot

        self.setLayout(QtWidgets.QGridLayout())
        self.setFrameStyle(QtWidgets.QFrame.Panel | QtWidgets.QFrame.Sunken)
        self.setMinimumWidth(200)

        # X
        self.x_label = QtWidgets.QLabel("X:")
        self.layout().addWidget(self.x_label, 0, 0)

        self.x_value = QtWidgets.QLabel("?")
        self.layout().addWidget(self.x_value, 0, 1)

        # Y
        self.y_label = QtWidgets.QLabel("Y:")
        self.layout().addWidget(self.y_label, 1, 0)

        self.y_value = QtWidgets.QLabel("?")
        self.layout().addWidget(self.y_value, 1, 1)

        # Angle
        self.a_label = QtWidgets.QLabel("A:")
        self.layout().addWidget(self.a_label, 2, 0)

        self.a_value = QtWidgets.QLabel("?")
        self.layout().addWidget(self.a_value, 2, 1)

        # VX
        self.vx_label = QtWidgets.QLabel("VX:")
        self.layout().addWidget(self.vx_label, 0, 2)

        self.vx_value = QtWidgets.QLabel("?")
        self.layout().addWidget(self.vx_value, 0, 3)

        # VY
        self.vy_label = QtWidgets.QLabel("VY:")
        self.layout().addWidget(self.vy_label, 1, 2)

        self.vy_value = QtWidgets.QLabel("?")
        self.layout().addWidget(self.vy_value, 1, 3)

        # W
        self.w_label = QtWidgets.QLabel("W:")
        self.layout().addWidget(self.w_label, 2, 2)

        self.w_value = QtWidgets.QLabel("?")
        self.layout().addWidget(self.w_value, 2, 3)

    def update(self):
        (pos_x, pos_y) = self.bot.get_pos()

        self.x_value.setText(str(round(pos_x, 2)))
        self.y_value.setText(str(round(pos_y, 2)))

        (vel_x, vel_y) = self.bot.get_vel()

        self.vx_value.setText(str(round(vel_x, 2)))
        self.vy_value.setText(str(round(vel_y, 2)))

        angle = self.bot.get_angle()
        angle_w = self.bot.get_w()

        self.a_value.setText(str(round(angle, 2)))
        self.w_value.setText(str(round(angle_w, 2)))
