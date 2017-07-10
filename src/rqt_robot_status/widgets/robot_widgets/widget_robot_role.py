from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from python_qt_binding import QtGui


class WidgetRobotRole(QtWidgets.QFrame):

    def __init__(self, bot, config):
        super(WidgetRobotRole, self).__init__()

        self.bot = bot
        self.config = config

        self.setLayout(QtWidgets.QGridLayout())
        self.setFrameStyle(QtWidgets.QFrame.Panel | QtWidgets.QFrame.Sunken)

        self.role_label = QtWidgets.QLabel("Role:")
        self.layout().addWidget(self.role_label, 0, 0)

        self.role_name = QtWidgets.QLabel()
        self.layout().addWidget(self.role_name, 0, 1)

        self.stop_button = QtWidgets.QPushButton("Stop role")
        self.layout().addWidget(self.stop_button, 1, 0, 1, 2)

        # ---- Connect signals ----

        self.stop_button.clicked.connect(self.activate_stop_callback)

        # ---- /Connect signals ----


    def update(self):
        role = self.bot.get_role()

        self.role_name.setText(role)


    def activate_stop_callback(self):
        callback = self.config.get_stop_callback()
        if callback:
            callback(self.bot.id)

        self.bot.clear_role_status()
