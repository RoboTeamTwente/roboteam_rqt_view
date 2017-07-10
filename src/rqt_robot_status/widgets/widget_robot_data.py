
from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from python_qt_binding import QtGui


from robot_widgets import widget_robot_settings, \
    widget_status_icons, \
    widget_value_display, \
    widget_robot_role, \
    widget_color_pattern, \
    widget_graph_display
from rqt_robot_status.widgets import colors


class WidgetRobotData(QtWidgets.QFrame):

    def __init__(self, bot, config):
        super(WidgetRobotData, self).__init__()
        self.bot_id = bot.id
        self.bot_data = bot
        self.config = config

        self.delete_callback = None


        self.setLayout(QtWidgets.QHBoxLayout())
        self.layout().setContentsMargins(5, 5, 5, 5)

        self.setFrameStyle(QtWidgets.QFrame.Box)
        self.setLineWidth(1)


        self.id_layout = QtWidgets.QVBoxLayout()
        self.layout().addLayout(self.id_layout)

        self.close_button = QtWidgets.QPushButton("X")
        self.close_button.setMaximumWidth(20)
        self.close_button.setMaximumHeight(20)
        self.close_button.setStyleSheet(colors.FADED_STYLE)
        self.id_layout.addWidget(self.close_button)

        self.id_layout.addStretch(1)

        self.id_font = QtGui.QFont("Arial", 20, QtGui.QFont.Bold)

        self.id_label = QtWidgets.QLabel(str(self.bot_id))
        self.id_label.setFont(self.id_font)
        self.id_label.setMaximumHeight(20)
        self.id_layout.addWidget(self.id_label)

        self.id_layout.addStretch(1)

        # ---- /Info panels ----

        self.color_pattern = widget_color_pattern.WidgetColorPattern(self.bot_data)
        self.layout().addWidget(self.color_pattern)

        self.status_icons = widget_status_icons.WidgetStatusIcons(self.bot_data)
        self.layout().addWidget(self.status_icons)


        self.value_display = widget_value_display.WidgetValueDisplay(self.bot_data)
        self.layout().addWidget(self.value_display)


        # self.graph_display = widget_graph_display.WidgetGraphDisplay(self.bot_data)
        # self.layout().addWidget(self.graph_display)


        self.robot_settings = widget_robot_settings.WidgetRobotSettings(self.bot_data)
        self.layout().addWidget(self.robot_settings)


        self.robot_role = widget_robot_role.WidgetRobotRole(self.bot_data, self.config)
        self.layout().addWidget(self.robot_role)

        # ---- /Info panels ----

        self.layout().addStretch(0)

        # ---- Connect callbacks ----

        self.close_button.clicked.connect(self.close_callback)

        # ---- /Connect callbacks ----


    def update(self):
        # ---- Apply config ----

        self.color_pattern.setVisible(self.config.is_pattern_visible())
        self.status_icons.setVisible(self.config.is_icons_visible())
        self.value_display.setVisible(self.config.is_value_display_visible())
        self.robot_settings.setVisible(self.config.is_robot_settings_visible())
        self.robot_role.setVisible(self.config.is_robot_role_visible())

        # ---- /Apply config ----

        self.status_icons.update()

        self.value_display.update()

        # self.graph_display.update()

        self.robot_settings.update()

        self.robot_role.update()


    def connect_delete_callback(self, delete_callback):
        self.delete_callback = delete_callback


    def close_callback(self):
        if self.delete_callback:
            self.delete_callback(self.bot_id)
