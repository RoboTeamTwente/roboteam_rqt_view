
from python_qt_binding import QtCore
from python_qt_binding.QtCore import Qt
from python_qt_binding import QtWidgets
from python_qt_binding import QtGui


class WidgetPluginConfig(QtWidgets.QFrame):

    def __init__(self, config):
        super(WidgetPluginConfig, self).__init__()

        self.config = config

        self.setLayout(QtWidgets.QHBoxLayout())

        # ---- Scroll area ----

        self.scroll_area = QtWidgets.QScrollArea()
        self.scroll_frame = QtWidgets.QFrame();
        self.scroll_layout = QtWidgets.QHBoxLayout(self.scroll_frame)
        self.scroll_area.setWidget(self.scroll_frame)

        self.scroll_area.setContentsMargins(0, 0, 0, 0)
        self.scroll_frame.setContentsMargins(0, 0, 0, 0)
        self.scroll_layout.setContentsMargins(0, 0, 0, 0)

        # This shouldn't be necessary.
        # A scroll area with no vertical scroll bar should automatically resize to the minimum size of it's contents.
        # It doesn't, so this arbitrary height is needed.
        self.scroll_area.setMaximumHeight(40)

        self.scroll_area.setSizePolicy(self.scroll_frame.sizePolicy().Expanding, self.scroll_frame.sizePolicy().MinimumExpanding)

        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setFrameStyle(QtWidgets.QFrame.NoFrame)
        self.scroll_area.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

        self.layout().addWidget(self.scroll_area)

        # ---- /Scroll area ----

        self.pattern_checkbox = QtWidgets.QCheckBox("Patterns")
        self.scroll_layout.addWidget(self.pattern_checkbox)
        self.pattern_checkbox.stateChanged.connect(self.pattern_state_changed)

        self.icons_checkbox = QtWidgets.QCheckBox("Icons")
        self.scroll_layout.addWidget(self.icons_checkbox)
        self.icons_checkbox.stateChanged.connect(self.icons_state_changed)

        self.values_checkbox = QtWidgets.QCheckBox("Values")
        self.scroll_layout.addWidget(self.values_checkbox)
        self.values_checkbox.stateChanged.connect(self.values_state_changed)

        self.settings_checkbox = QtWidgets.QCheckBox("Settings")
        self.scroll_layout.addWidget(self.settings_checkbox)
        self.settings_checkbox.stateChanged.connect(self.robot_settings_state_changed)

        self.role_info_checkbox = QtWidgets.QCheckBox("Role info")
        self.scroll_layout.addWidget(self.role_info_checkbox)
        self.role_info_checkbox.stateChanged.connect(self.role_info_state_changed)

        self.scroll_layout.addStretch(2)

        self.update_from_config()


    def update_from_config(self):
        self.pattern_checkbox.setChecked(self.config.is_icons_visible())
        self.icons_checkbox.setChecked(self.config.is_icons_visible())
        self.values_checkbox.setChecked(self.config.is_value_display_visible())
        self.settings_checkbox.setChecked(self.config.is_robot_settings_visible())
        self.role_info_checkbox.setChecked(self.config.is_robot_role_visible())


    def pattern_state_changed(self, state):
        if state == Qt.Checked:
            self.config.set_pattern_visible(True)
        else:
            self.config.set_pattern_visible(False)

    def icons_state_changed(self, state):
        if state == Qt.Checked:
            self.config.set_icons_visible(True)
        else:
            self.config.set_icons_visible(False)

    def values_state_changed(self, state):
        if state == Qt.Checked:
            self.config.set_value_display_visible(True)
        else:
            self.config.set_value_display_visible(False)

    def robot_settings_state_changed(self, state):
        if state == Qt.Checked:
            self.config.set_robot_settings_visible(True)
        else:
            self.config.set_robot_settings_visible(False)

    def role_info_state_changed(self, state):
        if state == Qt.Checked:
            self.config.set_robot_role_visible(True)
        else:
            self.config.set_robot_role_visible(False)
