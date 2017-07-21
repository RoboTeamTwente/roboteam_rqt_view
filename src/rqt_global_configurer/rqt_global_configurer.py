import rospy
import roslib
roslib.load_manifest("roboteam_msgs")

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore, QtGui, QtWidgets
from python_qt_binding.QtCore import pyqtSignal


PARAM_OUR_COLOR = "our_color"
PARAM_OUR_SIDE = "our_side"
PARAM_NORMALIZE = "normalize_field"


class GlobalConfigurerPlugin(Plugin):

    def __init__(self, context):
        super(GlobalConfigurerPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('GlobalConfigurerPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns


        # Create QWidget
        self.widget = QtWidgets.QWidget()

        # # Get path to UI file which should be in the "resource" folder of this package
        # ui_file = os.path.join(rospkg.RosPack().get_path('roboteam_rqt_view'), 'resource', 'WorldView.ui')
        # # Extend the widget with all attributes and children from UI file
        # loadUi(ui_file, self.widget)

        # Give QObjects reasonable names
        self.widget.setObjectName('GlobalConfigurer')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        self.widget.setWindowTitle("Global Configurer")
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self.widget)

        # ---- Widgets ----

        self.widget.setLayout(QtWidgets.QVBoxLayout())
        self.main_layout = QtWidgets.QGridLayout()
        self.widget.layout().addLayout(self.main_layout)
        self.widget.layout().addStretch(1)

        self.color_button = QtWidgets.QPushButton("")
        self.main_layout.addWidget(self.color_button, 0, 0)
        self.color_button.clicked.connect(self.color_button_pressed)

        self.side_button = QtWidgets.QPushButton("")
        self.main_layout.addWidget(self.side_button, 0, 1)
        self.side_button.clicked.connect(self.side_button_pressed)

        self.normalize_checkbox = QtWidgets.QCheckBox("Normalize field")
        self.main_layout.addWidget(self.normalize_checkbox, 1, 0, 1, 2)
        self.normalize_checkbox.stateChanged.connect(self.normalize_checkbox_changed)

        # ---- /Widgets ----

        # ---- Params ----

        self.color_param = ""
        self.side_param = ""
        self.normalize_param = False

        # ---- Params ----

        # Widget update timer.
        self.ui_update_timer = QtCore.QTimer()
        self.ui_update_timer.timeout.connect(self.update_ui)
        self.ui_update_timer.start(100) # Update every x millisecconds.

        # Parameter update timer.
        self.parameter_update_timer = QtCore.QTimer()
        self.parameter_update_timer.timeout.connect(self.update_parameters_from_ros)
        self.parameter_update_timer.start(500) # Update every x millisecconds.


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog


    def update_ui(self):
        if self.color_param == "blue":
            self.color_button.setText("Blue")
        elif self.color_param == "yellow":
            self.color_button.setText("Yellow")
        else:
            self.color_button.setText("Color?")

        if self.side_param == "left":
            self.side_button.setText("<-- Left")
        elif self.side_param == "right":
            self.side_button.setText("Right -->")
        else:
            self.side_button.setText("Side?")

        self.normalize_checkbox.setChecked(self.normalize_param)

    def update_parameters_from_ros(self):
        self.color_param = self.get_param_or_set_default(PARAM_OUR_COLOR, ["blue", "yellow"], "yellow")
        self.side_param = self.get_param_or_set_default(PARAM_OUR_SIDE, ["left", "right"], "left")
        self.normalize_param = self.get_param_or_set_default(PARAM_NORMALIZE, [True, False], True)


    def get_param_or_set_default(self, param_name, valid_values, default):
        result = default
        if rospy.has_param(param_name):
            result = rospy.get_param(param_name)

            if result not in valid_values:
                # Whoops, this parameter is invalid.
                # Reset it to yellow.
                rospy.set_param(param_name, "default")
                result = default
        else:
            rospy.set_param(param_name, "default")

        return result


    # ---------------------------
    # ---- User action slots ----
    # ---------------------------

    def color_button_pressed(self):
        if self.color_param == "blue":
            self.color_param = "yellow"
        else:
            self.color_param = "blue"
        rospy.set_param(PARAM_OUR_COLOR, self.color_param)

    def side_button_pressed(self):
        if self.side_param == "left":
            self.side_param = "right"
        else:
            self.side_param = "left"
        rospy.set_param(PARAM_OUR_SIDE, self.side_param)

    def normalize_checkbox_changed(self, state):
        if state == QtCore.Qt.Checked:
            self.normalize_param = True
        else:
            self.normalize_param = False
        rospy.set_param(PARAM_NORMALIZE, self.normalize_param)
