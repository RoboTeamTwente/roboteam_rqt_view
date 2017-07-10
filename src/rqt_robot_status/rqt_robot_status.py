import rospy
import roslib
roslib.load_manifest("roboteam_msgs")

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore, QtGui, QtWidgets
from python_qt_binding import QtCore


from roboteam_msgs import msg


import robot_map
import plugin_config
from widgets import widget_robot_list, widget_plugin_config


# Milliseconds to wait for detection packages before we will assume
# the connection to vision is lost.
DETECTION_TIMEOUT = 1000


class RobotStatusPlugin(Plugin):

    received_detection_message = QtCore.pyqtSignal()

    def __init__(self, context):
        super(RobotStatusPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('RobotStatusPlugin')

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
        self.widget.setObjectName('RobotStatus')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        self.widget.setWindowTitle("Robot Status")
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self.widget)

        # ---- data ----


        self.robot_map = robot_map.RobotMap()
        self.plugin_config = plugin_config.PluginConfig()

        self.plugin_config.set_stop_callback(self.send_rolenode_stop_command)


        # ---- /data ----

        # ---- Widgets ----

        self.widget.setLayout(QtWidgets.QVBoxLayout())

        self.config_widget = widget_plugin_config.WidgetPluginConfig(self.plugin_config)
        self.widget.layout().addWidget(self.config_widget, stretch=0)

        self.robot_list_widget = widget_robot_list.WidgetRobotList(self.robot_map, self.plugin_config)
        self.widget.layout().addWidget(self.robot_list_widget, stretch=1)

        # ---- /Widgets ----

        # ---- Subscribers ----

        self.worldstate_sub = rospy.Subscriber("world_state", msg.World, self.callback_worldstate)
        self.refbox_sub = rospy.Subscriber("vision_refbox", msg.RefereeData, self.callback_refbox)
        self.serial_status_sub = rospy.Subscriber("robot_serial_status", msg.RobotSerialStatus, self.callback_serial_status)
        self.bt_debug_sub = rospy.Subscriber("bt_debug_info", msg.BtDebugInfo, self.callback_bt_debug)

        # ---- /Subscribers ----

        # ---- Publishers ----

        self.role_directive_pub = rospy.Publisher("role_directive", msg.RoleDirective, queue_size=1)

        # ---- /Publishers ----

        # ---- Timers ----

        # Widget update timer.
        self.ui_update_timer = QtCore.QTimer()
        self.ui_update_timer.timeout.connect(self.update_ui)
        self.ui_update_timer.start(100) # Update every x millisecconds.

        # Parameter update timer.
        self.parameter_update_timer = QtCore.QTimer()
        self.parameter_update_timer.timeout.connect(self.update_parameters_from_ros)
        self.parameter_update_timer.start(500) # Update every x millisecconds.


        self.detection_timeout_timer = QtCore.QTimer()
        self.detection_timeout_timer.setSingleShot(True)
        self.detection_timeout_timer.timeout.connect(self.detection_timed_out)

        # ---- /Timers ----

        self.received_detection_message.connect(self.reset_detection_timeout)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.worldstate_sub.unregister()
        self.serial_status_sub.unregister()
        self.bt_debug_sub.unregister()

        self.role_directive_pub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        instance_settings.set_value("Config", self.plugin_config.to_json())

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        json_string = instance_settings.value("Config")
        self.plugin_config.from_json(json_string)
        self.config_widget.update_from_config()

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog


    def update_ui(self):
        self.robot_list_widget.update()

    def update_parameters_from_ros(self):
        self.robot_map.update_parameters_from_ros()


    def send_rolenode_stop_command(self, bot_id):
        command = msg.RoleDirective()
        command.robot_id = bot_id
        command.tree = msg.RoleDirective.STOP_EXECUTING_TREE

        self.role_directive_pub.publish(command)

    # ----------
    # Message callbacks
    # ----------

    def callback_worldstate(self, message):
        self.robot_map.update_with_detections(message.us)
        self.received_detection_message.emit()

    def callback_refbox(self, message):
        self.robot_map.update_with_refbox_message(message)

    def callback_serial_status(self, message):
        self.robot_map.update_with_serial_status(message)

    def callback_bt_debug(self, message):
        self.robot_map.update_with_role_status(message)

    # ----------
    # Slots
    # ----------

    def reset_detection_timeout(self):
        self.detection_timeout_timer.start(DETECTION_TIMEOUT)

    def detection_timed_out(self):
        self.robot_map.detection_timed_out()
