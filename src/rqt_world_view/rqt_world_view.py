import os
import rospy
import rospkg
import actionlib
import roslib
roslib.load_manifest("roboteam_msgs")

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore, QtGui, QtWidgets
from python_qt_binding.QtCore import pyqtSignal, Qt
from python_qt_binding.QtWidgets import QWidget, QLabel, QSplitter, QVBoxLayout

from roboteam_msgs.msg import World as WorldMessage
from roboteam_msgs.msg import GeometryData as GeometryMessage
from roboteam_msgs.msg import RefereeData as RefboxMessage
from roboteam_msgs.msg import SteeringAction, SteeringGoal

from items.widget_robot_details import WidgetRobotDetails
from items.widget_world_view import WidgetWorldView
from items.widget_scoreboard import WidgetScoreboard
from items.widget_skill_tester import WidgetSkillTester


# Size of the area around the field in mm.
FIELD_RUNOUT_ZONE = 300

FIELD_COLOR = QtGui.QColor(0, 200, 50)
FIELD_LINE_COLOR = QtGui.QColor(255, 255, 255)
BALL_COLOR = QtGui.QColor(255, 100, 0)

US_COLOR = QtGui.QColor(255, 50, 50); # The color of our robots.
THEM_COLOR = QtGui.QColor(127, 84, 147); # The color of the opponents robots.


class WorldViewPlugin(Plugin):

    # Qt signal for when the world state changes.
    worldstate_signal = pyqtSignal(WorldMessage)
    # Qt signal for when the graphics calibration changes.
    geometry_signal = pyqtSignal(GeometryMessage)
    referee_signal = pyqtSignal(RefboxMessage)


    def __init__(self, context):
        super(WorldViewPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('WorldViewPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "---quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns


        # Create QWidget
        self.widget = QWidget()

        # # Get path to UI file which should be in the "resource" folder of this package
        # ui_file = os.path.join(rospkg.RosPack().get_path('roboteam_rqt_view'), 'resource', 'WorldView.ui')
        # # Extend the widget with all attributes and children from UI file
        # loadUi(ui_file, self.widget)

        # Give QObjects reasonable names
        self.widget.setObjectName('WorldView')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self.widget)

        # Subscribe to the world state.
        self.worldstate_sub = rospy.Subscriber("world_state", WorldMessage, self.callback_worldstate)

        # Subscribe to the geometry information.
        self.geometry_sub = rospy.Subscriber("vision_geometry", GeometryMessage, self.callback_geometry)

        # Subscribe to the referee information.
        self.referee_sub = rospy.Subscriber("vision_refbox", RefboxMessage, self.callback_referee)

        # Create the steering action client.
        self.client = actionlib.SimpleActionClient("steering", SteeringAction)

        # ---- Top layout ----

        self.widget.setLayout(QVBoxLayout())

        self.vertical_splitter = QSplitter(Qt.Vertical)
        self.widget.layout().addWidget(self.vertical_splitter)

        self.horizontal_splitter = QSplitter(Qt.Horizontal)
        self.vertical_splitter.addWidget(self.horizontal_splitter)

        # ---- /Top layout ----

        # ---- World viewer ----

        self.world_view = WidgetWorldView(US_COLOR, THEM_COLOR)

        self.horizontal_splitter.addWidget(self.world_view)

        # ---- /World viewer ----

        # ---- Score board ----

        self.scoreboard = WidgetScoreboard(US_COLOR, THEM_COLOR)
        # Insert it all the way at the top.
        self.vertical_splitter.insertWidget(0, self.scoreboard)

        # ---- /Score board ----

        # ---- Sidebar ----

        self.sidebar = QtWidgets.QFrame()
        self.sidebar.setSizePolicy(self.sidebar.sizePolicy().Preferred, self.sidebar.sizePolicy().Expanding)
        self.sidebar.setLayout(QtWidgets.QVBoxLayout())
        self.horizontal_splitter.addWidget(self.sidebar)

        self.skill_tester = WidgetSkillTester()
        self.sidebar.layout().addWidget(self.skill_tester)

        self.sidebar.layout().addStretch(1)

        # ---- /Sidebar ----

        # ---- Signal connections ----

        # Connect the signal sent by the worldstate callback to the message slot.
        self.worldstate_signal.connect(self.slot_worldstate)
        # Connect the Geometry callback to the Geometry slot.
        self.geometry_signal.connect(self.slot_geometry)
        self.referee_signal.connect(self.slot_referee)

        # ---- /Signal connections ----


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.worldstate_sub.unregister()
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


    # Is called when a worldstate message is received.
    def callback_worldstate(self, message):
        # Send signal to qt thread.
        self.worldstate_signal.emit(message)


    def callback_referee(self, message):
        self.referee_signal.emit(message)


    def callback_geometry(self, message):
        # Send signal to qt thread.
        self.geometry_signal.emit(message)


# ------------------------------------------------------------------------------
# ---------- Gui change slots --------------------------------------------------
# ------------------------------------------------------------------------------


    # Receives the changeUI(PyQt_PyObject) signal which gets sent when a message arrives at 'message_callback'.
    def slot_worldstate(self, message):
        self.world_view.update_world_state(message)


    def slot_geometry(self, message):
        self.world_view.update_field_configuration(message)


    def slot_referee(self, message):
        self.scoreboard.update_with_message(message)
