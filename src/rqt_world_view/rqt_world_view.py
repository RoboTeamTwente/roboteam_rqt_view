import os
import rospy
import roslib
roslib.load_manifest("roboteam_msgs")

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore, QtGui, QtWidgets
from python_qt_binding.QtCore import pyqtSignal, Qt, QTimer
from python_qt_binding.QtWidgets import QWidget, QLabel, QSplitter, QVBoxLayout

from roboteam_msgs import msg
from std_msgs import msg as std_msg

from items.widget_robot_details import WidgetRobotDetails
from items.widget_world_view import WidgetWorldView
from items.widget_scoreboard import WidgetScoreboard
from items.widget_skill_tester import WidgetSkillTester
from items.widget_multi_skill_tester import WidgetMultiSkillTester

FIELD_COLOR = QtGui.QColor(0, 200, 50)
FIELD_LINE_COLOR = QtGui.QColor(255, 255, 255)
BALL_COLOR = QtGui.QColor(255, 100, 0)

US_COLOR = QtGui.QColor(255, 50, 50) # The color of our robots.
THEM_COLOR = QtGui.QColor(127, 84, 147) # The color of the opponents robots.

# Milliseconds to wait for a vision packet before assuming it has stopped.
VISION_TIMEOUT_TIME = 2000


class WorldViewPlugin(Plugin):

    # Qt signal for when the world state changes.
    worldstate_signal = pyqtSignal(msg.World)
    # Qt signal for when the graphics calibration changes.
    geometry_signal = pyqtSignal(msg.GeometryData)
    referee_signal = pyqtSignal(msg.RefereeData)

    debug_point_signal = pyqtSignal(msg.DebugPoint)
    debug_line_signal = pyqtSignal(msg.DebugLine)

    halt_update_signal = pyqtSignal(std_msg.Bool)

    def __init__(self, context):
        super(WorldViewPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('WorldViewPlugin')

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
        self.widget.setWindowTitle("World view")
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self.widget)

        # ---- Subscribers ----

        # Subscribe to the world state.
        self.worldstate_sub = rospy.Subscriber("world_state", msg.World, self.callback_worldstate)

        # Subscribe to the geometry information.
        self.geometry_sub = rospy.Subscriber("vision_geometry", msg.GeometryData, self.callback_geometry)

        # Subscribe to the referee information.
        self.referee_sub = rospy.Subscriber("vision_refbox", msg.RefereeData, self.callback_referee)


        # Through this channel debugging locations can be given.
        self.debug_point_sub = rospy.Subscriber("view_debug_points", msg.DebugPoint, self.callback_debug_point)
        self.debug_line_sub = rospy.Subscriber("view_debug_lines", msg.DebugLine, self.callback_debug_line)

        # Whenever something broadcasts a halt, this subscriber gets it
        self.halt_sub = rospy.Subscriber("halt", std_msg.Bool, self.callback_halt)

        # ---- /Subscribers ----

        # ---- Topics ----

        # Create the Strategy Ignore Robot topic.
        self.strategy_ignore_topic = rospy.Publisher("strategy_ignore_robot", msg.StrategyIgnoreRobot, queue_size=100)

        # Halt publisher such that the widget can halt as well
        self.halt_pub = rospy.Publisher("halt", std_msg.Bool, queue_size = 10)

        # ---- /Topics ----

        # ---- Top layout ----

        self.widget.setLayout(QVBoxLayout())

        self.vertical_splitter = QSplitter(Qt.Vertical)
        self.widget.layout().addWidget(self.vertical_splitter)

        self.horizontal_splitter = QSplitter(Qt.Horizontal)
        self.vertical_splitter.addWidget(self.horizontal_splitter)

        # ---- /Top layout ----

        # ---- World viewer ----

        self.world_view = WidgetWorldView(US_COLOR, THEM_COLOR, self.halt_pub)

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
        self.sidebar.layout().setContentsMargins(0, 0, 0, 0)
        self.horizontal_splitter.addWidget(self.sidebar)

        self.multi_skill_tester = WidgetMultiSkillTester(self.strategy_ignore_topic)
        self.sidebar.layout().addWidget(self.multi_skill_tester)

        #self.sidebar.layout().addStretch(1)

        # ---- /Sidebar ----

        # Make only the main view expand when the window resizes.
        self.vertical_splitter.setStretchFactor(0, 0)
        self.vertical_splitter.setStretchFactor(1, 1)

        self.horizontal_splitter.setStretchFactor(0, 1)
        self.horizontal_splitter.setStretchFactor(1, 0)

        # ---- Vision status timer ----

        self.vision_timer = QTimer()
        self.vision_timer.setInterval(VISION_TIMEOUT_TIME)
        self.vision_timer.setSingleShot(True)
        self.vision_timer.timeout.connect(self.slot_vision_lost)

        self.has_vision_connection = False

        # ---- /Vision status timer ----

        # ---- Signal connections ----

        # Connect the signal sent by the worldstate callback to the message slot.
        self.worldstate_signal.connect(self.slot_worldstate)
        # Connect the Geometry callback to the Geometry slot.
        self.geometry_signal.connect(self.slot_geometry)
        self.referee_signal.connect(self.slot_referee)

        self.debug_point_signal.connect(self.slot_debug_point)
        self.debug_line_signal.connect(self.slot_debug_line)

        self.halt_update_signal.connect(self.slot_halt_update)

        # ---- /Signal connections ----


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.strategy_ignore_topic.unregister()
        self.halt_pub.unregister()

        self.debug_point_sub.unregister()
        self.debug_line_sub.unregister()
        self.worldstate_sub.unregister()
        self.geometry_sub.unregister()
        self.referee_sub.unregister()
        self.halt_sub.unregister()

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

    def callback_debug_point(self, message):
        self.debug_point_signal.emit(message)

    def callback_debug_line(self, message):
        self.debug_line_signal.emit(message)

    def callback_halt(self, message):
        self.halt_update_signal.emit(message)

# ------------------------------------------------------------------------------
# ---------- Gui change slots --------------------------------------------------
# ------------------------------------------------------------------------------


    def slot_worldstate(self, message):
        """Receives the changeUI(PyQt_PyObject) signal which gets sent when a message arrives at 'message_callback'."""
        self.world_view.update_world_state(message)

        # Reset the vision timeout timer.
        self.vision_timer.start()

        if self.has_vision_connection == False:
            self.world_view.set_vision_status_indicator(True)
            self.has_vision_connection = True


    def slot_geometry(self, message):
        self.world_view.update_field_configuration(message)


    def slot_referee(self, message):
        self.scoreboard.update_with_message(message)

    def slot_debug_point(self, message):
        self.world_view.set_debug_point(message)

    def slot_debug_line(self, message):
        self.world_view.set_debug_line(message)

    def slot_halt_update(self, message):
        self.world_view.halt_update(message)


    def slot_vision_lost(self):
        """Call when there hasn't been a packet from vision for a while."""
        self.world_view.set_vision_status_indicator(False)
        self.has_vision_connection = False
