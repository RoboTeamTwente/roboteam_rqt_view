import os
import rospy
import rospkg
import actionlib
import roslib
roslib.load_manifest("roboteam_msgs")

import math

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore, QtGui, QtWidgets
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtWidgets import QWidget, QLabel, QGraphicsScene, QGraphicsItemGroup, QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsTextItem, QGraphicsRectItem

from roboteam_msgs.msg import World as WorldMessage
from roboteam_msgs.msg import GeometryData as GeometryMessage
from roboteam_msgs.msg import SteeringAction, SteeringGoal

from field_graphics_view import FieldGraphicsView
from field_graphics_scene import FieldGraphicsScene


BOT_DIAMETER = 180 # Diameter of the bots in mm.


class WorldViewPlugin(Plugin):

    # Qt signal for when the world state changes.
    worldstate_signal = pyqtSignal(WorldMessage)
    # Qt signal for when the graphics calibration changes.
    geometry_signal = pyqtSignal(GeometryMessage)

    # Graphic representations of the blue and yellow robots.
    # QGraphicsItemGroup.
    robots_blue = {}
    robots_yellow = {}

    # References to the selection circles.
    robots_blue_selectors = {}
    robots_yellow_selectors = {}

    # List of selected robot id's.
    robots_blue_selected = []
    robots_yellow_selected = []

    ball = QGraphicsEllipseItem(0, 0, 50, 50)


    # Field size in mm.
    field_width = 9000
    field_height = 6000

    field_background = QGraphicsRectItem(-field_width/2, -field_height/2, field_width, field_height)


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

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('roboteam_rqt_view'), 'resource', 'WorldView.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self.widget)

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
        self.worldstate_sub = rospy.Subscriber("world_state", WorldMessage, self.worldstate_callback)

        # Subscribe to the geometry information.
        self.geometry_sub = rospy.Subscriber("vision_geometry", GeometryMessage, self.geometry_callback)

        # Create the steering action client.
        self.client = actionlib.SimpleActionClient("steering", SteeringAction)

        # ---- Field view initialization ----

        self.scene = FieldGraphicsScene();
        self.fieldview = FieldGraphicsView()
        self.fieldview.setScene(self.scene)

        # Add to the main window.
        self.widget.layout().addWidget(self.fieldview)

        #self.fieldview.setDragMode(FieldGraphicsView.RubberBandDrag)

        self.scene.clear()

        # Add the field to the scene.
        self.scene.addItem(self.field_background)
        self.field_background.setBrush(QtGui.QBrush(QtGui.QColor(0, 200, 50)))

        # Add the ball to the scene.
        self.scene.addItem(self.ball)
        # Ball = orange.
        self.ball.setBrush(QtGui.QBrush(QtGui.QColor(255, 100, 0)))

        # Scale the scene so that it fits into the view area.
        self.fieldview.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

        # ---- /Field view initialization ----

        self.font = QtGui.QFont()
        self.font.setPixelSize(BOT_DIAMETER*0.8)

        # ---- Signal connections ----

        # Connect the signal sent by the worldstate callback to the message slot.
        self.worldstate_signal.connect(self.worldstate_slot)
        # Connect the Geometry callback to the Geometry slot.
        self.geometry_signal.connect(self.geometry_slot)

        # Connect the scene's selectionChanged signal.
        self.scene.selectionChanged.connect(self.selection_changed_slot)

        # Connect the scenes right click signal.
        self.scene.right_click_signal.connect(self.view_right_clicked_slot)

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
    def worldstate_callback(self, message):
        # Send signal to qt thread.
        self.worldstate_signal.emit(message)


    # Receives the changeUI(PyQt_PyObject) signal which gets sent when a message arrives at 'message_callback'.
    def worldstate_slot(self, message):
        # Move the ball.
        self.ball.setPos(message.ball.pos.x, -(message.ball.pos.y))

        # Process the blue bots.
        for bot in message.robots_blue:
            if not bot.id in self.robots_blue:
                (self.robots_blue[bot.id], self.robots_blue_selectors[bot.id]) = \
                        self.create_new_robot(bot.id, QtGui.QColor(0, 100, 255))
                self.scene.addItem(self.robots_blue[bot.id])

            screen_bot = self.robots_blue[bot.id]
            screen_bot.setPos(bot.pos.x, -bot.pos.y)
            screen_bot.setRotation(-math.degrees(bot.w))

        # Draw the yellow bots.
        for bot in message.robots_yellow:
            if not bot.id in self.robots_yellow:
                (self.robots_yellow[bot.id], self.robots_yellow_selectors[bot.id]) = \
                        self.create_new_robot(bot.id, QtGui.QColor(255, 255, 0))
                self.scene.addItem(self.robots_yellow[bot.id])

            screen_bot = self.robots_yellow[bot.id]
            screen_bot.setPos(bot.pos.x, -bot.pos.y)
            screen_bot.setRotation(-math.degrees(bot.w))

        # Scale the scene so that it fits into the view area.
        self.fieldview.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)



    def geometry_callback(self, message):
        # Send signal to qt thread.
        self.geometry_signal.emit(message)


    def geometry_slot(self, message):
        self.field_width = message.field.field_width
        self.field_length = message.field.field_length

        # Resize the field background.
        #self.field_background.setRect(-self.field_width/2, -self.field_length/2, self.field_width, self.field_length)

        rospy.loginfo("Field width: %i", self.field_width)
        rospy.loginfo("Field length: %i", self.field_length)


    # Slot for the scenes selectionChanged signal.
    def selection_changed_slot(self):
        # Clear the selection lists.
        del self.robots_blue_selected[:]
        del self.robots_yellow_selected[:]

        for bot_id, selector in self.robots_blue_selectors.iteritems():
            if selector.isSelected():
                selector.setVisible(True)
                self.robots_blue_selected.append(bot_id)
            else:
                selector.setVisible(False)

        for bot_id, selector in self.robots_yellow_selectors.iteritems():
            if selector.isSelected():
                selector.setVisible(True)
                self.robots_yellow_selected.append(bot_id)
            else:
                selector.setVisible(False)


    # Called when the view is right clicked.
    def view_right_clicked_slot(self, event):
        #TODO: Only send actions when there is a server connected.

        for bot_id in self.robots_yellow_selected:
            goal_pos = event.scenePos()

            goal = SteeringGoal()
            goal.robot_id = bot_id

            # The message is in meters, on screen it is in mm.
            # So divide by 1000.
            goal.x = goal_pos.x()/1000.0
            goal.y = -goal_pos.y()/1000.0

            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration.from_sec(1.0))



    # Creates a QGraphicsItemGroup that represents a robot.
    # Takes an integer as bot id.
    # Takes a QColor to color the bot with.
    # Returns (QGraphicsItemGroup, QGraphicsEllipseItem)
    # Returns the QGraphicsItemGroup representing the bot
    # plus the QGraphicsEllipse used for indicating the bot is selected.
    def create_new_robot(self, bot_id, color):
        bot = QGraphicsItemGroup()

        # Make the bot selectable.
        bot.setFlag(QtWidgets.QGraphicsItem.ItemIsSelectable, True)

        ellipse = QGraphicsEllipseItem(-BOT_DIAMETER/2, -BOT_DIAMETER/2, BOT_DIAMETER, BOT_DIAMETER)
        ellipse.setBrush(QtGui.QBrush(color))
        bot.addToGroup(ellipse)

        line_pen = QtGui.QPen()
        line_pen.setWidth(10)
        rot_line = QGraphicsLineItem(0, 0, BOT_DIAMETER/2, 0)
        rot_line.setPen(line_pen)
        bot.addToGroup(rot_line)

        id_text = QGraphicsTextItem(str(bot_id))
        id_text.setFont(self.font)
        id_text.setPos(-BOT_DIAMETER/3,-BOT_DIAMETER/2)
        bot.addToGroup(id_text)

        selection_pen = QtGui.QPen(QtGui.QColor(0, 255, 255))
        selection_pen.setWidth(10)

        # Pixels for the selection circle to be bigger than the bot.
        offset = BOT_DIAMETER * 0.3

        selector_ellipse = QGraphicsEllipseItem(-((BOT_DIAMETER/2)+offset/2), -((BOT_DIAMETER/2)+offset/2), BOT_DIAMETER + offset, BOT_DIAMETER + offset)
        selector_ellipse.setPen(selection_pen)
        selector_ellipse.setVisible(False)
        bot.addToGroup(selector_ellipse)

        return (bot, selector_ellipse)

        #id_text = self.scene.addText(str(bot.id), self.font)
        #id_text.setPos(bot.pos.x - BOT_DIAMETER*0.2, -(bot.pos.y - BOT_DIAMETER*0.5))
