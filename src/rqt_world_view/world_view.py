import os
import rospy
import rospkg
import actionlib

import math

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore, QtGui
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtWidgets import QWidget, QLabel, QGraphicsScene, QGraphicsItemGroup, QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsTextItem

from roboteam_msgs.msg import World as WorldMessage
from roboteam_msgs.msg import GeometryData as GeometryMessage
from roboteam_msgs.msg import SteeringAction, SteeringGoal

from field_graphics_view import FieldGraphicsView


BOT_DIAMETER = 180 # Diameter of the bots in mm.


class WorldViewPlugin(Plugin):

    # Qt signal for when the world state changes.
    worldstate_signal = pyqtSignal(WorldMessage)

    # Graphic representations of the blue and yellow robots.
    # QGraphicsItemGroup.
    robots_blue = {}
    robots_yellow = {}

    ball = QGraphicsEllipseItem(0, 0, 50, 50)


    # Field size in mm.
    field_width = 9000
    field_height = 6000


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

        # Connect the signal sent by the worldstate callback to the message slot.
        self.worldstate_signal.connect(self.worldstate_slot)



        # ---- Field view initialization ----

        self.scene = QGraphicsScene();
        self.fieldview = FieldGraphicsView()
        self.fieldview.setScene(self.scene)

        # Add to the main window.
        self.widget.layout().addWidget(self.fieldview)

        self.reset_view()

        # Add the ball to the scene.
        self.scene.addItem(self.ball)

        # Ball = orange.
        self.ball.setBrush(QtGui.QBrush(QtGui.QColor(255, 100, 0)))

        # ---- /Field view initialization ----

        self.font = QtGui.QFont()
        self.font.setPixelSize(BOT_DIAMETER*0.8)


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

    def reset_view(self):
        self.scene.clear()

        # Draw the background.
        self.scene.addRect(-self.field_width/2, -self.field_height/2, self.field_width, self.field_height, pen=QtGui.QPen(), brush=QtGui.QBrush(QtCore.Qt.green))

        # Scale the scene so that it fits into the view area.
        self.fieldview.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)


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
                self.robots_blue[bot.id] = self.create_new_robot(bot.id, QtGui.QColor(0, 100, 255))
                self.scene.addItem(self.robots_blue[bot.id])

            screen_bot = self.robots_blue[bot.id]
            screen_bot.setPos(bot.pos.x, bot.pos.y)
            screen_bot.setRotation(-math.degrees(bot.w))

        # Draw the yellow bots.
        for bot in message.robots_yellow:
            if not bot.id in self.robots_yellow:
                self.robots_yellow[bot.id] = self.create_new_robot(bot.id, QtGui.QColor(255, 255, 0))
                self.scene.addItem(self.robots_yellow[bot.id])

            screen_bot = self.robots_yellow[bot.id]
            screen_bot.setPos(bot.pos.x, bot.pos.y)
            screen_bot.setRotation(-math.degrees(bot.w))

        # Scale the scene so that it fits into the view area.
        self.fieldview.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)



    def geometry_callback(self, message):
        self.field_width = message.field.field_width
        self.field_length = message.field.field_length

        #TODO: Resize the field rectangle.

        rospy.loginfo("Field width: %i", self.field_width)
        rospy.loginfo("Field length: %i", self.field_length)


    # Creates a QGraphicsItemGroup that represents a robot.
    # Takes an integer as bot id.
    # Takes a QColor to color the bot with.
    # Returns the QGraphicsItemGroup.
    def create_new_robot(self, bot_id, color):
        bot = QGraphicsItemGroup()

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

        return bot

        #id_text = self.scene.addText(str(bot.id), self.font)
        #id_text.setPos(bot.pos.x - BOT_DIAMETER*0.2, -(bot.pos.y - BOT_DIAMETER*0.5))
