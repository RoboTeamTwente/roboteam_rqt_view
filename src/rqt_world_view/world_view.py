import os
import rospy
import rospkg

import math

from roboteam_msgs.msg import World as WorldMessage
from roboteam_msgs.msg import GeometryData as GeometryMessage

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore, QtGui
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtWidgets import QWidget, QLabel, QGraphicsScene

from field_graphics_view import FieldGraphicsView

BOT_DIAMETER = 180 # Diameter of the bots in mm.


class WorldViewPlugin(Plugin):

    _worldstate_signal = pyqtSignal(WorldMessage)

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


        # ---- Field parameters ----

        self.field_width = 9000 # Width of the field in mm.
        self.field_height = 6000 # Height of the field in mm.

        self.half_field_width = self.field_width*0.5
        self.half_field_height = self.field_height*0.5

        # ---- /Field parameters ----


        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('roboteam_rqt_view'), 'resource', 'WorldView.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('WorldView')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)


        # Subscribe to the world state.
        self._worldstate_sub = rospy.Subscriber("world_state", WorldMessage, self.worldstate_callback)

        # Subscribe to the geometry information.
        self._geometry_sub = rospy.Subscriber("vision_geometry", GeometryMessage, self.geometry_callback)

        # Connect the signal sent by the callback to the message slot.
        self._worldstate_signal.connect(self.worldstate_slot)

        # ---- Field view initialization ----

        self._scene = QGraphicsScene();
        self._fieldview = FieldGraphicsView()
        self._fieldview.setScene(self._scene)

        self._scene.setSceneRect(-self.half_field_width, -self.half_field_height, self.field_width, self.field_height)

        # Scale the scene so that it fits into the view area.
        self._fieldview.fitInView(self._scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

        # Add to the main window.
        self._widget.layout().addWidget(self._fieldview)

        self.reset_view()

        # ---- /Field view initialization ----

        self._font = QtGui.QFont()
        self._font.setPixelSize(BOT_DIAMETER*0.8)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self._worldstate_sub.unregister()
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
        self._scene.clear()

        # Draw the background.
        self._scene.addRect(-self.half_field_width, -self.half_field_width, self.field_width, self.field_height, pen=QtGui.QPen(), brush=QtGui.QBrush(QtCore.Qt.green))

        # Scale the scene so that it fits into the view area.
        self._fieldview.fitInView(self._scene.sceneRect(), QtCore.Qt.KeepAspectRatio)


    def worldstate_callback(self, message):
        # Send signal to qt thread.
        self._worldstate_signal.emit(message)


    # Receives the changeUI(PyQt_PyObject) signal which gets sent when a message arrives at 'message_callback'.
    def worldstate_slot(self, message):
        self.reset_view()

        # Draw the ball.
        self._scene.addEllipse(message.ball.pos.x, -(message.ball.pos.y), 50, 50, brush=QtGui.QBrush(QtGui.QColor(255, 100, 0)))

        # Draw the blue bots.
        for bot in message.robots_yellow:
            self.draw_robot(bot, QtGui.QColor(255, 255, 0))

        # Draw the yellow bots.
        for bot in message.robots_blue:
            self.draw_robot(bot, QtGui.QColor(0, 100, 255))


    def geometry_callback(self, message):
        self.field_width = message.field.field_width
        self.field_length = message.field.field_length

        self.half_field_width = self.field_width * 0.5
        self.half_field_length = self.field_length * 0.5

        rospy.loginfo("Field width: %i", self.field_width)
        rospy.loginfo("Field length: %i", self.field_length)


    # Draws a bot from a message, uses the color supplied.
    def draw_robot(self, bot, color):
        self._scene.addEllipse(
            bot.pos.x - BOT_DIAMETER/2, -(bot.pos.y - BOT_DIAMETER/2),
            BOT_DIAMETER, BOT_DIAMETER,
            brush=QtGui.QBrush(color)
            )
        line_pen = QtGui.QPen()
        line_pen.setWidth(10)
        rot_line = self._scene.addLine(0, 0, BOT_DIAMETER/2, 0, pen=line_pen)
        rot_line.setPos(bot.pos.x, -(bot.pos.y - BOT_DIAMETER))
        rot_line.setRotation(-math.degrees(bot.w))
        id_text = self._scene.addText(str(bot.id), self._font)
        id_text.setPos(bot.pos.x - BOT_DIAMETER*0.2, -(bot.pos.y - BOT_DIAMETER*0.5))
