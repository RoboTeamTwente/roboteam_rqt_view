import sys

from python_qt_binding.QtWidgets import QApplication, QGraphicsItem, QFrame, QHBoxLayout, QVBoxLayout, QGraphicsRectItem, QGraphicsItemGroup, QGraphicsEllipseItem, QGraphicsLineItem, QPushButton, QLabel, QAction, QShortcut
from python_qt_binding import QtGui, QtCore
from python_qt_binding.QtCore import pyqtSignal

from field_graphics_view import FieldGraphicsView
from field_graphics_scene import FieldGraphicsScene

from rqt_world_view.utils import utils
from rqt_world_view.utils.grsim_connector import GrsimConnector
from graphics_robot import GraphicsRobot
from qgraphics_arc_item import QGraphicsArcItem
from item_debug_point import ItemDebugPoint

from std_msgs import msg as std_msg

import math

import rospy


BOT_DIAMETER = 180 # Diameter of the bots in mm.
BALL_DIAMETER = 50
BALL_CROSSHAIR_SPACING = 100 # Distance from the ball to the ball crosshair.

FIELD_COLOR = QtGui.QColor(0, 200, 50)
FIELD_LINE_COLOR = QtGui.QColor(255, 255, 255)
BALL_COLOR = QtGui.QColor(255, 100, 0)


class WidgetWorldView(QFrame):
    """Displays the current world state."""

    def __init__(self, us_color, them_color, halt_pub):
        """
        us_color: QColor() -- The color to use for our robots.
        them_color: QColor() -- The color to use for their robots.
        """
        super(WidgetWorldView, self).__init__()

        # Stuff needed to halt properly
        self.halt_pub = halt_pub
        self.is_halting = False

        # Field size in mm.
        self.field_width = 9000
        self.field_length = 6000
        self.field_boundary = 300

        self.us_color = us_color
        self.them_color = them_color

        self.field_normalized = False

        # Mouse position on the field.
        self.field_mouse_x = 0
        self.field_mouse_y = 0


        self.field_background = QGraphicsRectItem(-self.field_width/2, -self.field_length/2, self.field_width, self.field_length)
        self.field_background.setZValue(-10)

        self.robots_us = {}
        self.robots_them = {}

        self.ball = QGraphicsEllipseItem(0, 0, BALL_DIAMETER, BALL_DIAMETER)
        self.ball.setBrush(QtGui.QBrush(BALL_COLOR))

        self.field_lines = QGraphicsItemGroup()
        self.field_lines.setZValue(-5)

        self.goals = QGraphicsItemGroup()
        self.field_lines.setZValue(-4)

        # Debug markers sent via the `view_debug_points` topic.
        self.debug_point_parent = QGraphicsItemGroup()
        self.debug_point_parent.setZValue(10)
        self.debug_points = {}

        self.debug_line_parent = QGraphicsItemGroup()
        self.debug_line_parent.setZValue(9)
        self.debug_lines = {}

        self.ball_crosshair_pen = QtGui.QPen()
        self.ball_crosshair_pen.setColor(QtGui.QColor(0, 0, 0, 50))
        self.ball_crosshair_pen.setWidth(15)

        self.ball_crosshair_parent = QGraphicsItemGroup()
        self.ball_crosshair_parent.setZValue(0)
        self.ball_top_line = QGraphicsLineItem()
        self.ball_right_line = QGraphicsLineItem()
        self.ball_bottom_line = QGraphicsLineItem()
        self.ball_left_line = QGraphicsLineItem()

        self.ball_top_line.setPen(self.ball_crosshair_pen)
        self.ball_right_line.setPen(self.ball_crosshair_pen)
        self.ball_bottom_line.setPen(self.ball_crosshair_pen)
        self.ball_left_line.setPen(self.ball_crosshair_pen)

        self.ball_crosshair_parent.addToGroup(self.ball_top_line)
        self.ball_crosshair_parent.addToGroup(self.ball_right_line)
        self.ball_crosshair_parent.addToGroup(self.ball_bottom_line)
        self.ball_crosshair_parent.addToGroup(self.ball_left_line)

        self.font = QtGui.QFont()
        self.font.setPixelSize(BOT_DIAMETER*0.8)

        self.setLayout(QVBoxLayout())

        self.setContentsMargins(0, 0, 0, 0)

        # ---- Toolbar initialization ----

        self.toolbar = QFrame()
        self.toolbar.setLayout(QHBoxLayout())
        self.toolbar.setContentsMargins(0, 0, 0, 0)
        self.layout().addWidget(self.toolbar)

        self.vision_status_indicator = QLabel("Vision status")
        self.vision_status_indicator.setAlignment(QtCore.Qt.AlignCenter)
        self.toolbar.layout().addWidget(self.vision_status_indicator)

        self.out_of_field_button = QPushButton("Put all out of field")
        self.toolbar.layout().addWidget(self.out_of_field_button)
        self.out_of_field_button.clicked.connect(self.out_of_field_button_pressed)

        self.reset_view_button = QPushButton("Reset view")
        self.toolbar.layout().addWidget(self.reset_view_button)
        self.reset_view_button.clicked.connect(self.reset_view)

        self.clear_debug_button = QPushButton("Clear debug drawings")
        self.toolbar.layout().addWidget(self.clear_debug_button)
        self.clear_debug_button.clicked.connect(self.clear_debug_drawings)

        self.toggle_halt_button = QPushButton("Press to halt")
        self.toolbar.layout().addWidget(self.toggle_halt_button)
        self.toggle_halt_button.clicked.connect(self.toggle_halt)

        # ---- /Toolbar initialization ----

        # ---- Field view initialization ----

        self.scene = FieldGraphicsScene();
        self.fieldview = FieldGraphicsView()
        self.fieldview.setScene(self.scene)
        self.layout().addWidget(self.fieldview)

        # Connect the scenes right clicks.
        self.scene.right_click_signal.connect(self.slot_scene_right_clicked)
        self.scene.left_click_signal.connect(self.slot_scene_left_clicked)

        # Enable antialiasing.
        self.fieldview.setRenderHints(QtGui.QPainter.Antialiasing)
        self.scene.clear()

        # Add the field to the scene.
        self.scene.addItem(self.field_background)
        self.field_background.setBrush(QtGui.QBrush(FIELD_COLOR))

        # Add the field lines.
        self.scene.addItem(self.field_lines)

        # Add the goals.
        self.scene.addItem(self.goals)

        # Add the ball to the scene.
        self.scene.addItem(self.ball)

        # Add the debug points to the scene.
        self.scene.addItem(self.debug_point_parent)
        self.scene.addItem(self.debug_line_parent)

        # Add the ball crosshair.
        self.scene.addItem(self.ball_crosshair_parent)

        # Scale the scene so that it fits into the view area.
        self.fieldview.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

        # ---- /Field view initialization ----

        # ---- Info labels ----

        self.info_layout = QHBoxLayout()
        self.layout().addLayout(self.info_layout)

        self.cursor_info_label = QLabel("0, 0")
        self.cursor_info_label.setMinimumWidth(100)
        self.info_layout.addWidget(self.cursor_info_label)

        self.copy_instruction_label = QLabel("Double click field to copy coordinates.")
        self.info_layout.addWidget(self.copy_instruction_label)

        self.info_layout.addStretch(1)

        # ---- /Info labels ----

        # Start tracking the mouse.
        self.scene.mouse_move_signal.connect(self.slot_scene_mouse_moved)
        self.fieldview.setMouseTracking(True)

        # Open a connection to grsim.
        self.grsim = GrsimConnector()

        # Process handle for the TestX programm.
        self.testx_thread = None

        # Ui update timer.
        self.ui_update_timer = QtCore.QTimer()
        self.ui_update_timer.timeout.connect(self.update_scene)
        self.ui_update_timer.start(100) # Update every x millisecconds.


    def update_scene(self):
        self.scene.update(self.fieldview.sceneRect())


    def update_world_state(self, message):
        """
        Updates the world state.
        This includes robot and ball positions.
        Expects a DetectionFrame message.
        """
        ball_x = utils.m_to_mm(message.ball.pos.x)
        ball_y = -utils.m_to_mm(message.ball.pos.y)

        # Move the ball.
        self.ball.setPos(ball_x - BALL_DIAMETER/2, ball_y - BALL_DIAMETER/2)

        # Move the ball croshairs.
        self.ball_top_line.setLine(ball_x, -self.field_width/2, ball_x, ball_y - BALL_CROSSHAIR_SPACING)
        self.ball_right_line.setLine(ball_x + BALL_CROSSHAIR_SPACING, ball_y, self.field_length/2, ball_y)
        self.ball_bottom_line.setLine(ball_x, self.field_width/2, ball_x, ball_y + BALL_CROSSHAIR_SPACING)
        self.ball_left_line.setLine(ball_x - BALL_CROSSHAIR_SPACING, ball_y, -self.field_length/2, ball_y)

        # Process the us bots.
        for bot in message.us:
            if not bot.id in self.robots_us:
                # Create a graphics item for this robot.
                self.robots_us[bot.id] = GraphicsRobot(bot.id, True, self.us_color, self.font)
                self.scene.addItem(self.robots_us[bot.id])

            self.robots_us[bot.id].setPos(utils.m_to_mm(bot.pos.x), -utils.m_to_mm(bot.pos.y))
            self.robots_us[bot.id].rotate_to(-math.degrees(bot.angle))
            self.robots_us[bot.id].set_was_seen_last_update(True)

        for bot_id in self.robots_us.keys():
            if self.robots_us[bot_id].was_seen_last_update:
                self.robots_us[bot_id].set_was_seen_last_update(False)
            else:
                # Old robot, remove it.
                self.scene.removeItem(self.robots_us[bot_id])
                del self.robots_us[bot_id]

        # Process the them bots.
        for bot in message.them:
            if not bot.id in self.robots_them:
                self.robots_them[bot.id] = GraphicsRobot(bot.id, False, self.them_color, self.font)
                self.scene.addItem(self.robots_them[bot.id])

            self.robots_them[bot.id].setPos(utils.m_to_mm(bot.pos.x), -utils.m_to_mm(bot.pos.y))
            self.robots_them[bot.id].rotate_to(-math.degrees(bot.angle))
            self.robots_them[bot.id].set_was_seen_last_update(True)

        for bot_id in self.robots_them.keys():
            if self.robots_them[bot_id].was_seen_last_update:
                self.robots_them[bot_id].set_was_seen_last_update(False)
            else:
                # Old robot, remove it.
                self.scene.removeItem(self.robots_them[bot_id])
                del self.robots_them[bot_id]


    def update_field_configuration(self, message):
        """
        Updates the field configuration.
        Field size, lines, goal positions etc.
        Expects a GeometryFieldSize message.
        """

        # Check for field normalization.
        if rospy.has_param("normalize_field") and rospy.has_param("our_side"):
            normalize_field = rospy.get_param("normalize_field")
            our_side = rospy.get_param("our_side")

            if normalize_field and our_side == "right":
                self.field_normalized = True
            else:
                self.field_normalized = False

        self.field_width = utils.m_to_mm(message.field.field_width)
        self.field_length = utils.m_to_mm(message.field.field_length)
        self.field_boundary = utils.m_to_mm(message.field.boundary_width)

        # Resize the field background.
        self.field_background.setRect(
            -(self.field_length/2 + self.field_boundary), -(self.field_width/2 + self.field_boundary),
            self.field_length + self.field_boundary * 2, self.field_width + self.field_boundary * 2)

        # Remove all field lines.
        for item in self.field_lines.childItems():
            self.field_lines.removeFromGroup(item)
            del item

        # Remove all goals.
        for item in self.goals.childItems():
            self.goals.removeFromGroup(item)
            del item


        # Add all the lines.
        for msg_line in message.field.field_lines:
            line_pen = QtGui.QPen()
            line_pen.setColor(FIELD_LINE_COLOR)
            line_pen.setWidth(utils.m_to_mm(msg_line.thickness))

            line = QGraphicsLineItem(
                utils.m_to_mm(msg_line.begin.x), -utils.m_to_mm(msg_line.begin.y),
                utils.m_to_mm(msg_line.end.x), -utils.m_to_mm(msg_line.end.y))
            line.setPen(line_pen)
            self.field_lines.addToGroup(line)

        # Add all the arcs.
        for msg_arc in message.field.field_arcs:
            line_pen = QtGui.QPen()
            line_pen.setColor(FIELD_LINE_COLOR)
            line_pen.setWidth(utils.m_to_mm(msg_line.thickness))

            arc = QGraphicsArcItem(
                utils.m_to_mm(msg_arc.center.x - msg_arc.radius), -utils.m_to_mm(msg_arc.center.y - msg_arc.radius),
                utils.m_to_mm(msg_arc.radius)*2, -utils.m_to_mm(msg_arc.radius)*2)
            arc.setStartAngle(math.degrees(msg_arc.a1)*16)
            arc.setSpanAngle(math.degrees(msg_arc.a2 - msg_arc.a1)*16)
            arc.setPen(line_pen)
            self.field_lines.addToGroup(arc)

        # Create the goals.
        goal_width = utils.m_to_mm(message.field.goal_width)
        goal_depth = utils.m_to_mm(message.field.goal_depth)

        goal_x_from_center = self.field_length/2 + goal_depth
        goal_y_from_center = goal_width/2

        goal_pen = QtGui.QPen()
        goal_pen.setWidth(goal_depth / 20)

        # Left goal.

        back_line = QGraphicsLineItem(
            -goal_x_from_center, goal_y_from_center,
            -goal_x_from_center, -goal_y_from_center)
        back_line.setPen(goal_pen)
        self.goals.addToGroup(back_line)

        top_line = QGraphicsLineItem(
            -goal_x_from_center, goal_y_from_center,
            -self.field_length/2, goal_y_from_center)
        top_line.setPen(goal_pen)
        self.goals.addToGroup(top_line)

        bottom_line = QGraphicsLineItem(
            -goal_x_from_center, -goal_y_from_center,
            -self.field_length/2, -goal_y_from_center)
        bottom_line.setPen(goal_pen)
        self.goals.addToGroup(bottom_line)

        # Right goal.

        back_line = QGraphicsLineItem(
            goal_x_from_center, goal_y_from_center,
            goal_x_from_center, -goal_y_from_center)
        back_line.setPen(goal_pen)
        self.goals.addToGroup(back_line)

        top_line = QGraphicsLineItem(
            goal_x_from_center, goal_y_from_center,
            self.field_length/2, goal_y_from_center)
        top_line.setPen(goal_pen)
        self.goals.addToGroup(top_line)

        bottom_line = QGraphicsLineItem(
            goal_x_from_center, -goal_y_from_center,
            self.field_length/2, -goal_y_from_center)
        bottom_line.setPen(goal_pen)
        self.goals.addToGroup(bottom_line)


        # Reset the graphicsscene's size.
        self.scene.setSceneRect(self.scene.itemsBoundingRect());


    # Slot for the scenes selectionChanged signal.
    # def slot_selection_changed(self):
    #     # Clear the selection lists.
    #     del self.robots_us_selected[:]
    #
    #     for bot_id, bot in self.robots_us_graphic.iteritems():
    #         if bot.isSelected():
    #             self.robots_us_selected.append(bot_id)


    def set_debug_point(self, point):
        """Expects a `roboteam_msgs.DebugPoint` point."""
        if not point.name in self.debug_points:
            if not point.remove:
                new_point = ItemDebugPoint()
                point_color = QtGui.QColor(point.color.r, point.color.g, point.color.b)
                new_point.set_color(point_color)
                new_point.set_pos(utils.m_to_mm(point.pos.x), utils.m_to_mm(point.pos.y))
                new_point.setParentItem(self.debug_point_parent)
                self.debug_points[point.name] = new_point
        else:
            if not point.remove:
                point_color = QtGui.QColor(point.color.r, point.color.g, point.color.b)
                point_item = self.debug_points[point.name]
                point_item.set_color(point_color)
                point_item.set_pos(utils.m_to_mm(point.pos.x), utils.m_to_mm(point.pos.y))
            else:
                self.scene.removeItem(self.debug_points[point.name])
                del self.debug_points[point.name]


    def set_debug_line(self, line):
        """Expects a `roboteam_msgs.DebugLine` point."""

        if not line.name in self.debug_lines:
            if not line.remove:
                self.add_debug_line(line)
        else:
            # First remove the line.
            for item in self.debug_lines[line.name].childItems():
                self.debug_lines[line.name].removeFromGroup(item)
                self.scene.removeItem(item)
            del self.debug_lines[line.name]

            # Then add the new version if necessary.
            if not line.remove:
                self.add_debug_line(line)


    def add_debug_line(self, line):
        line_group = QGraphicsItemGroup()
        line_group.setParentItem(self.debug_line_parent)

        line_pen = QtGui.QPen()
        line_pen.setColor(QtGui.QColor(line.color.r, line.color.g, line.color.b))
        line_pen.setWidth(15)

        last_point = None

        for i, point in enumerate(line.points):
            if i > 0:
                # Add all segments.
                segment = QGraphicsLineItem(
                    utils.m_to_mm(point.x), -utils.m_to_mm(point.y),
                    utils.m_to_mm(last_point.x), -utils.m_to_mm(last_point.y))
                segment.setPen(line_pen)
                line_group.addToGroup(segment)

            last_point = point

        self.debug_lines[line.name] = line_group


    def slot_scene_right_clicked(self, event):
        """To be called when the field scene is right clicked."""

        normalize_value = 1

        if self.field_normalized:
            normalize_value = -1

        pos_x = event.scenePos().x() * normalize_value
        pos_y = -(event.scenePos().y() * normalize_value)

        selected_robot = -1

        # Get the keyboard modifiers.
        key_modifiers = QApplication.keyboardModifiers()
        shift_pressed = key_modifiers & QtCore.Qt.ShiftModifier

        for bot_id, robot in self.robots_us.iteritems():
            if robot.isSelected():
                selected_robot = bot_id

                if not shift_pressed:
                    # Send a command to grsim to teleport the robot.
                    self.grsim.place_robot(selected_robot, True, pos_x, pos_y)
                else:
                    # Start GoToPos with the selected robot.

                    if self.is_test_running():
                        # Ask the testx thread to stop.
                        self.testx_thread.stop()

                    command = ["rosrun", "roboteam_tactics", "TestX", "GoToPos"]
                    command.append("int:ROBOT_ID=" + str(selected_robot))
                    # Positions need to be re-normalized. Because the test skills are expecting normalized positions.
                    command.append("double:xGoal=" + str( utils.mm_to_m(pos_x * normalize_value) ))
                    command.append("double:yGoal=" + str( utils.mm_to_m(pos_y * normalize_value) ))
                    print command

                    # Start the test.
                    self.testx_thread = utils.popen_and_call(self.on_exit, command, stdout=sys.stdout)


        for bot_id, robot in self.robots_them.iteritems():
            if robot.isSelected():
                selected_robot = bot_id

                if not shift_pressed:
                    # Send a command to grsim to teleport the robot.
                    self.grsim.place_robot(selected_robot, False, pos_x, pos_y)

        if selected_robot == -1:
            # No robot was selected, so move the ball.
            self.grsim.place_ball(pos_x, pos_y)


    def slot_scene_left_clicked(self, event):
        """To be called when the field scene is left clicked."""
        pass

    def on_exit(self):
        return

    def is_test_running(self):
        """
        Checks whether the testx process is running.
        Returns true for running, false for not running or nonexistent.
        """
        if self.testx_thread:
            if self.testx_thread.isAlive():
                # The test is still running.
                return True
        # Fell through, test is not running.
        return False


    def slot_scene_mouse_moved(self, event):

        normalize_value = 1

        if self.field_normalized:
            normalize_value = -1

        self.field_mouse_x = round(utils.mm_to_m(event.scenePos().x() * normalize_value), 2)
        self.field_mouse_y = round(utils.mm_to_m(-(event.scenePos().y() * normalize_value)), 2)

        self.cursor_info_label.setText(
            str(self.field_mouse_x) + ", " +
            str(self.field_mouse_y)
            )


    def slot_copy_coordinates_to_clipboard(self):
        QApplication.clipboard().setText(
            (str(self.field_mouse_x) + ", " +
            str(self.field_mouse_y))
            )

    def mouseDoubleClickEvent(self, event):
        self.slot_copy_coordinates_to_clipboard()

    # --------------------------------------------------------------------------
    # ---- Toolbar slots -------------------------------------------------------
    # --------------------------------------------------------------------------

    def out_of_field_button_pressed(self):
        """Places all the robots outside the field."""
        for bot_id, robot in self.robots_us.iteritems():
            x = (BOT_DIAMETER * 2 * (bot_id + 1))
            y = - self.field_width/2 - self.field_boundary - BOT_DIAMETER

            self.grsim.place_robot(bot_id, True, x, y)

        for bot_id, robot in self.robots_them.iteritems():
            x = -(BOT_DIAMETER * 2 * (bot_id + 1))
            y = - self.field_width/2 - self.field_boundary - BOT_DIAMETER

            self.grsim.place_robot(bot_id, False, x, y)


    def reset_view(self):
        self.fieldview.fitInView(self.field_background, QtCore.Qt.KeepAspectRatio)


    def clear_debug_drawings(self):
        for name in self.debug_points.keys():
            self.scene.removeItem(self.debug_points[name])
            del self.debug_points[name]

        for name in self.debug_lines.keys():
            for item in self.debug_lines[name].childItems():
                self.debug_lines[name].removeFromGroup(item)
                self.scene.removeItem(item)
            del self.debug_lines[name]

    def halt_update(self, message):
        """
        Called when something updates the halt state. Maks the halt button red/ordinary and
        changes the text as well.
        """
        self.is_halting = message.data

        if self.is_halting:
            self.toggle_halt_button.setStyleSheet('QPushButton {background-color: #FF0000;}')
            self.toggle_halt_button.setText("Halting")
        else:
            self.toggle_halt_button.setStyleSheet('QPushButton {}')
            self.toggle_halt_button.setText("Not halting")

    def toggle_halt(self):
        """
        Called when the halt button is pressed. Sends an opposite halt command of the currently
        known halt state.
        """
        message = std_msg.Bool()
        message.data = not self.is_halting

        self.halt_pub.publish(message)


    def set_vision_status_indicator(self, status):
        if status == True:
            self.vision_status_indicator.setStyleSheet("color: #00aa00")
        else:
            self.vision_status_indicator.setStyleSheet("color: #FF0000")
