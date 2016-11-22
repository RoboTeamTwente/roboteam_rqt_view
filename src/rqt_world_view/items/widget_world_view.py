from python_qt_binding.QtWidgets import QGraphicsItem, QFrame, QHBoxLayout, QVBoxLayout, QGraphicsRectItem, QGraphicsItemGroup, QGraphicsEllipseItem, QGraphicsLineItem, QPushButton
from python_qt_binding import QtGui, QtCore

from field_graphics_view import FieldGraphicsView
from field_graphics_scene import FieldGraphicsScene

from rqt_world_view.utils import utils
from rqt_world_view.utils.grsim_connector import GrsimConnector
from graphics_robot import GraphicsRobot
from qgraphics_arc_item import QGraphicsArcItem

import math


BOT_DIAMETER = 180 # Diameter of the bots in mm.
BALL_DIAMETER = 50

# Size of the area around the field in mm.
FIELD_RUNOUT_ZONE = 300

FIELD_COLOR = QtGui.QColor(0, 200, 50)
FIELD_LINE_COLOR = QtGui.QColor(255, 255, 255)
BALL_COLOR = QtGui.QColor(255, 100, 0)


class WidgetWorldView(QFrame):
    """Displays the current world state."""


    def __init__(self, us_color, them_color):
        """
        us_color: QColor() -- The color to use for our robots.
        them_color: QColor() -- The color to use for their robots.
        """
        super(WidgetWorldView, self).__init__()

        # Field size in mm.
        self.field_width = 9000
        self.field_height = 6000

        self.us_color = us_color
        self.them_color = them_color

        self.field_background = QGraphicsRectItem(-self.field_width/2, -self.field_height/2, self.field_width, self.field_height)

        self.robots_us = {}
        self.robots_them = {}

        self.ball = QGraphicsEllipseItem(0, 0, BALL_DIAMETER, BALL_DIAMETER)
        self.ball.setBrush(QtGui.QBrush(BALL_COLOR))

        self.field_lines = QGraphicsItemGroup()

        self.goals = QGraphicsItemGroup()

        # Debug markers sent via the `view_debug_points` topic.
        self.debug_point_parent = QGraphicsItemGroup()
        self.debug_point_parent.setZValue(10)
        self.debug_points = {}

        self.debug_line_parent = QGraphicsItemGroup()
        self.debug_line_parent.setZValue(9)
        self.debug_lines = {}

        self.font = QtGui.QFont()
        self.font.setPixelSize(BOT_DIAMETER*0.8)


        self.setLayout(QVBoxLayout())

        # ---- Toolbar initialization ----

        self.toolbar = QFrame()
        self.toolbar.setLayout(QHBoxLayout())
        self.layout().addWidget(self.toolbar)

        self.out_of_field_button = QPushButton("Put all out of field")
        self.toolbar.layout().addWidget(self.out_of_field_button)
        self.out_of_field_button.clicked.connect(self.out_of_field_button_pressed)

        self.reset_view_button = QPushButton("Reset view")
        self.toolbar.layout().addWidget(self.reset_view_button)
        self.reset_view_button.clicked.connect(self.reset_view)

        # ---- /Toolbar initialization ----

        # ---- Field view initialization ----

        self.scene = FieldGraphicsScene();
        self.fieldview = FieldGraphicsView()
        self.fieldview.setScene(self.scene)
        self.layout().addWidget(self.fieldview)

        # Connect the scenes right clicks.
        self.scene.right_click_signal.connect(self.slot_scene_right_clicked)

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

        # Scale the scene so that it fits into the view area.
        self.fieldview.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

        # ---- /Field view initialization ----

        # Open a connection to grsim.
        self.grsim = GrsimConnector()


    def update_world_state(self, message):
        """
        Updates the world state.
        This includes robot and ball positions.
        Expects a DetectionFrame message.
        """
        # Move the ball.
        self.ball.setPos(utils.m_to_mm(message.ball.pos.x) - BALL_DIAMETER/2, -utils.m_to_mm(message.ball.pos.y) - BALL_DIAMETER/2)

        # Process the us bots.
        for bot in message.us:
            if not bot.id in self.robots_us:
                # Create a graphics item for this robot.
                self.robots_us[bot.id] = GraphicsRobot(bot.id, True, self.us_color, self.font)
                self.scene.addItem(self.robots_us[bot.id])

            self.robots_us[bot.id].setPos(utils.m_to_mm(bot.pos.x), -utils.m_to_mm(bot.pos.y))
            self.robots_us[bot.id].rotate_to(-math.degrees(bot.angle))

        # Process the them bots.
        for bot in message.them:
            if not bot.id in self.robots_them:
                self.robots_them[bot.id] = GraphicsRobot(bot.id, False, self.them_color, self.font)
                self.scene.addItem(self.robots_them[bot.id])

            self.robots_them[bot.id].setPos(utils.m_to_mm(bot.pos.x), -utils.m_to_mm(bot.pos.y))
            self.robots_them[bot.id].rotate_to(-math.degrees(bot.angle))


    def update_field_configuration(self, message):
        """
        Updates the field configuration.
        Field size, lines, goal positions etc.
        Expects a GeometryFieldSize message.
        """
        self.field_width = utils.m_to_mm(message.field.field_width)
        self.field_length = utils.m_to_mm(message.field.field_length)

        # Resize the field background.
        self.field_background.setRect(
            -(self.field_length/2 + FIELD_RUNOUT_ZONE), -(self.field_width/2 + FIELD_RUNOUT_ZONE),
            self.field_length + FIELD_RUNOUT_ZONE * 2, self.field_width + FIELD_RUNOUT_ZONE * 2)

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
                utils.m_to_mm(msg_line.x_begin), -utils.m_to_mm(msg_line.y_begin),
                utils.m_to_mm(msg_line.x_end), -utils.m_to_mm(msg_line.y_end))
            line.setPen(line_pen)
            self.field_lines.addToGroup(line)

        # Add all the arcs.
        for msg_arc in message.field.field_arcs:
            line_pen = QtGui.QPen()
            line_pen.setColor(FIELD_LINE_COLOR)
            line_pen.setWidth(utils.m_to_mm(msg_line.thickness))

            arc = QGraphicsArcItem(
                utils.m_to_mm(msg_arc.x_center - msg_arc.radius), -utils.m_to_mm(msg_arc.y_center - msg_arc.radius),
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
                self.add_debug_point(point)
        else:
            for item in self.debug_points[point.name].childItems():
                self.debug_points[point.name].removeFromGroup(item)
                self.scene.removeItem(item)
            del self.debug_points[point.name]

            if not point.remove:
                self.add_debug_point(point)

    def add_debug_point(self, point):
        line_pen = QtGui.QPen()
        line_pen.setColor(QtGui.QColor(point.color.r, point.color.g, point.color.b))
        line_pen.setWidth(15)

        size = 100

        crosshair = QGraphicsItemGroup()
        crosshair.setParentItem(self.debug_point_parent)

        # Create a crosshair at the indicated location.
        h_line = QGraphicsLineItem(
            -size, 0,
            size, 0)
        h_line.setPen(line_pen)
        crosshair.addToGroup(h_line)

        v_line = QGraphicsLineItem(
            0, -size,
            0, size)
        v_line.setPen(line_pen)
        crosshair.addToGroup(v_line)

        self.debug_points[point.name] = crosshair
        crosshair.setPos(utils.m_to_mm(point.pos.x), -utils.m_to_mm(point.pos.y))


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

        pos_x = event.scenePos().x()
        pos_y = -event.scenePos().y()

        placed_a_robot = False

        for bot_id, robot in self.robots_us.iteritems():
            if robot.isSelected():
                self.grsim.place_robot(bot_id, True, pos_x, pos_y)
                placed_a_robot = True

        for bot_id, robot in self.robots_them.iteritems():
            if robot.isSelected():
                self.grsim.place_robot(bot_id, False, pos_x, pos_y)
                placed_a_robot = True

        if not placed_a_robot:
            self.grsim.place_ball(pos_x, pos_y)


    # --------------------------------------------------------------------------
    # ---- Toolbar slots -------------------------------------------------------
    # --------------------------------------------------------------------------

    def out_of_field_button_pressed(self):
        """Places all the robots outside the field."""
        for bot_id, robot in self.robots_us.iteritems():
            x = (BOT_DIAMETER * 2 * (bot_id + 1))
            y = - self.field_width/2 - FIELD_RUNOUT_ZONE - BOT_DIAMETER

            self.grsim.place_robot(bot_id, True, x, y)

        for bot_id, robot in self.robots_them.iteritems():
            x = -(BOT_DIAMETER * 2 * (bot_id + 1))
            y = - self.field_width/2 - FIELD_RUNOUT_ZONE - BOT_DIAMETER

            self.grsim.place_robot(bot_id, False, x, y)


    def reset_view(self):
        self.fieldview.fitInView(self.field_background, QtCore.Qt.KeepAspectRatio)
