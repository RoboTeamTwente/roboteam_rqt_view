from python_qt_binding import QtGui, QtCore
from python_qt_binding.QtWidgets import QGraphicsItem

BOT_DIAMETER = 180 # Diameter of the bots in mm.
SELECTION_WIDTH = 15
BOT_BODY_RECT = QtCore.QRectF(-BOT_DIAMETER/2, -BOT_DIAMETER/2, BOT_DIAMETER, BOT_DIAMETER)
BOUNDING_RECT = QtCore.QRectF(-BOT_DIAMETER/2 - SELECTION_WIDTH/2, -BOT_DIAMETER/2 - SELECTION_WIDTH/2, BOT_DIAMETER + SELECTION_WIDTH, BOT_DIAMETER + SELECTION_WIDTH)

class GraphicsRobot(QGraphicsItem):

    def __init__(self, bot_id, is_us, color, font):
        """
        Creates a new GraphicsRobot.
        bot_id: integer -- The id to display on the robot.
        is_us: boolean -- Determines whether this robot belongs to our team.
           If it does, it's color is red and it will be selectable.
           Otherwhise it will be yellow and not selectable.
        color: QColor() -- The color to use for the robot body.
        font: QFont() -- The font to use to draw the id with.
        """
        super(GraphicsRobot, self).__init__()

        self.bot_id = bot_id
        self.is_us = is_us
        self.font = font

        self.bot_rotation = 0

        # Bot color fill brush.
        self.fill_brush = QtGui.QBrush(QtCore.Qt.SolidPattern)
        self.fill_brush.setColor(color)

        self.selection_pen = QtGui.QPen(QtGui.QColor(0, 255, 255))
        self.selection_pen.setWidth(SELECTION_WIDTH)

        # Make the bot selectable.
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)

        self.was_seen_last_update = True


    def rotate_to(self, angle):
        """
        Rotates the rotatable part of the bot to the supplied angle.
        Angle is in degrees.
        """
        self.bot_rotation = angle

    def boundingRect(self):
        return BOUNDING_RECT


    def itemChange(self, change, value):
        """
        Is called when the item changes.
        Currently only listens for selection changes.
        """
        if change == QGraphicsItem.ItemSelectedChange:
            pass

        return QGraphicsItem.itemChange(self, change, value)

    def set_was_seen_last_update(self, was_seen):
        self.was_seen_last_update = was_seen

    def was_seen_last_update(self):
        return self.was_seen_last_update


    def paint(self, painter, option, widget):
        painter.rotate(self.bot_rotation)
        if self.isSelected():
            painter.setPen(self.selection_pen)
        else:
            painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(self.fill_brush)
        # The `* 16` is here because drawChord wants it's angles in 16th of a degree.
        start_angle = 45 * 16
        span_angle = 270 * 16
        painter.drawChord(BOT_BODY_RECT, start_angle, span_angle)

        painter.rotate(-self.bot_rotation)
        painter.setPen(QtGui.QPen(QtCore.Qt.white))
        painter.setFont(self.font)

        painter.drawText(BOT_BODY_RECT, QtCore.Qt.AlignCenter, str(self.bot_id))
