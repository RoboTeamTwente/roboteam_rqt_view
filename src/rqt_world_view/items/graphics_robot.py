from python_qt_binding import QtGui
from python_qt_binding.QtWidgets import QGraphicsWidget, QGraphicsItem, QGraphicsItemGroup, QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsTextItem


BOT_DIAMETER = 180 # Diameter of the bots in mm.


class GraphicsRobot(QGraphicsItemGroup):

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

        # The part of the bot graphic that should rotate.
        self.rotator = QGraphicsItemGroup()
        self.addToGroup(self.rotator)

        # Make the bot selectable.
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)

        # Main robot body.
        ellipse = QGraphicsEllipseItem(-BOT_DIAMETER/2, -BOT_DIAMETER/2, BOT_DIAMETER, BOT_DIAMETER)
        ellipse.setBrush(QtGui.QBrush(color))
        self.rotator.addToGroup(ellipse)

        # Rotation line.
        line_pen = QtGui.QPen()
        line_pen.setWidth(10)
        rot_line = QGraphicsLineItem(0, 0, BOT_DIAMETER/2, 0)
        rot_line.setPen(line_pen)
        self.rotator.addToGroup(rot_line)

        # Bot id text.
        id_text = QGraphicsTextItem(str(self.bot_id))
        id_text.setFont(self.font)
        id_text.setDefaultTextColor(QtGui.QColor(220, 220, 220))
        id_text.setPos(-BOT_DIAMETER/3,-BOT_DIAMETER/2)
        self.addToGroup(id_text)

        # Selection circle.
        selection_pen = QtGui.QPen(QtGui.QColor(100, 255, 255))
        selection_pen.setWidth(10)
        # Pixels for the selection circle to be bigger than the bot.
        offset = BOT_DIAMETER * 0.3

        self.selector = QGraphicsEllipseItem(-((BOT_DIAMETER/2)+offset/2), -((BOT_DIAMETER/2)+offset/2), BOT_DIAMETER + offset, BOT_DIAMETER + offset)
        self.selector.setPen(selection_pen)
        self.selector.setVisible(False)
        self.addToGroup(self.selector)


    def rotate_to(self, angle):
        """
        Rotates the rotatable part of the bot to the supplied angle.
        Angle is in degrees.
        """
        self.rotator.setRotation(angle)


    def itemChange(self, change, value):
        """
        Is called when the item changes.
        Currently only listens for selection changes.
        """
        if change == QGraphicsWidget.ItemSelectedChange:
            if value == True:
                self.selector.setVisible(True)
            else:
                self.selector.setVisible(False)

        return QGraphicsItem.itemChange(self, change, value)


    def paint(self, painter, option, widget):
        """
        Overrides the default `paint` function.
        Doesn't paint anything.
        This removes the square selection border, which isn't needed because of the seleciton circle being handled in `itemChange`.
        No idea if this does anything else strange.
        If something is missing from a robot, its probably because of this function.
        Look here to only remove the selection line if this implementation turns out to do weird stuff:
        http://www.qtcentre.org/threads/15089-QGraphicsView-change-selected-rectangle-style
        """

        pass
