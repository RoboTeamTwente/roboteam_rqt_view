from python_qt_binding import QtWidgets
from python_qt_binding import QtGui
from python_qt_binding import QtCore

CROSSHAIR_EXTENSION = 100
CROSSHAIR_PEN_WIDTH = 15

class ItemDebugPoint(QtWidgets.QGraphicsItemGroup):
    def __init__(self):
        super(ItemDebugPoint, self).__init__()

        self.pos_x = 0
        self.pos_y = 0
        self.left_edge = 0
        self.right_edge = 0
        self.top_edge = 0
        self.bottom_edge = 0

        self.pen = QtGui.QPen()
        self.pen.setWidth(CROSSHAIR_PEN_WIDTH)

    def set_pos(self, x, y):
        self.pos_x = x
        self.pos_y = y
        self.left_edge = x - CROSSHAIR_EXTENSION
        self.right_edge = x + CROSSHAIR_EXTENSION
        self.top_edge = y - CROSSHAIR_EXTENSION
        self.bottom_edge = y + CROSSHAIR_EXTENSION

    def set_color(self, color):
        self.pen.setColor(color)

    def paint(self, painter, option, widget):
        painter.setPen(self.pen)
        painter.drawLine(self.left_edge, self.pos_y, self.right_edge, self.pos_y)
        painter.drawLine(self.pos_x, self.top_edge, self.pos_x, self.bottom_edge)

    def boundingRect(self):
        return QtCore.QRectF(self.left_edge, self.top_edge, self.right_edge, self.bottom_edge)
