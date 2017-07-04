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
        self.prepareGeometryChange()
        self.pos_x = int(x)
        self.pos_y = int(y)
        self.setX(self.pos_x)
        self.setY(self.pos_y)
        self.left_edge = int(x - CROSSHAIR_EXTENSION)
        self.right_edge = int(x + CROSSHAIR_EXTENSION)
        self.top_edge = int(y - CROSSHAIR_EXTENSION)
        self.bottom_edge = int(y + CROSSHAIR_EXTENSION)
        self.width = self.right_edge - self.left_edge
        self.height = self.bottom_edge - self.top_edge

    def set_color(self, color):
        self.pen.setColor(color)

    def paint(self, painter, option, widget):
        painter.setPen(self.pen)
        painter.drawLine(-CROSSHAIR_EXTENSION, 0, CROSSHAIR_EXTENSION, 0)
        painter.drawLine(0, CROSSHAIR_EXTENSION, 0, -CROSSHAIR_EXTENSION)

    def boundingRect(self):
        return QtCore.QRectF(-self.width/2, -self.height/2, self.width, self.height)
