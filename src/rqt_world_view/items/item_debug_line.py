from python_qt_binding import QtWidgets
from python_qt_binding import QtGui
from python_qt_binding import QtCore

LINE_PEN_WIDTH = 15

class ItemDebugLine(QtWidgets.QGraphicsItemGroup):
    def __init__(self):
        super(ItemDebugLine, self).__init__()

        self.start_x = 0
        self.start_y = 0
        self.stop_x = 0
        self.stop_y = 0
        self.left_edge = 0
        self.top_edge = 0
        self.width = 0
        self.height = 0

        self.pen = QtGui.QPen()
        self.pen.setWidth(LINE_PEN_WIDTH)

    def set_line(self, start_x, start_y, stop_x, stop_y):
        self.prepareGeometryChange()
        self.setX((stop_x - start_x) / 2 + start_x)
        self.setY((stop_y - start_y) / 2 + start_y)
        self.start_x = int(start_x)
        self.start_y = int(start_y)
        self.stop_x = int(stop_x)
        self.stop_y = int(stop_y)
        self.left_edge = int(min(start_x, stop_x)) - LINE_PEN_WIDTH/2
        self.top_edge = int(min(start_y, stop_y)) - LINE_PEN_WIDTH/2
        self.width = int(abs(start_x - stop_x)) + LINE_PEN_WIDTH
        self.height = int(abs(start_y - stop_y)) + LINE_PEN_WIDTH

    def set_color(self, color):
        self.pen.setColor(color)

    def paint(self, painter, option, widget):
        painter.setPen(self.pen)
        painter.drawLine(self.start_x - self.x(), self.start_y - self.y(), self.stop_x - self.x(), self.stop_y - self.y())

    def boundingRect(self):
        return QtCore.QRectF(-self.width/2, -self.height/2, self.width, self.height)
