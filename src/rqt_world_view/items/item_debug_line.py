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
        self.right_edge = 0
        self.width = 0
        self.height = 0

        self.pen = QtGui.QPen()
        self.pen.setWidth(CROSSHAIR_PEN_WIDTH)

    def set_line(self, start_x, start_y, end_x, end_y):
        self.start_x = int(start_x)
        self.start_y = int(start_y)
        self.stop_x = int(end_x)
        self.stop_y = int(end_y)
        self.left_edge = int(min(start_x, stop_x))
        self.right_edge = int(min(start_y, stop_y))
        self.width = int(abs(start_x - stop_x))
        self.height = int(abs(start_y - stop_y))

    def set_color(self, color):
        self.pen.setColor(color)

    def paint(self, painter, option, widget):
        painter.setPen(self.pen)
        painter.drawLine(self.start_x, self.stop_x, self.start_y, self.stop_y)

    def boundingRect(self):
        return QtCore.QRectF(self.left_edge, self.top_edge, self.width, self.height)
