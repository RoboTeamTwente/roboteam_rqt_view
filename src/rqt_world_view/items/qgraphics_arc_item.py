from python_qt_binding.QtWidgets import QGraphicsEllipseItem


class QGraphicsArcItem(QGraphicsEllipseItem):
    """
    Arc item.
    Works almost the same as ellipse item.
    Only instead of drawing a pie arc,
    it draws only the outside line of the arc.
    """

    def __init__(self, x_center, y_center, width, height):
        super(QGraphicsArcItem, self).__init__(x_center, y_center, width, height)


    def paint(self, painter, option, widget):
        painter.setPen(self.pen())
        painter.setBrush(self.brush())
        painter.drawArc(self.rect(), self.startAngle(), self.spanAngle())
