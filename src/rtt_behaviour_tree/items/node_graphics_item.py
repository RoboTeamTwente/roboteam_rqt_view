from python_qt_binding import QtWidgets, QtGui, QtCore

from rtt_behaviour_tree.utils import vector2

from roboteam_msgs import msg


DEFAULT_BODY_COL = QtGui.QColor(255, 255, 255)
DEFAULT_LINE_COL = QtGui.QColor(0, 0, 0)

STATE_BODY_COL_MAP = {
    msg.BtStatus.STARTUP: QtGui.QColor(200, 200, 255),
    msg.BtStatus.SUCCESS: QtGui.QColor(200, 255, 200),
    msg.BtStatus.FAILURE: QtGui.QColor(255, 200, 200),
    msg.BtStatus.INVALID: QtGui.QColor(255, 200, 255)
}

STATE_FLASH_COL_MAP = {
    msg.BtStatus.STARTUP: QtGui.QColor(150, 150, 255),
    msg.BtStatus.SUCCESS: QtGui.QColor(150, 255, 150),
    msg.BtStatus.FAILURE: QtGui.QColor(255, 150, 150),
    msg.BtStatus.INVALID: QtGui.QColor(255, 150, 255)
}

STATE_LINE_COL_MAP = {
    msg.BtStatus.STARTUP: QtGui.QColor(0, 0, 255),
    msg.BtStatus.SUCCESS: QtGui.QColor(0, 255, 0),
    msg.BtStatus.FAILURE: QtGui.QColor(255, 0, 0),
    msg.BtStatus.INVALID: QtGui.QColor(255, 0, 255)
}


class NodeGraphicsItem(QtWidgets.QGraphicsWidget):

    def __init__(self, node_reference):
        """
        The `node_reference` should be a reference to a `tree.Node` object.
        """
        super(NodeGraphicsItem, self).__init__()

        if node_reference:

            self._node = node_reference

            self.node_body = QtWidgets.QGraphicsItemGroup(parent=self)
            # Make the body dragable.
            # self.node_body.setFlag(QtWidgets.QGraphicsItemGroup.ItemIsMovable, True)

            self.size = vector2.Vector2(100, 30)

            if self._node.type.category.name == 'condition':
                self.body = QtWidgets.QGraphicsEllipseItem(0, 0, self.size.x, self.size.y)
            else:
                self.body = QtWidgets.QGraphicsRectItem(0, 0, self.size.x, self.size.y)

            self.node_body.addToGroup(self.body)
            self.text = QtWidgets.QGraphicsTextItem("")
            self.node_body.addToGroup(self.text)

            self.connectors = []

            self.body_pen = QtGui.QPen()
            self.body_brush = QtGui.QBrush()

            self.update()


    def update(self):

        self.setPos(self._node.display.x, self._node.display.y)

        text_string = self._node.title()

        # See if the node has an overriding custom icon.
        if self._node.type.has_custom_icon():
            text_string = self._node.type.custom_icon

        self.text.setPlainText(text_string)

        self.size.x = self.text.boundingRect().width()
        self.size.y = self.text.boundingRect().height()

        self.body.setRect(0, -self.size.y/2, self.size.x, self.size.y)
        self.text.setPos(0, -self.size.y/2)

        status = self._node.status()

        self.body_brush.setStyle(QtCore.Qt.SolidPattern)

        self.body_brush.setColor(STATE_BODY_COL_MAP.get(status, DEFAULT_BODY_COL))
        self.body_pen.setColor(STATE_LINE_COL_MAP.get(status, DEFAULT_LINE_COL))

        self.body.setPen(self.body_pen)
        self.body.setBrush(self.body_brush)


        del self.connectors[:]

        for child in self._node.children():
            path = QtGui.QPainterPath(QtCore.QPointF(
                self.size.x, 0))
            path.lineTo(child.display.x - self._node.display.x, child.display.y - self._node.display.y)

            connector = QtWidgets.QGraphicsPathItem(path)
            connector.setParentItem(self)
            self.connectors.append(connector)


    def flash(self):
        """
        Flashes to indicate activation.
        """
        status = self._node.status()

        self.animation = anim = QtCore.QPropertyAnimation(self, "bodycolor", self)
        anim.setDuration(500)
        anim.setStartValue(STATE_FLASH_COL_MAP.get(status, QtGui.QColor(255, 255, 255)))
        anim.setEndValue(STATE_BODY_COL_MAP.get(status, DEFAULT_BODY_COL))
        #anim.setKeyValueAt(0.2, QtGui.QColor(255, 255, 255))

        anim.start()


    def boundingRect(self):
        return self.body.boundingRect()

    def paint(self, painter, option, widget):
        pass

    def bodyColor(self):
        return self.body_brush.color()

    def setBodyColor(self, color):
        """
        Used by the animation framework.
        """
        self.body_brush.setColor(color)
        self.body.setBrush(self.body_brush)

    # Create the bodycolor property, so that the qt animation framework can find it.
    bodycolor = QtCore.pyqtProperty(QtGui.QColor, bodyColor, setBodyColor)