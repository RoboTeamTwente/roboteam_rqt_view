from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsSceneMouseEvent

from python_qt_binding.QtCore import pyqtSignal, Qt


class FieldGraphicsScene(QGraphicsScene):

    right_click_signal = pyqtSignal(QGraphicsSceneMouseEvent)
    left_click_signal = pyqtSignal(QGraphicsSceneMouseEvent)
    mouse_move_signal = pyqtSignal(QGraphicsSceneMouseEvent)

    def __init__(self):
        super(FieldGraphicsScene, self).__init__()

    def mousePressEvent(self, event):
        """Reimplement the mouse event function."""

        # Don't propagate right clicks.
        if (event.button() == Qt.RightButton):
            # Emit the right click signal.
            self.right_click_signal.emit(event)
            event.accept()
            return

        if (event.button() == Qt.LeftButton):
            # Emit the right click signal.
            self.left_click_signal.emit(event)
            event.accept()
            # return

        # Process everything else the default way.
        super(FieldGraphicsScene, self).mousePressEvent(event)


    def mouseMoveEvent(self, event):
        """Reimplement the mouse press event function."""

        self.mouse_move_signal.emit(event)

        super(FieldGraphicsScene, self).mouseMoveEvent(event)
