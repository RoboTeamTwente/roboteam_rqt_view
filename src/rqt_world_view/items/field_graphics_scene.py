from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsSceneMouseEvent

from python_qt_binding.QtCore import pyqtSignal, Qt


class FieldGraphicsScene(QGraphicsScene):

    right_click_signal = pyqtSignal(QGraphicsSceneMouseEvent)

    def __init__(self):
        super(FieldGraphicsScene, self).__init__()

    # Reimplement the mouse event function.
    def mousePressEvent(self, event):
        # Don't propagate right clicks.
        if (event.button() == Qt.RightButton):
            # Emit the right click signal.
            self.right_click_signal.emit(event)
            event.accept()
            return

        # Process everything else the default way.
        super(FieldGraphicsScene, self).mousePressEvent(event)
