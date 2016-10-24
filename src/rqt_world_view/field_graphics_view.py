from python_qt_binding.QtWidgets import QGraphicsView
from python_qt_binding.QtCore import Qt
from python_qt_binding import QtGui


# Source of the mouse events:
# https://github.com/ros-visualization/rqt_common_plugins/blob/master/rqt_graph/src/rqt_graph/interactive_graphics_view.py

class FieldGraphicsView(QGraphicsView):

    def __init__(self):
        super(FieldGraphicsView, self).__init__()

        self._last_pan_point = None
        self._last_scene_center = None

    def mousePressEvent(self, mouse_event):
        self._last_pan_point = mouse_event.pos()
        self._last_scene_center = self.mapToScene(self.frameRect().center())
        self.setCursor(Qt.ClosedHandCursor)

    def mouseReleaseEvent(self, mouse_event):
        self.setCursor(Qt.ArrowCursor)
        self._last_pan_point = None

    def mouseMoveEvent(self, mouse_event):
        if self._last_pan_point is not None:
            delta_scene = self.mapToScene(mouse_event.pos()) - self.mapToScene(self._last_pan_point)
            if not delta_scene.isNull():
                self.centerOn(self._last_scene_center - delta_scene)
                self._last_scene_center -= delta_scene
            self._last_pan_point = mouse_event.pos()
        QGraphicsView.mouseMoveEvent(self, mouse_event)


    # Called on mouse wheel event.
    # Zooms the graphics view.
    def wheelEvent(self, wheel_event):
        if wheel_event.modifiers() == Qt.NoModifier:
            try:
                delta = wheel_event.angleDelta().y()
            except AttributeError:
                delta = wheel_event.delta()
            delta = max(min(delta, 480), -480)
            mouse_before_scale_in_scene = self.mapToScene(wheel_event.pos())

            scale_factor = 1 + (0.2 * (delta / 120.0))
            scaling = QtGui.QTransform(scale_factor, 0, 0, scale_factor, 0, 0)
            self.setTransform(self.transform() * scaling)

            mouse_after_scale_in_scene = self.mapToScene(wheel_event.pos())
            center_in_scene = self.mapToScene(self.frameRect().center())
            self.centerOn(center_in_scene + mouse_before_scale_in_scene - mouse_after_scale_in_scene)

            wheel_event.accept()
        else:
            QGraphicsView.wheelEvent(self, wheel_event)

    # def _map_to_scene_f(self, pointf):
    #     point = pointf.toPoint()
    #     if pointf.x() == point.x() and pointf.y() == point.y():
    #         # map integer coordinates
    #         return self.mapToScene(point)
    #     elif pointf.x() == point.x():
    #         # map integer x and decimal y coordinates
    #         pointA = self.mapToScene((pointf + QPointF(0, -0.5)).toPoint())
    #         pointB = self.mapToScene((pointf + QPointF(0, 0.5)).toPoint())
    #         return (pointA + pointB) / 2.0
    #     elif pointf.y() == point.y():
    #         # map decimal x  and integer y and coordinates
    #         pointA = self.mapToScene((pointf + QPointF(-0.5, 0)).toPoint())
    #         pointB = self.mapToScene((pointf + QPointF(0.5, 0)).toPoint())
    #         return (pointA + pointB) / 2.0
    #     else:
    #         # map decimal coordinates
    #         pointA = self.mapToScene((pointf + QPointF(-0.5, -0.5)).toPoint())
    #         pointB = self.mapToScene((pointf + QPointF(-0.5, 0.5)).toPoint())
    #         pointC = self.mapToScene((pointf + QPointF(0.5, -0.5)).toPoint())
    #         pointD = self.mapToScene((pointf + QPointF(0.5, 0.5)).toPoint())
    #         return (pointA + pointB + pointC + pointD) / 4.0
