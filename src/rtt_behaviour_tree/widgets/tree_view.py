from python_qt_binding.QtCore import Qt
from python_qt_binding import QtGui, QtWidgets

from rtt_behaviour_tree.items import tree_graphics_item


# Source of the mouse events:
# https://github.com/ros-visualization/rqt_common_plugins/blob/master/rqt_graph/src/rqt_graph/interactive_graphics_view.py

class TreeView(QtWidgets.QGraphicsView):

    def __init__(self):
        super(TreeView, self).__init__()

        self._last_pan_point = None
        self._last_scene_center = None

        # The tree to display.
        self._disp_tree = None
        # The item to display the tree in.
        self._tree_item = None

        self.setMinimumWidth(200)
        self.setSizePolicy(self.sizePolicy().MinimumExpanding, self.sizePolicy().Preferred)

        self.setScene(QtWidgets.QGraphicsScene())



    def tree_reference(self):
        return self._disp_tree

    def tree_item(self):
        return self._tree_item


    def change_displayed_tree(self, tree_reference):
        self._disp_tree = tree_reference

        if self._tree_item:
            self.scene().removeItem(self._tree_item)

        self._tree_item = tree_graphics_item.TreeGraphicsItem(tree_reference)
        self.scene().addItem(self._tree_item)

        # Shrink the scene to fit the new tree when is smaller.
        self.scene().setSceneRect(self.scene().itemsBoundingRect())


    # ---- QGraphicsView ----------

    def mousePressEvent(self, mouse_event):
        if (mouse_event.button() == Qt.LeftButton):
            self._last_pan_point = mouse_event.pos()
            self._last_scene_center = self.mapToScene(self.frameRect().center())

        super(TreeView, self).mousePressEvent(mouse_event)

    def mouseReleaseEvent(self, mouse_event):
        if (mouse_event.button() == Qt.LeftButton):
            self.setCursor(Qt.ArrowCursor)
            self._last_pan_point = None

        super(TreeView, self).mouseReleaseEvent(mouse_event)

    def mouseMoveEvent(self, mouse_event):
        if self._last_pan_point is not None:
            delta_scene = self.mapToScene(mouse_event.pos()) - self.mapToScene(self._last_pan_point)
            if not delta_scene.isNull():
                self.centerOn(self._last_scene_center - delta_scene)
                self._last_scene_center -= delta_scene
            self._last_pan_point = mouse_event.pos()
            self.setCursor(Qt.ClosedHandCursor)
        QtWidgets.QGraphicsView.mouseMoveEvent(self, mouse_event)


    def wheelEvent(self, wheel_event):
        """
        Called on mouse wheel event.
        Zooms the graphics view.
        """
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

    # ---- /QGraphicsView ----------

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
