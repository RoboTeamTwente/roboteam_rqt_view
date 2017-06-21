from python_qt_binding import QtWidgets, QtCore

from rtt_behaviour_tree.items import node_graphics_item

class TreeGraphicsItem(QtWidgets.QGraphicsItem):

    def __init__(self, tree_reference=None):
        """
        The `node_reference` should be a reference to a `tree.BehaviourTree` object.
        """
        super(TreeGraphicsItem, self).__init__()

        if tree_reference:

            self.tree = tree_reference

            self._nodes = {}
            self._nodes_by_title = {}

            self._root_node = node_graphics_item.NodeGraphicsItem(tree_reference.root())
            self._root_node.setParentItem(self)

            for uuid, node in self.tree.nodes().iteritems():
                node_graphic = node_graphics_item.NodeGraphicsItem(node)
                node_graphic.setParentItem(self)
                self._nodes[uuid] = node_graphic
                self._nodes_by_title[node.title()] = node_graphic

    def get_node_by_uuid(self, uuid):
        if uuid in self._nodes:
            return self._nodes[uuid]
        else:
            return None


    def get_node_by_title(self, title):
        if title in self._nodes_by_title:
            return self._nodes_by_title[title]
        else:
            return None


    def update(self):
        for uuid, node in self._nodes.iteritems():
            node.update()

    def boundingRect(self):
        return QtCore.QRectF(0, 0, 0, 0)


    def paint(self, painter, option, widget):
        pass
