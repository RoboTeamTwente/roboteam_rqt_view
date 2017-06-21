import copy

from python_qt_binding import QtCore

from rtt_behaviour_tree.tree import behaviour_tree


class DebugList(QtCore.QObject):

    # ---- Signals ----

    # Emits when a new rosnode is detected.
    # Sends the name of the node, and the current tree.
    new_rosnode = QtCore.pyqtSignal(str, str)
    # Emitted when a rosnode changes it's tree.
    # Sends the name of the node, and the new tree.
    tree_switched = QtCore.pyqtSignal(str, str)
    # Emitted when a leaf changes state.
    # Sends the name of the node, the name of the leaf, and the new state.
    leaf_update = QtCore.pyqtSignal(str, str, int)

    # ---- /Signals ----

    def __init__(self, tree_list_reference):
        """
        Takes a reference to a TreeList.
        """
        super(DebugList, self).__init__()

        self._trees = tree_list_reference

        # Dict of nodes and the tree they are currently executing.
        self._node_trees = {}


    def get_node_tree(self, name):
        """
        Returns the tree of the node with name `name`.
        If there is no such node, returns None.
        """
        try:
            tree = self._node_trees[name]
        except KeyError:
            print "No such node: " + name
            tree = None

        return tree


    def update_with_debug_message(self, node_name, message):

        if message.type == message.TYPE_STRATEGY or message.type == message.TYPE_ROLE:
            if message.status.status == message.status.STARTUP:
                clean_tree = self._trees.get_tree(message.name)

                if clean_tree:

                    is_new_node = not node_name in self._node_trees

                    if clean_tree:
                        self._node_trees[node_name] = copy.deepcopy(clean_tree)

                    # Call this after adding the tree.
                    # Because objects receiving this signal might want to do something with the new tree.
                    if is_new_node:
                        self.new_rosnode.emit(node_name, message.name)
                    else:
                        self.tree_switched.emit(node_name, message.name)

        elif message.type == message.TYPE_LEAF:
            if node_name in self._node_trees:
                tree = self._node_trees[node_name]
                if tree:
                    leaf = tree.get_node_by_title(message.name)
                    if leaf:
                        leaf.update_status(message.status.status)

                        self.leaf_update.emit(node_name, leaf.title(), leaf.status())

    def clear(self):

        self._node_trees.clear()
