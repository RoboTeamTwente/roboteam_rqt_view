from python_qt_binding import QtWidgets, QtCore


class TreeListWidget(QtWidgets.QListWidget):

    def __init__(self):
        """
        The `node_reference` should be a reference to a `tree.BehaviourTree` object.
        """
        super(TreeListWidget, self).__init__()
