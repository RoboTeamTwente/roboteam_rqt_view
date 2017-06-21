from python_qt_binding import QtWidgets, QtCore


class DebugTableWidget(QtWidgets.QTableWidget):

    def __init__(self):

        super(DebugTableWidget, self).__init__()

        # Dict to keep track of which node is displayed in which row.
        self._node_to_row = {}

        self.setMinimumWidth(200)
        self.setSizePolicy(self.sizePolicy().Preferred, self.sizePolicy().Preferred)

        self.setColumnCount(2)
        self.setHorizontalHeaderLabels(["Node", "Tree"])

        self.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)

        self.horizontalHeader().setStretchLastSection(True)


    def add_node(self, node_name, tree_name):
        if not node_name in self._node_to_row.keys():
            self.insertRow(self.rowCount())
            self.setItem(self.rowCount()-1, 0, QtWidgets.QTableWidgetItem(node_name))
            self.setItem(self.rowCount()-1, 1, QtWidgets.QTableWidgetItem(tree_name))

            self._node_to_row[node_name] = self.rowCount()-1


    def update_node(self, node_name, tree_name):
        row = self._node_to_row[node_name]
        if row:
            self.setItem(row, 1, QtWidgets.QTableWidgetItem(tree_name))


    def clear_table(self):
        self.setRowCount(0)
