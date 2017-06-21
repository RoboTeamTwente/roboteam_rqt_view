import sys

import rospkg
import rospy
import roslib
roslib.load_manifest("roboteam_msgs")

from roboteam_msgs import msg

from qt_gui.plugin import Plugin
from python_qt_binding import QtWidgets, QtGui, QtCore

from tree import tree_list, behaviour_tree, debug_list
from widgets import tree_view, debug_table_widget
from items import tree_graphics_item


TREE_DIR = rospkg.RosPack().get_path('roboteam_tactics') + "/src/trees/json/"
PROJECTS_DIR = rospkg.RosPack().get_path('roboteam_tactics') + "/src/trees/projects/"


class TreeDebugPlugin(Plugin):

    # Signal for when behaviourtree debug info arrives.
    bt_debug_signal = QtCore.pyqtSignal(msg.BtDebugInfo)


    def __init__(self, context):
        super(TreeDebugPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TreeDebugPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self.widget = QtWidgets.QWidget()

        # # Get path to UI file which should be in the "resource" folder of this package
        # ui_file = os.path.join(rospkg.RosPack().get_path('roboteam_rqt_view'), 'resource', 'WorldView.ui')
        # # Extend the widget with all attributes and children from UI file
        # loadUi(ui_file, self.widget)

        # Give QObjects reasonable names
        self.widget.setObjectName('TreeDebugView')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self.widget)


        self.init_ui()

        self._trees = tree_list.TreeList()
        self.debug_list = debug_list.DebugList(self._trees)

        self._displayed_tree = None
        # Name of the rosnode the current displayed tree is from.
        self._displayed_node = ""


        self.init_ros()


        self._trees.load_trees(TREE_DIR, PROJECTS_DIR)

        # ---- Connect signals ----

        self.debug_list.new_rosnode.connect(self.slot_add_rosnode)
        self.debug_list.leaf_update.connect(self.slot_debug_leaf_update)
        self.debug_list.tree_switched.connect(self.slot_node_switched_tree)
        self.debug_table.cellPressed.connect(self.slot_debug_table_cell_pressed)

        # ---- /Connect signals ----


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass


    def init_ros(self):
        self.bt_debug_sub = rospy.Subscriber("bt_debug_info", msg.BtDebugInfo, self.callback_bt_debug)
        self.bt_debug_signal.connect(self.slot_bt_debug)


    def init_ui(self):
        self.widget.setLayout(QtWidgets.QVBoxLayout())

        # ---- Toolbar ----

        self.toolbar_frame = QtWidgets.QFrame()
        self.toolbar_frame.setSizePolicy(self.toolbar_frame.sizePolicy().Maximum, self.toolbar_frame.sizePolicy().Maximum)
        self.toolbar_frame.setLayout(QtWidgets.QHBoxLayout())
        self.widget.layout().addWidget(self.toolbar_frame)

        self.t_select_newest_checkbox = QtWidgets.QCheckBox("Auto select newest tree")
        self.toolbar_frame.layout().addWidget(self.t_select_newest_checkbox)

        self.t_clear_all_trees = QtWidgets.QPushButton("Clear trees")
        self.toolbar_frame.layout().addWidget(self.t_clear_all_trees)
        self.t_clear_all_trees.pressed.connect(self.slot_clear_trees)

        # ---- /Toolbar ----

        self.horizontal_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.widget.layout().addWidget(self.horizontal_splitter)

        # ---- Tree view ----

        self.tree_view = tree_view.TreeView()
        # Enable antialiasing
        self.tree_view.setRenderHints(QtGui.QPainter.Antialiasing)
        self.horizontal_splitter.addWidget(self.tree_view)

        # ---- Tree view ----

        # ---- Debug table ----

        self.debug_table = debug_table_widget.DebugTableWidget()
        self.debug_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.horizontal_splitter.addWidget(self.debug_table)

        # ---- /Debug table ----

        # Calculate the initial sizes.
        total_size = self.horizontal_splitter.width()
        table_size = self.debug_table.minimumSizeHint().width()
        view_size = total_size - table_size
        self.horizontal_splitter.setSizes([view_size, table_size])


    def display_tree(self, disp_tree):
        """
        Displays the tree disp_tree.
        """

        self.tree_view.change_displayed_tree(disp_tree)


    def callback_bt_debug(self, message):
        self.bt_debug_signal.emit(message)


    # ----------------
    # Gui change slots
    # ----------------

    def slot_bt_debug(self, message):

        origin_name = message._connection_header['callerid']

        self.debug_list.update_with_debug_message(origin_name, message)


    def slot_add_rosnode(self, node_name, tree_name):
        self.debug_table.add_node(node_name, tree_name)

        # Do we auto switch to the new nodes tree?
        if self.t_select_newest_checkbox.isChecked():
            self.slot_change_displayed_node(node_name)

            # Select this node in the debug table.
            for item in self.debug_table.findItems(node_name, QtCore.Qt.MatchFixedString):
                self.debug_table.selectRow(item.row())


    def slot_node_switched_tree(self, node_name, tree_name):
        if node_name == self._displayed_node:
            self.slot_change_displayed_node(node_name)

        self.debug_table.update_node(node_name, tree_name)


    def slot_debug_leaf_update(self, node_name, leaf_name, status):
        if self._displayed_node == node_name:
            tree = self.tree_view.tree_item()
            leaf = tree.get_node_by_title(leaf_name)

            if leaf:
                leaf.update()
                leaf.flash()


    def slot_change_displayed_node(self, node_name):
        debug_tree = self.debug_list.get_node_tree(node_name)

        if debug_tree:
            self._displayed_node = node_name
            self.display_tree(debug_tree)


    def slot_debug_table_cell_pressed(self, row, column):
        self.slot_change_displayed_node(self.debug_table.item(row, 0).text())


    def slot_clear_trees(self):
        self.debug_table.clear_table()
        self.debug_list.clear()
        self.tree_view.change_displayed_tree(None)
