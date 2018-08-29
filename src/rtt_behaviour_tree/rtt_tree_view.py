import sys

import rospkg
import rospy
import roslib
roslib.load_manifest("roboteam_msgs")

from qt_gui.plugin import Plugin
from python_qt_binding import QtWidgets, QtGui

from tree import tree_list
from widgets import tree_view, tree_list_widget
from items import tree_graphics_item


PROJECTS_DIR = rospkg.RosPack().get_path('roboteam_tactics') + "/src/trees/projects/"


class TreeViewPlugin(Plugin):

    def __init__(self, context):
        super(TreeViewPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('BehaviourTreePlugin')

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
        self.widget.setObjectName('TreeView')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        self.widget.setWindowTitle("Tree view")
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self.widget)


        self.selected_tree = None

        self.init_ui()

        self._trees = tree_list.TreeList()

        self.init_ros()

        # ---- Connect signals ----

        # New tree in list -> Add to list widget
        self._trees.tree_added.connect(self.list_widget.addItem)
        # List item selected -> Display selected tree
        self.list_widget.currentTextChanged.connect(self.display_tree)

        # ---- /Connect signals ----


        self._trees.load_trees(PROJECTS_DIR)

        # Sort ascending.
        self.list_widget.sortItems()

        # default_tree = None
        # if self.selected_tree:
        #     default_tree = self._trees.get_tree(self.selected_tree)
        #
        # self.tree_item = tree_graphics_item.TreeGraphicsItem(default_tree)
        # self.scene.addItem(self.tree_item)


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
        pass


    def init_ui(self):
        self.widget.setLayout(QtWidgets.QHBoxLayout())

        # ---- Tree view ----

        self.tree_view = tree_view.TreeView()
        # Enable antialiasing
        self.tree_view.setRenderHints(QtGui.QPainter.Antialiasing)
        self.widget.layout().addWidget(self.tree_view)

        # ---- Tree view ----

        # ---- Tree list ----

        self.list_widget = tree_list_widget.TreeListWidget()
        self.list_widget.setSizePolicy(self.list_widget.sizePolicy().Preferred, self.list_widget.sizePolicy().Preferred)
        self.widget.layout().addWidget(self.list_widget)

        # ---- Tree list ----


    # ---- Slots ----------

    def display_tree(self, title):
        self.selected_tree = title

        new_tree = self._trees.get_tree(self.selected_tree)

        self.tree_view.change_displayed_tree(new_tree)
