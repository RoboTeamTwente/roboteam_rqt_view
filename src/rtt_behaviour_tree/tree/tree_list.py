import json
import os

from python_qt_binding import QtCore

from rtt_behaviour_tree.tree import behaviour_tree, node_types


class TreeList(QtCore.QObject):

    # ---- Signals ----

    # Emits when a tree is added to the list.
    # Sends along the title of the tree.
    tree_added = QtCore.pyqtSignal(str)

    # ---- /Signals ----

    def __init__(self):
        super(TreeList, self).__init__()

        self._trees = {}


    def load_trees(self, tree_dir, projects_dir):
        """
        Loads all behaviour trees from the json files found in the given tree_dir.
        Also loads all tree projects from the projects_dir.
        """
        for file_name in os.listdir(tree_dir):
            if file_name.endswith(".json"):

                print "Loading " + file_name

                with open(tree_dir + file_name) as data_file:
                    data = json.load(data_file)

                    # Extract the custom node types from the tree.
                    if 'custom_nodes' in data:
                        node_types.add_types_from_json(data['custom_nodes'])

                    tree = behaviour_tree.BehaviourTree()
                    tree.load_from_json(data)
                    self._trees[tree.title()] = tree

                    # Notify other objects of the new tree.
                    self.tree_added.emit(tree.title())

        for file_name in os.listdir(projects_dir):
            if file_name.endswith(".b3"):

                print "Loading " + file_name

                with open(projects_dir + file_name) as data_file:
                    data = json.load(data_file)['data']

                    # Extract the custom node types from the tree.
                    if 'custom_nodes' in data:
                        node_types.add_types_from_json(data['custom_nodes'])

                    if 'trees' in data:
                        for tree_data in data['trees']:
                            tree = behaviour_tree.BehaviourTree()
                            tree.load_from_json(tree_data, os.path.splitext(file_name)[0])
                            self._trees[tree.title()] = tree

                            # Notify other objects of the new tree.
                            self.tree_added.emit(tree.title())



    def titles(self):
        """
        Returns a list of all tree titles.
        """
        titles = []
        for key, tree in self._trees.iteritems():
            titles.append(tree.title())
        return titles


    def get_tree(self, title):
        """
        Returns the tree with the given title.
        If there is no such tree, returns None.
        """
        try:
            tree = self._trees[title]
        except KeyError:
            print "No such tree: " + title
            tree = None

        return tree
