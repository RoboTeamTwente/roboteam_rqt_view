import uuid

from rtt_behaviour_tree.tree import node as tree_node
from rtt_behaviour_tree.utils import vector2
from rtt_behaviour_tree.tree import node_types


class BehaviourTree:

    def __init__(self):

        self._title = ""
        self._root_node = None

        # All the nodes in the tree.
        self._nodes = {}
        self._nodes_by_title = {}


    def add_node(self, node=tree_node.Node()):
        self._nodes[node.id()] = node
        self._nodes_by_title[node.title()] = node

    def title(self):
        return self._title

    def root(self):
        return self._root_node

    def nodes(self):
        return self._nodes

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


    def load_from_json(self, data, project_name=None):

        if project_name:
            self._title = project_name + "/" + data['title']
        else:
            self._title = data['title']

        self._root_node = tree_node.Node()

        self._root_node.set_title("O")
        self._root_node.type = node_types.node_types['Root']
        self._root_node.description = data['description']

        self._root_node.display.x = data['display']['x']
        self._root_node.display.y = data['display']['y']

        # Add the nodes.
        for key, node_data in data['nodes'].iteritems():
            node = tree_node.Node()
            node.load_from_json(node_data)

            self.add_node(node)

        # Connect the nodes to each other.
        if 'root' in data:
            root = data['root']
            # This check for None is needed because apparently it sometimes happens
            # that a tree doesn't have a root node?
            # Which is weird, but something we have to look out for.
            if root:
                uid = uuid.UUID(data['root'])
                self._root_node.add_child(self._nodes[uid])
            else:
                print "Warning: This tree doesn't have a root node: "
                print "\t" + self._title

        for key, node_data in data['nodes'].iteritems():
            uid = uuid.UUID(key)
            node = self._nodes[uid]

            if 'children' in node_data:
                for child_key in node_data['children']:
                    child_uid = uuid.UUID(child_key)
                    node.add_child(self._nodes[child_uid])

            if 'child' in node_data:
                child_uid = uuid.UUID(node_data['child'])
                node.add_child(self._nodes[child_uid])
