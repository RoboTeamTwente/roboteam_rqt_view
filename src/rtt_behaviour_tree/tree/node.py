import uuid

from rtt_behaviour_tree.utils import vector2
from rtt_behaviour_tree.tree import node_types

class Node:

    def __init__(self):
        self._id = uuid.uuid1()
        self.type = None
        self._title = ""
        self.description = ""
        self.display = vector2.Vector2()
        self.properties = {}
        self._children = []

        self._blackboard = ""

        # Status of this node. For the values see roboteam_msgs/BtStatus.msg.
        self._status = None

    def id(self):
        return self._id

    def title(self):
        return self._title

    def status(self):
        return self._status

    def set_title(self, title):
        self._title = title

    def update_status(self, status):
        self._status = status

    def has_blackboard(self):
        if self._blackboard == "":
            return False
        else:
            return True

    def add_to_blackboard(self, param_name, value):
        self._blackboard += param_name + ": " + str(value) + "\n"

    def get_blackboard(self):
        return self._blackboard

    def load_from_json(self, data):
        self._id = uuid.UUID(data['id'])

        if data['name'] in node_types.node_types:
            self.type = node_types.node_types[data['name']]
        else:
            print "Warning: Node has unknown type:"
            print "\t" + str(data)
            self.type = node_types.UNKNOWN_NODE_TYPE

        self._title = data['title']
        self.description = data['description']

        self.display.x = data['display']['x']
        self.display.y = data['display']['y']


    def children(self):
        return self._children

    def add_child(self, child):
        self._children.append(child)
