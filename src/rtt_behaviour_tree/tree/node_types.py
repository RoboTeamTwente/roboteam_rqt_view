
class NodeType:

    def __init__(self, name="", category="composite", custom_icon=None):
        """
        custom_icon is a string that replaces the title that is displayed on a node.
        """
        self.name = name
        self.default_title = name
        self.category = node_categories[category]
        self.custom_icon = custom_icon
        self.description = ""
        self.properties = {}


    def load_from_json(self, data):
        self.name = data['name']
        self.default_title = data['title']
        self.category = node_categories[data['category']]

    def has_custom_icon(self):
        return self.custom_icon != None


class NodeCategory:

    def __init__(self, name, child_amount):
        """
        name => String: Name of the category.
        child_amount => int: How many children a node of this type can have. -1 means infinite.
        """

        self.name = name
        self.child_amount = child_amount

    def can_have_children(self):
        return self.child_amount != 0

    def has_single_child(self):
        return self.child_amount == 1



# Default node categories.
node_categories = {
    'root': NodeCategory('root', 1),
    'composite': NodeCategory('composite', -1),
    'decorator': NodeCategory('decorator', 1),
    'action': NodeCategory('action', 0),
    'condition': NodeCategory('condition', 0)
}

# Default node types.
node_types = {
    'Root': NodeType("Root", "root"),

    # Unknown node type. Default when a type not known is encountered.
    'Unknown': NodeType("Unknown", "composite", custom_icon="<UNKNOWN>"),

    'Sequence': NodeType("Sequence", "composite", custom_icon="->"),
    'Priority': NodeType("Priority", "composite", custom_icon="?"),
    'MemSequence': NodeType("MemSequence", "composite", custom_icon="->*"),
    'MemPriority': NodeType("MemPriority", "composite", custom_icon="?*"),

    'Repeater': NodeType("Repeater", "decorator"),
    'RepeatUntilFailure': NodeType("RepeatUntilFailure", "decorator"),
    'RepeatUntilSuccess': NodeType("RepeatUntilSuccess", "decorator"),
    'MaxTime': NodeType("MaxTime", "decorator"),
    'Limiter': NodeType("Limiter", "decorator"),
    'Inverter': NodeType("Inverter", "decorator"),

    'Failer': NodeType("Failer", "action"),
    'Succeeder': NodeType("Succeeder", "action"),
    'Runner': NodeType("Runner", "action"),
    'Error': NodeType("Error", "action"),
    'Wait': NodeType("Wait", "action"),
}

UNKNOWN_NODE_TYPE = node_types['Unknown']


def add_types_from_json(data):
    """
    Loads node types from json.
    """
    for custom_type in data:
        if not 'name' in custom_type or not 'category' in custom_type:
            print 'Warning: Malformed custom node found:'
            print "\t" + str(custom_type)
            return

        node_types[custom_type['name']] =  NodeType(custom_type['name'], custom_type['category'])
