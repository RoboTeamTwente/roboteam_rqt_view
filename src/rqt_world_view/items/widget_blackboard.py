from python_qt_binding.QtWidgets import QFrame, QLabel, QGridLayout, QPushButton, QComboBox, QLineEdit, QStackedWidget, QCheckBox
from python_qt_binding.QtGui import QDoubleValidator, QRegExpValidator
from python_qt_binding.QtCore import QRegExp

from roboteam_msgs import msg


class WidgetBlackboard(QFrame):

    def __init__(self):
        super(WidgetBlackboard, self).__init__()

        self.setLayout(QGridLayout())

        self.setSizePolicy(self.sizePolicy().Preferred, self.sizePolicy().Fixed)

        self.layout().addWidget(QLabel("Blackboard"), 0, 0)

        # ---- Add item button ----
        self.add_item_button = QPushButton("Add value")
        self.layout().addWidget(self.add_item_button, 0, 1, 1, 3)
        self.add_item_button.clicked.connect(self.slot_add_item)
        # ---- /Add item button ----

        # ---- Setup table ----

        self.layout().addWidget(QLabel("Type"), 1, 0)
        self.layout().addWidget(QLabel("Name"), 1, 1)
        self.layout().addWidget(QLabel("Value"), 1, 2)

        self.blackboard_items = {}

        # The row where we can start inserting blackboard items.
        self.insert_row = 2

        # Keeps track of which item id to use next.
        self.new_item_id = 0

        # ---- /Setup table ----


    # Returns the values in this blackboard in the form of a
    # Blackboard.msg
    def get_blackboard_message(self):

        message = msg.Blackboard()

        for item_id, item in self.blackboard_items.iteritems():
            entry = item.get_entry_message()

            # Only add items that have a name.
            if entry.name != "":

                if type(entry) is msg.BoolEntry:
                    message.bools.append(entry)
                elif type(entry) is msg.StringEntry:
                    message.strings.append(entry)
                elif type(entry) is msg.Float64Entry:
                    message.doubles.append(entry)
                elif type(entry) is msg.Int32Entry:
                    message.ints.append(entry)

        return message


    # Returns the blackboard in a way usable by the "TestX" program.
    # Returns a list of blackboard entries.
    # The entry format is as follows: "type:name=value"
    def get_blackboard_testx(self):
        blackboard = []
        for item_id, item in self.blackboard_items.iteritems():
            entry = item.get_entry_testx()

            # Only append non-empty entries.
            if entry != "":
                blackboard.append(entry)

        return blackboard


    # Changes whether the widgets in the blackboard are editable.
    # editable: boolean
    def set_editable(self, editable):
        self.add_item_button.setEnabled(editable)

        for item_id, item in self.blackboard_items.iteritems():
            item.set_editable(editable)


    # Adds a new blackboard item.
    def slot_add_item(self):
        item = BlackboardItem(self.new_item_id, self.slot_remove_item)
        self.blackboard_items[self.new_item_id] = item

        self.layout().addWidget(item.type_widget, self.insert_row, 0)
        self.layout().addWidget(item.name_widget, self.insert_row, 1)
        self.layout().addWidget(item.value_widget, self.insert_row, 2)
        self.layout().addWidget(item.remove_widget, self.insert_row, 3)
        item.remove_widget.setMaximumWidth(30)

        self.new_item_id += 1
        self.insert_row += 1


    # Removes the blackboard item with id: item_id.
    def slot_remove_item(self, item_id):
        item = self.blackboard_items[item_id]

        self.layout().removeWidget(item.type_widget)
        self.layout().removeWidget(item.name_widget)
        self.layout().removeWidget(item.value_widget)
        self.layout().removeWidget(item.remove_widget)

        item.type_widget.deleteLater()
        item.type_widget = None
        item.name_widget.deleteLater()
        item.name_widget = None
        item.value_widget.deleteLater()
        item.value_widget = None
        item.remove_widget.deleteLater()
        item.remove_widget = None

        del self.blackboard_items[item_id]



class BlackboardItem():

    # Remove_callback is the function to call when the
    # remove button is pressed.
    def __init__(self, item_id, remove_callback):

        self.id = item_id
        self.remove_callback = remove_callback

        self.type_widget = QComboBox()
        self.type_widget.insertItem(0, "String")
        self.type_widget.insertItem(1, "Double")
        self.type_widget.insertItem(2, "Int")
        self.type_widget.insertItem(3, "Bool")

        # ---- Value widget ----

        self.value_widget = QStackedWidget()
        self.value_widget.setMinimumSize(20, 0)
        self.value_widget.setSizePolicy(self.value_widget.sizePolicy().Minimum, self.value_widget.sizePolicy().Fixed)

        # String input.
        self.string_edit = QLineEdit()
        self.value_widget.addWidget(self.string_edit)

        # Double input.
        self.double_edit = QLineEdit()
        self.double_validator = QDoubleValidator()
        self.double_edit.setValidator(self.double_validator)
        self.value_widget.addWidget(self.double_edit)

        # Integer input.
        self.int_edit = QLineEdit()
        self.int_validator = QRegExpValidator(QRegExp("[0-9]+"))
        self.int_edit.setValidator(self.int_validator)
        self.value_widget.addWidget(self.int_edit)

        # Boolean input.
        self.bool_edit = QCheckBox()
        self.value_widget.addWidget(self.bool_edit)

        # ---- /Value widget ----

        self.name_widget = QLineEdit()
        self.remove_widget = QPushButton("x")

        # ---- Connect signals ----

        self.remove_widget.clicked.connect(self.slot_remove_widget_pressed)
        self.type_widget.currentIndexChanged.connect(self.slot_type_selection_changed)


    # Returns one of the below:
    # msg.StringEntry
    # msg.Int32Entry
    # msg.Float64Entry
    # msg.BoolEntry
    def get_entry_message(self):
        typestring = self.type_widget.currentText()

        if typestring == "String":
            item = msg.StringEntry()
            item.name = self.name_widget.text()
            item.value = self.string_edit.text()
            return item
        elif typestring == "Int":
            item = msg.Int32Entry()
            item.name = self.name_widget.text()
            try:
                item.value = int(self.int_edit.text())
            except ValueError, e:
                item.value = 0
            return item
        elif typestring == "Double":
            item = msg.Float64Entry()
            item.name = self.name_widget.text()
            try:
                item.value = float(self.double_edit.text())
            except ValueError, e:
                item.value = 0
            return item
        elif typestring == "Bool":
            item = msg.BoolEntry()
            item.name = self.name_widget.text()
            item.value = self.bool_edit.isChecked()
            return item


    # Returns the blackboard entry in a way usable by the "TestX" program.
    # The format is as follows: "type:name=value"
    # Returns an empty string if the entry has no name.
    def get_entry_testx(self):
        # Maps selectable types to their "TestX" name.
        # Currently it is only lowercasing, but there is no
        # guarantee it stays that way. Hence the dict.
        type_mapping = {
            "String": "string",
            "Int": "int",
            "Double": "double",
            "Bool": "bool"
        }

        typestring = type_mapping.get(self.type_widget.currentText(), "")

        if typestring != "":
            name = self.name_widget.text()

            # If there is no name, return an empty string.
            if name == "":
                return ""

            value = ""
            if typestring == "string":
                value = "\"" + self.string_edit.text() + "\""
            elif typestring == "int":
                try:
                    value = int(self.int_edit.text())
                except ValueError, e:
                    value = 0
            elif typestring == "double":
                try:
                    value = float(self.double_edit.text())
                except ValueError, e:
                    value = 0
            elif typestring == "bool":
                if self.bool_edit.isChecked():
                    value = "true"
                else:
                    value = "false"

            return typestring + ":" + name + "=" + str(value)


    # Changes whether the entries options are editable.
    # editable: boolean
    def set_editable(self, editable):
        self.type_widget.setEnabled(editable)
        self.name_widget.setEnabled(editable)
        self.string_edit.setEnabled(editable)
        self.double_edit.setEnabled(editable)
        self.int_edit.setEnabled(editable)
        self.bool_edit.setEnabled(editable)
        self.remove_widget.setEnabled(editable)


    def slot_remove_widget_pressed(self):
        self.remove_callback(self.id)

    # This slot is connected to the type_widget combobox currentIndexChanged().
    def slot_type_selection_changed(self, index):

        # Clear the inputs.
        self.string_edit.clear()
        self.double_edit.clear()
        self.int_edit.clear()
        self.bool_edit.setChecked(False)

        # Change the visible input.
        self.value_widget.setCurrentIndex(index)
