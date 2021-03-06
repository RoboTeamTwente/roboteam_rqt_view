from python_qt_binding.QtWidgets import QFrame, QLabel, QGridLayout, QPushButton, QLineEdit, QStackedWidget, QCheckBox
from python_qt_binding.QtGui import QDoubleValidator, QRegExpValidator
from python_qt_binding.QtCore import QRegExp, Qt
from rqt_world_view.items.non_scrollable_combo_box import NonScrollableQComboBox
from rqt_world_view.utils import tactics_informator

import unicodedata

from roboteam_msgs import msg


class WidgetBlackboard(QFrame):

    def __init__(self, skill):
        super(WidgetBlackboard, self).__init__()

        # Get all parameters of the skill
        self.parameters = tactics_informator.get_parameters(skill)

        self.setLayout(QGridLayout())

        self.setSizePolicy(self.sizePolicy().Preferred, self.sizePolicy().Fixed)

        self.layout().addWidget(QLabel("Blackboard"), 0, 0)

        # ---- Add item button ----
        self.add_item_button = QPushButton("Add value")
        self.add_item_button.setFocusPolicy(Qt.ClickFocus)
        self.layout().addWidget(self.add_item_button, 0, 1, 1, 3)
        self.add_item_button.clicked.connect(self.slot_add_item)
        # ---- /Add item button ----

        # ---- Setup table ----

        self.layout().addWidget(QLabel("Parameter"), 1, 0)
        self.layout().addWidget(QLabel("Value"), 1, 1)

        self.blackboard_items = {}

        # The row where we can start inserting blackboard items.
        self.insert_row = 2

        # Keeps track of which item id to use next.
        self.new_item_id = 0

        # ---- /Setup table ----


    def get_state(self):
        state = []

        for item_id, item in self.blackboard_items.items():
            item_state = item.get_state()
            state.append(item_state)

        return state


    def set_state(self, state):
        for item_state in state:
            item = self.slot_add_item()
            item.set_state(item_state)


    def get_blackboard_message(self):
        """Returns the values in this blackboard in the form of a Blackboard.msg"""

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


    def get_blackboard_testx(self):
        """
        Returns the blackboard in a way usable by the "TestX" program.
        Returns a list of blackboard entries.
        The entry format is as follows: "type:name=value"
        """
        blackboard = []
        for item_id, item in self.blackboard_items.iteritems():
            entry = item.get_entry_testx()

            # Only append non-empty entries.
            if entry != "":
                blackboard.append(entry)

        return blackboard


    def set_editable(self, editable):
        """
        Changes whether the widgets in the blackboard are editable.
        editable: boolean
        """

        self.add_item_button.setEnabled(editable)

        for item_id, item in self.blackboard_items.iteritems():
            item.set_editable(editable)


    def slot_add_item(self):
        """Adds a new blackboard item."""
        item = BlackboardItem(self.new_item_id, self.slot_remove_item, self.parameters)
        self.blackboard_items[self.new_item_id] = item
	
        self.layout().addWidget(item.param_widget, self.insert_row, 0)
        self.layout().addWidget(item.value_widget, self.insert_row, 1)
        self.layout().addWidget(item.remove_widget, self.insert_row, 2)
        item.remove_widget.setMaximumWidth(30)

        self.new_item_id += 1
        self.insert_row += 1

        return item


    def slot_remove_item(self, item_id):
        """Removes the blackboard item with id: item_id."""
        item = self.blackboard_items[item_id]

        self.layout().removeWidget(item.param_widget)
        self.layout().removeWidget(item.value_widget)
        self.layout().removeWidget(item.remove_widget)

        item.param_widget.deleteLater()
        item.param_widget = None
        item.value_widget.deleteLater()
        item.value_widget = None
        item.remove_widget.deleteLater()
        item.remove_widget = None

        del self.blackboard_items[item_id]


class BlackboardItem():

    def __init__(self, item_id, remove_callback, parameters):
        """
        Remove_callback is the function to call when the
        remove button is pressed.
        """

        self.id = item_id
        self.remove_callback = remove_callback
        self.parameters = parameters


        # ---- Param widget ----

        self.param_widget = NonScrollableQComboBox()

        if parameters != None:
            sortedParameters = sorted(parameters, key=lambda s: s.lower())

            # Add tooltip to the parameters in the dropdown
            for i in range(len(parameters.keys())):
                parameter = sortedParameters[i]
                self.param_widget.insertItem(i, parameter)
                # Construct tooltip
                if 'Descr' in parameters[parameter] or 'Used when' in parameters[parameter] or 'Note' in parameters[parameter]:
                    if 'Descr' in parameters[parameter]:
                        description = parameters[parameter]['Descr'] + "\n"
                    if 'Used when' in parameters[parameter]:
                        description = description + "Used when: " + parameters[parameter]['Used when'] + "\n"
                    if 'Note' in parameters[parameter]:
                        description = description + "Note: " + parameters[parameter]['Note']
                    else:
                        # Remove trailing newline
                        description = description[:-1]
                else:
                    description = 'No description available'
                self.param_widget.setItemData(i, description, Qt.ToolTipRole)

            self.param_widget.insertItem(len(parameters.keys()), "Other")
            self.param_widget.setItemData(len(parameters.keys()), "Used to send any parameter. Use NAME:VALUE. Type is derived. Bool: True or False, Int: integer value, Double: two integer values seperated by a dot (.), String: anything else", Qt.ToolTipRole)
        else:
            self.param_widget.insertItem(0, "Other")
            self.param_widget.setItemData(0, "Used to send any parameter. Use NAME:VALUE. Type is derived. Bool: True of Ralse, Int: integer value, Double: two integer values seperated by a dot (.), String: anything else", Qt.ToolTipRole)

        # ---- /Param widget ----


        # ---- Value widget ----

        self.value_widget = QStackedWidget()
        self.value_widget.setMinimumSize(20, 0)
        self.value_widget.setSizePolicy(self.value_widget.sizePolicy().Minimum, self.value_widget.sizePolicy().Fixed)

        # String input.
        self.string_edit = QLineEdit()
        self.value_widget.addWidget(self.string_edit)

        # Double input.
        self.double_edit = QLineEdit()
        self.double_validator = QRegExpValidator(QRegExp("-?[0-9]*.?[0-9]*"))
        self.double_edit.setValidator(self.double_validator)
        self.value_widget.addWidget(self.double_edit)

        # Integer input.
        self.int_edit = QLineEdit()
        self.int_validator = QRegExpValidator(QRegExp("-?[0-9]+"))
        self.int_edit.setValidator(self.int_validator)
        self.value_widget.addWidget(self.int_edit)

        # Boolean input.
        self.bool_edit = QCheckBox()
        self.value_widget.addWidget(self.bool_edit)

        # Dropdown input. Used when "Can be" is specified in YAML
        self.dropdown_edit = NonScrollableQComboBox()
        self.value_widget.addWidget(self.dropdown_edit)
        
        # Other input. Used to input anything. Format as NAME:VALUE. Type is derived automatically
        self.other_edit = QLineEdit()
        self.value_widget.addWidget(self.other_edit)

        # Make value widget show correct input
        self.slot_type_selection_changed(self.param_widget.currentIndex())

        # ---- /Value widget ----

        self.remove_widget = QPushButton("x")
        self.remove_widget.setFocusPolicy(Qt.ClickFocus)
        self.remove_widget.setMinimumWidth(20)
        self.remove_widget.setSizePolicy(self.remove_widget.sizePolicy().Preferred, self.remove_widget.sizePolicy().Fixed)

        # ---- Connect signals ----
        self.remove_widget.clicked.connect(self.slot_remove_widget_pressed)
        self.param_widget.currentIndexChanged.connect(self.slot_type_selection_changed)


    def get_state(self):
        state = {}
        param = self.param_widget.currentText()
        state["param"] = param
        
        # If 'Other' is chosen, the user can supply anything. The type of the value is derived in get_entry_message()
        if param == "Other":
            typestring = "undetermined"
        else:
            typestring = self.parameters[param]['Type']
            
        state["typestring"] = typestring

        # If the dropdown is used, read its text instead of the field of the corresponding type
        using_dropdown = self.value_widget.currentIndex() == 4
        if using_dropdown:
            state["value"] = self.dropdown_edit.currentText()
        elif typestring == "String":
            state["value"] = self.string_edit.text()
        elif typestring == "Int":
            state["value"] = self.int_edit.text()
        elif typestring == "Double":
            state["value"] = self.double_edit.text()
        elif typestring == "Bool":
            state["value"] = self.bool_edit.isChecked()
        elif typestring == "undetermined":
            state["value"] = self.other_edit.text()
        return state


    def set_state(self, state):
        typestring = state["typestring"]

        index = self.param_widget.findText(state["param"])
        self.param_widget.setCurrentIndex(index if index >= 0 else 0)
	
	    # If the dropdown is used, write the value to there instead of to the field of the corresponding type
        using_dropdown = self.value_widget.currentIndex() == 4
        if using_dropdown:
            index = self.dropdown_edit.findText(state["value"])
            if index >= 0:
                self.dropdown_edit.setCurrentIndex(index)
        elif typestring == "String":
            self.string_edit.setText(state["value"])
        elif typestring == "Int":
            self.int_edit.setText(state["value"])
        elif typestring == "Double":
            self.double_edit.setText(state["value"])
        elif typestring == "Bool":
            checked = bool(state["value"])
            self.bool_edit.setChecked(checked)
        elif typestring == "undetermined":
            self.other_edit.setText(state["value"])


    def get_entry_message(self):
        """
        Returns one of the below:
        msg.StringEntry
        msg.Int32Entry
        msg.Float64Entry
        msg.BoolEntry
        """
        param = self.param_widget.currentText()
        # Special case if parameter is Other
        if param == 'Other':
            param = self.other_edit.text()
            info = self.get_info_from_other(param)
            name = info[0]
            value = info[1]
            typestring = info[2]
            if typestring == "String":
                item = msg.StringEntry()
            elif typestring == "Int":
                item = msg.IntEntry()
            elif typestring == "Double":
                item = msg.DoubleEntry()
            elif typestring == "Bool":
                item = msg.BoolEntry()
            else:
                item = msg.StringEntry()
            item.name = name
            item.value = value
            return item
        else:
            typestring = self.parameters[param]['Type']

        if typestring == "String":
            item = msg.StringEntry()
            item.name = self.param_widget.currentText()
            item.value = self.string_edit.text()
            return item
        elif typestring == "Int":
            item = msg.Int32Entry()
            item.name = self.param_widget.currentText()
            try:
                item.value = int(self.int_edit.text())
            except ValueError, e:
                item.value = 0
            return item
        elif typestring == "Double":
            item = msg.Float64Entry()
            item.name = self.param_widget.currentText()
            try:
                item.value = float(self.double_edit.text())
            except ValueError, e:
                item.value = 0
            return item
        elif typestring == "Bool":
            item = msg.BoolEntry()
            item.name = self.param_widget.currentText()
            item.value = self.bool_edit.isChecked()
            return item


    def get_entry_testx(self):
        """
        Returns the blackboard entry in a way usable by the "TestX" program.
        The format is as follows: "type:name=value"
        Returns an empty string if the entry has no name.
        """

        # Maps selectable types to their "TestX" name.
        # Currently it is only lowercasing, but there is no
        # guarantee it stays that way. Hence the dict.
        type_mapping = {
            "String": "string",
            "Int": "int",
            "Double": "double",
            "Bool": "bool"
        }

        param = self.param_widget.currentText()
        if param == "Other":
            param = self.other_edit.text()
            info = self.get_info_from_other(param)
            if info == None:
                return ""
            name = info[0]
            value = info[1]
            typestring = info[2]
            typestring = type_mapping.get(typestring)
            return typestring + ":" + name + "=" + str(value).lower()
        else:
            typestring = type_mapping.get(self.parameters[self.param_widget.currentText()]['Type'], "")

        if typestring != "":
            name = unicodedata.normalize('NFKD', self.param_widget.currentText()).encode('ascii','ignore')

            # If there is no name, return an empty string.
            if name == "":
                return ""
	        # Check if the dropdown is used. We assume that this can be the case for every type except booleans
            using_dropdown = self.value_widget.currentIndex() == 4
            value = ""
            if typestring == "string":
                if using_dropdown:
                    value = self.dropdown_edit.currentText()
                else:
                    value = self.string_edit.text()
                # Convert the unicode string coming from the text box into a python one.
                value = unicodedata.normalize('NFKD', value).encode('ascii','ignore')
            elif typestring == "int":
                try:
                    if using_dropdown:
                        value = int(self.dropdown_edit.currentText())
                    else:
                        value = int(self.int_edit.text())
                except ValueError, e:
                    value = 0
            elif typestring == "double":
                try:
                    if using_dropdown:
                        value = float(self.dropdown_edit.currentText())
                    else:
                        value = float(self.double_edit.text())
                except ValueError, e:
                    value = 0
            elif typestring == "bool":
                value = str(self.bool_edit.isChecked()).lower()
            return typestring + ":" + name + "=" + str(value)


    def get_info_from_other(self, param):
        """Returns the name, value and type of the Other parameter"""
        
        if len(param.split(":")) != 2:
            return
        name = param.split(":")[0]
        valueText = param.split(":")[1]
        # Booleans
        if valueText == "True" or valueText == "False" or valueText == "true" or valueText == "false":
            typeText = "Bool"
            value = valueText == "True" or valueText == "true"
        else:
        # Hacky but easy solution. Ask forgiveness, not permission
            # Ints
            try:
                value = int(valueText)
                typeText = "Int"
            except ValueError, e:
                # Doubles
                try:
                    value = float(valueText)
                    typeText = "Double"
                except ValueError, e:
                    # Strings
                    value = valueText
                    typeText = "String"
        return [name, value, typeText]
        

    def set_editable(self, editable):
        """
        Changes whether the entries options are editable.
        editable: boolean
        """
        self.param_widget.setEnabled(editable)
        self.dropdown_edit.setEnabled(editable)
        self.string_edit.setEnabled(editable)
        self.double_edit.setEnabled(editable)
        self.int_edit.setEnabled(editable)
        self.bool_edit.setEnabled(editable)
        self.other_edit.setEnabled(editable)
        self.remove_widget.setEnabled(editable)


    def slot_remove_widget_pressed(self):
        self.remove_callback(self.id)


    def slot_type_selection_changed(self, index):
        """This slot is connected to the param_widget combobox currentIndexChanged()."""

        # Clear the inputs.
        self.string_edit.clear()
        self.double_edit.clear()
        self.int_edit.clear()
        self.bool_edit.setChecked(False)
        self.dropdown_edit.clear()
        self.other_edit.clear()

        selectedParam = self.param_widget.currentText()
        # If 'Other' has been selected, set input to other_edit and return
        if selectedParam == "Other":
            self.value_widget.setCurrentIndex(5)
            return

        parameter = self.parameters[selectedParam]

        # If 'Can be' has been specified in YAML change the inputfield to a dropdown with all possible options
        if 'Can be' in parameter:
            options = parameter['Can be']
            for i in range(len(options)):
                # Check if options contains dicts or values. If it contains dicts it means that a description is included
                if isinstance(options[0], dict):
                    key = options[i].keys()[0]
                    self.dropdown_edit.insertItem(i, str(key))
                    description = options[i][key]
                    self.dropdown_edit.setItemData(i, description, Qt.ToolTipRole)
                else:
                    self.dropdown_edit.insertItem(i, str(options[i]))
                    self.dropdown_edit.setItemData(i, "No description available", Qt.ToolTipRole)
            # Check if a default value is present and set it if it is
            if 'Default' in parameter:
                defaultValue = parameter['Default']
                index = self.dropdown_edit.findText(defaultValue)
                if index >= 0:
                    self.dropdown_edit.setCurrentIndex(index)
            self.value_widget.setCurrentIndex(4)
        else:
            # Change the visible input and set default value if present
            requiredType = parameter['Type']
            if 'Default' in parameter:
                defaultValue = parameter['Default']
            else:
                defaultValue = ""
            if requiredType == 'String':
                self.value_widget.setCurrentIndex(0)
                self.string_edit.setText(defaultValue)
            elif requiredType == 'Double':
                self.value_widget.setCurrentIndex(1)
                self.double_edit.setText(str(defaultValue))
            elif requiredType == 'Int':
                self.value_widget.setCurrentIndex(2)
                self.int_edit.setText(str(defaultValue))
            elif requiredType == 'Bool':
                self.value_widget.setCurrentIndex(3)
                self.bool_edit.setChecked(defaultValue != "" and defaultValue)
            else:
                self.value_widget.setCurrentIndex(0)
