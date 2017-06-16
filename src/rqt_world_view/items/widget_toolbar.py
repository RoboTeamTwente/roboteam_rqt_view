from python_qt_binding import QtWidgets, QtCore

from dialog_field_transformation import DialogFieldTransformation

class WidgetToolbar(QtWidgets.QFrame):

    def __init__(self):
        super(WidgetToolbar, self).__init__()

        self.setLayout(QtWidgets.QHBoxLayout())
        self.setContentsMargins(0, 0, 0, 0)

        # ---- Widgets ----

        self.vision_status_indicator = QtWidgets.QLabel("Vision status")
        self.vision_status_indicator.setAlignment(QtCore.Qt.AlignCenter)
        self.layout().addWidget(self.vision_status_indicator)

        self.dropdown_button = QtWidgets.QToolButton()
        self.dropdown_button.setText("Field")
        self.dropdown_button.setPopupMode(QtWidgets.QToolButton.InstantPopup)
        self.layout().addWidget(self.dropdown_button)

        self.dropdown_menu = QtWidgets.QMenu()
        self.dropdown_button.setMenu(self.dropdown_menu)

        self.out_of_field_action = QtWidgets.QAction(self)
        self.out_of_field_action.setText("Put all out of field")
        self.dropdown_menu.addAction(self.out_of_field_action)

        self.reset_view_action = QtWidgets.QAction(self)
        self.reset_view_action.setText("Reset view")
        self.dropdown_menu.addAction(self.reset_view_action)

        self.clear_debug_action = QtWidgets.QAction(self)
        self.clear_debug_action.setText("Clear debug drawings")
        self.dropdown_menu.addAction(self.clear_debug_action)

        self.transformation_modal_action = QtWidgets.QAction(self)
        self.transformation_modal_action.setText("Configure field transformation")
        self.dropdown_menu.addAction(self.transformation_modal_action)
        self.transformation_modal_action.triggered.connect(self.open_field_transformation_modal)

        self.toggle_halt_button = QtWidgets.QPushButton("Press to halt")
        self.layout().addWidget(self.toggle_halt_button)

        # ---- /Widgets ----

        self.layout().addStretch(1)

        self.field_transformation_dialog = DialogFieldTransformation()


    # --------------------------------------------------------------------------
    # ---- Toolbar slots -------------------------------------------------------
    # --------------------------------------------------------------------------


    def set_vision_status_indicator(self, status):
        if status == True:
            self.vision_status_indicator.setStyleSheet("color: #00aa00")
        else:
            self.vision_status_indicator.setStyleSheet("color: #FF0000")

    def open_field_transformation_modal(self):
        self.field_transformation_dialog.show()
        self.field_transformation_dialog.activateWindow()

    def halt_update(self, message):
        """
        Called when something updates the halt state. Maks the halt button red/ordinary and
        changes the text as well.
        """
        # Check if halting.
        if message.data:
            self.toggle_halt_button.setStyleSheet('QPushButton {background-color: #FF0000;}')
            self.toggle_halt_button.setText("Halting")
        else:
            self.toggle_halt_button.setStyleSheet('QPushButton {}')
            self.toggle_halt_button.setText("Not halting")
