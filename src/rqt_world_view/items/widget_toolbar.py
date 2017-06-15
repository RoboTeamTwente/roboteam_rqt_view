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

        self.out_of_field_button = QtWidgets.QPushButton("Put all out of field")
        self.layout().addWidget(self.out_of_field_button)

        self.reset_view_button = QtWidgets.QPushButton("Reset view")
        self.layout().addWidget(self.reset_view_button)

        self.clear_debug_button = QtWidgets.QPushButton("Clear debug drawings")
        self.layout().addWidget(self.clear_debug_button)

        self.transformation_modal_button = QtWidgets.QPushButton("Config!")
        self.layout().addWidget(self.transformation_modal_button)
        self.transformation_modal_button.clicked.connect(self.open_field_transformation_modal)

        self.toggle_halt_button = QtWidgets.QPushButton("Press to halt")
        self.layout().addWidget(self.toggle_halt_button)

        # ---- /Widgets ----

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
