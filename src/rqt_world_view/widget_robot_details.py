from python_qt_binding.QtWidgets import QFrame, QGridLayout, QLabel
from python_qt_binding.QtCore import pyqtSignal, Qt


class WidgetRobotDetails(QFrame):

    # Signal to fire when this frame gets selected or deselected.
    change_bot_selection = pyqtSignal(int, bool)

    def __init__(self, bot_id):
        super(WidgetRobotDetails, self).__init__()

        self.bot_id = bot_id
        self.is_selected = False

        # Configure the frame appearance.
        self.setLayout(QGridLayout())
        self.set_selected_state(self.is_selected)

        # Bot id label.
        self.id_label = QLabel("<p style=\"font-weight:600\">" + \
                                str(self.bot_id) + "</p>")
        self.id_label.setTextFormat(Qt.RichText)
        self.layout().addWidget(self.id_label, 0, 0)

        # Position labels.
        self.xpos_label = QLabel(str(0.0))
        self.layout().addWidget(QLabel("x: "), 1, 0)
        self.layout().addWidget(self.xpos_label, 1, 1)
        self.ypos_label = QLabel(str(0.0))
        self.layout().addWidget(QLabel("y: "), 2, 0)
        self.layout().addWidget(self.ypos_label, 2, 1)


    # Update the info on display using a bot message.
    def update_display(self, bot_msg):
        self.xpos_label.setText(str("%.2f" % bot_msg.pos.x))
        self.ypos_label.setText(str("%.2f" % bot_msg.pos.y))


    # Sets the selection state of the frame.
    # Then calls `set_selected_appearance`.
    def set_selected_state(self, is_selected):
        self.is_selected = is_selected
        self.set_selected_appearance(is_selected)


    # Changes the appearance to be selected or not.
    # is_selected: boolean => wether the frame is selected or not.
    def set_selected_appearance(self, is_selected):
        if is_selected:
            self.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        else:
            self.setFrameStyle(QFrame.Panel | QFrame.Raised)

# ------------------------------------------------------------------------------
# ---------- Events ------------------------------------------------------------
# ------------------------------------------------------------------------------

    def mousePressEvent(self, mouse_event):
        # Fire the signal.
        # In `world_view.py` the apropriate bot will be selected
        # and this frames `set_selected_state` will be called.
        self.change_bot_selection.emit(self.bot_id, not self.is_selected)
