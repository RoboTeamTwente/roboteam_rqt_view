from python_qt_binding.QtWidgets import QFrame, QVBoxLayout, QLabel
from python_qt_binding.QtCore import pyqtSignal


class WidgetRobotDetails(QFrame):

    # Signal to fire when this frame gets selected or deselected.
    change_bot_selection = pyqtSignal(int, bool)

    def __init__(self, bot_id):
        super(WidgetRobotDetails, self).__init__()

        self.bot_id = bot_id
        self.is_selected = False

        # Add the vertical layout.
        self.setLayout(QVBoxLayout())
        self.setFrameStyle(QFrame.Panel | QFrame.Raised)

        # Bot id label.
        self.id_label = QLabel(str(self.bot_id))
        self.layout().addWidget(self.id_label)


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
