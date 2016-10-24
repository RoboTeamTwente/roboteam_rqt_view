from python_qt_binding.QtWidgets import QFrame, QVBoxLayout, QLabel


class WidgetRobotDetails(QFrame):

    def __init__(self, bot_id):
        super(WidgetRobotDetails, self).__init__()

        # Add the layout.
        self.setLayout(QVBoxLayout())

        self.bot_id = bot_id

        self.id_label = QLabel(str(self.bot_id))
        self.layout().addWidget(self.id_label)
