from python_qt_binding.QtWidgets import QFrame, QLabel, QGridLayout, QPushButton


class WidgetBlackboard(QFrame):

    def __init__(self):
        super(WidgetBlackboard, self).__init__()

        self.setLayout(QGridLayout())

        self.layout().addWidget(QLabel("Type"), 0, 0)
        self.layout().addWidget(QLabel("Name"), 0, 1)
        self.layout().addWidget(QLabel("Value"), 0, 2)


        # ---- Add item button ----
        self.add_item_button = QPushButton("Add value")
        self.layout().addWidget(self.add_item_button, 1, 0, 1, 4)
        self.add_item_button.clicked.connect(self.slot_add_item)


    def slot_add_item(self):
        print "Whoop!"
