from python_qt_binding import QtWidgets
from python_qt_binding.QtGui import QRegExpValidator
from python_qt_binding.QtCore import QRegExp

from widget_blackboard import WidgetBlackboard


class WidgetSkillTester(QtWidgets.QFrame):


    def __init__(self):
        super(WidgetSkillTester, self).__init__()

        self.setLayout(QtWidgets.QGridLayout())

        self.setSizePolicy(self.sizePolicy().Preferred, self.sizePolicy().Fixed)

        self.test_button = QtWidgets.QPushButton("Test!")
        self.test_button.clicked.connect(self.slot_start_test)
        self.layout().addWidget(self.test_button, 0, 0, 1, 2)

        self.layout().addWidget(QtWidgets.QLabel("Id"), 1, 0)
        self.layout().addWidget(QtWidgets.QLabel("Skill"), 1, 1)

        self.id_entry = QtWidgets.QLineEdit()
        self.id_entry.setValidator(QRegExpValidator(QRegExp("[0-9]+")))
        self.id_entry.setMinimumWidth(20)
        self.id_entry.setMaximumWidth(50)
        self.id_entry.setSizePolicy(self.id_entry.sizePolicy().Fixed, self.id_entry.sizePolicy().Fixed)
        self.layout().addWidget(self.id_entry, 2, 0)

        self.skill_entry = QtWidgets.QLineEdit()
        self.layout().addWidget(self.skill_entry, 2, 1)

        self.blackboard = WidgetBlackboard()
        self.layout().addWidget(self.blackboard, 3, 0, 1, 2)


    # This slot is called when the "Test" button is pressed.
    def slot_start_test(self):
        print "--------------"
        print self.blackboard.get_blackboard_message()
