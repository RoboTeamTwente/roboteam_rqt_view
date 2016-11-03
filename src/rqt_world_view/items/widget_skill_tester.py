import subprocess32 as subprocess
import sys

from python_qt_binding import QtWidgets
from python_qt_binding.QtGui import QRegExpValidator
from python_qt_binding.QtCore import QRegExp

from widget_blackboard import WidgetBlackboard


ROSRUN = "rosrun"
TESTX_PACKAGE = "roboteam_tactics"
TESTX_COMMAND = "TestX"


class WidgetSkillTester(QtWidgets.QFrame):


    def __init__(self):
        super(WidgetSkillTester, self).__init__()

        self.setLayout(QtWidgets.QGridLayout())

        self.setSizePolicy(self.sizePolicy().Preferred, self.sizePolicy().Fixed)

        # ---- Test button ----

        self.test_button = QtWidgets.QPushButton("Test!")
        self.test_button.clicked.connect(self.slot_test_button_pushed)
        self.layout().addWidget(self.test_button, 0, 0, 1, 2)

        # ---- /Test button ----

        self.layout().addWidget(QtWidgets.QLabel("Id"), 1, 0)
        self.layout().addWidget(QtWidgets.QLabel("Skill"), 1, 1)

        # ---- Id entry ----

        self.id_entry = QtWidgets.QLineEdit()
        self.id_entry.setValidator(QRegExpValidator(QRegExp("[0-9]+")))
        self.id_entry.setMinimumWidth(20)
        self.id_entry.setMaximumWidth(50)
        self.id_entry.setSizePolicy(self.id_entry.sizePolicy().Fixed, self.id_entry.sizePolicy().Fixed)
        self.layout().addWidget(self.id_entry, 2, 0)

        # ---- /Id entry ----

        self.skill_entry = QtWidgets.QLineEdit()
        self.layout().addWidget(self.skill_entry, 2, 1)

        self.blackboard = WidgetBlackboard()
        self.blackboard.layout().setContentsMargins(2, 2, 2, 5)
        self.blackboard.setFrameStyle(QtWidgets.QFrame.Panel | QtWidgets.QFrame.Sunken)
        self.layout().addWidget(self.blackboard, 3, 0, 1, 2)

        # ---- Process ----

        self.testx_process = None


    # This slot is called when the "Test" button is pressed.
    def slot_test_button_pushed(self):
        self.toggle_testing_process()

    # Starts the test process when it isn't running.
    # Stops the process when it is running.
    def toggle_testing_process(self):
        if self.is_test_running():
            # Stop the process.
            self.testx_process.terminate()
        else:
            # Construct the rosrun TestX command.
            command = [ROSRUN, TESTX_PACKAGE, TESTX_COMMAND]
            skill = str(self.skill_entry.text())
            command.append(skill)

            # Add the robot id to the command.
            try:
                bot_id = str(int(self.id_entry.text()))
            except ValueError:
                bot_id = "0"

            command.append("int:ROBOT_ID=" + bot_id)

            # Get the blackboard and add it to the command.
            blackboard = self.blackboard.get_blackboard_testx()
            command.extend(blackboard)

            print command

            # Start the test.
            self.testx_process = subprocess.Popen(command, stdout=sys.stdout)


    # Checks whether the testx process is running.
    # Returns true for running, false for not running or nonexistent.
    def is_test_running(self):
        if self.testx_process:
            if self.testx_process.poll() == None:
                # The test is still running.
                return True
        # Fell through, test is not running.
        return False
