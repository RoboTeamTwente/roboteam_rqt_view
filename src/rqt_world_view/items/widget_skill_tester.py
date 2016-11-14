import subprocess32 as subprocess
import sys

from python_qt_binding import QtWidgets
from python_qt_binding.QtGui import QRegExpValidator
from python_qt_binding.QtCore import QRegExp, pyqtSignal

from widget_blackboard import WidgetBlackboard
from rqt_world_view.utils import utils

from roboteam_msgs import msg


ROSRUN = "rosrun"
TESTX_PACKAGE = "roboteam_tactics"
TESTX_COMMAND = "TestX"


class WidgetSkillTester(QtWidgets.QFrame):

    # Signal to be called when the test process exits.
    test_stopped_signal = pyqtSignal()


    def __init__(self, strategy_ignore_topic):
        """strategy_ignore_topic: rospy.Publisher -- Topic to notify strategy nodes on that they should ignore a robot."""
        super(WidgetSkillTester, self).__init__()

        self.strategy_ignore_topic = strategy_ignore_topic

        self.setLayout(QtWidgets.QGridLayout())

        self.setSizePolicy(self.sizePolicy().Preferred, self.sizePolicy().Fixed)

        # ---- Test button ----

        self.test_button = QtWidgets.QPushButton("Run test")
        self.test_button.clicked.connect(self.slot_test_button_pushed)
        self.layout().addWidget(self.test_button, 0, 0, 1, 3)

        # ---- /Test button ----

        self.id_label = QtWidgets.QLabel("Id")
        self.layout().addWidget(self.id_label, 1, 0)
        self.skill_label = QtWidgets.QLabel("Skill")
        self.layout().addWidget(self.skill_label, 1, 1, 1, 2)

        # ---- Id entry ----

        self.id_entry = QtWidgets.QLineEdit()
        self.id_entry.setValidator(QRegExpValidator(QRegExp("[0-9]+")))
        self.id_entry.setMinimumWidth(20)
        self.id_entry.setMaximumWidth(50)
        self.id_entry.setSizePolicy(self.id_entry.sizePolicy().Fixed, self.id_entry.sizePolicy().Fixed)
        self.layout().addWidget(self.id_entry, 2, 0)

        # ---- /Id entry ----

        self.skill_entry = QtWidgets.QLineEdit()
        self.layout().addWidget(self.skill_entry, 2, 1, 1, 2)

        self.blackboard = WidgetBlackboard()
        self.blackboard.layout().setContentsMargins(2, 2, 2, 5)
        self.blackboard.setFrameStyle(QtWidgets.QFrame.Panel | QtWidgets.QFrame.Sunken)
        self.layout().addWidget(self.blackboard, 3, 0, 1, 3)

        # ---- Process ----

        # Process handle for the TestX programm.
        self.testx_thread = None

        # The id of the robot we are currently testing.
        self.bot_test_id = None

        self.test_stopped_signal.connect(self.slot_on_test_exit)


    def remove_child_widgets(self):
        """Called before cleaning the tester up. Makes sure there are no child wigets floating around."""
        self.test_button.deleteLater()
        self.id_label.deleteLater()
        self.skill_label.deleteLater()
        self.id_entry.deleteLater()
        self.skill_entry.deleteLater()
        self.blackboard.deleteLater()
        self.remove_button.deleteLater()

        self.test_button = None
        self.id_label = None
        self.skill_label = None
        self.id_entry = None
        self.skill_entry = None
        self.blackboard = None
        self.remove_button = None

        self.deleteLater()


    def delete_self(self):
        """Called when the "x" button is pressed."""
        self.remove_child_widgets()

        if self.remove_callback != None and self.remove_id != None:
            self.remove_callback(self.remove_id)


    def add_remove_button(self, remove_callback, remove_id):
        """
        Creates an "x" button for removing the tester.
        Should be given a callback from the parent widget that will clean this widget up.
        Also needs an id to give to the callback, so that the parent widget knows which tester should be removed.
        """
        self.remove_button = QtWidgets.QPushButton("x")
        self.layout().addWidget(self.remove_button, 0, 2)
        self.remove_button.setMaximumWidth(20)
        self.remove_button.setSizePolicy(self.remove_button.sizePolicy().Preferred, self.remove_button.sizePolicy().Fixed)
        # Relocate the test button.
        self.layout().addWidget(self.test_button, 0, 0, 1, 2)

        self.remove_callback = remove_callback
        self.remove_id = remove_id

        self.remove_button.clicked.connect(self.delete_self)


    def change_ui_to_running(self):
        """Changes the widget to reflex that the test is running."""
        # Disable edit widgets.
        self.id_entry.setEnabled(False)
        self.skill_entry.setEnabled(False)

        self.blackboard.set_editable(False)

        # Change the test button text to something appropriate.
        self.test_button.setText("Stop test")


    def change_ui_to_idle(self):
        """Changes the widget to reflect that no test is running."""
        self.id_entry.setEnabled(True)
        self.skill_entry.setEnabled(True)

        self.blackboard.set_editable(True)

        # Change the test button text to something appropriate.
        self.test_button.setText("Run test")


    def toggle_testing_process(self):
        """
        Starts the test process when it isn't running.
        Stops the process when it is running.
        """
        if self.is_test_running():
            self.stop_test()
        else:
            self.start_test()


    def stop_test(self):
        """Stops the test when it's running."""
        if self.is_test_running():
            # Ask the testx thread to stop.
            self.testx_thread.stop()
            self.test_button.setText("Stopping...")


    def start_test(self):
        """Starts the test. Only when the test isn't already running."""
        if not self.is_test_running():
            # Construct the rosrun TestX command.
            command = [ROSRUN, TESTX_PACKAGE, TESTX_COMMAND]
            skill = str(self.skill_entry.text())
            command.append(skill)

            # Read the id of the bot to be tested.
            try:
                self.bot_test_id = int(self.id_entry.text())
            except ValueError:
                # Default to 0.
                self.bot_test_id = 0
                self.id_entry.clear()
                self.id_entry.insert("0")

            # Make any online strategy node ignore this robot.
            self.publish_strategy_ignore_command(self.bot_test_id, True)

            command.append("int:ROBOT_ID=" + str(self.bot_test_id))

            # Get the blackboard and add it to the command.
            blackboard = self.blackboard.get_blackboard_testx()
            command.extend(blackboard)

            print command

            # Start the test.
            self.testx_thread = utils.popen_and_call(self.callback_on_test_exit, command, stdout=sys.stdout)

            # Make the ui reflect the running state.
            self.change_ui_to_running()


    def publish_strategy_ignore_command(self, bot_id, ignore):
        """
        Publishes StrategyIgnoreRobot message on the strategy ignore topic.

        bot_id: integer -- The id of the robot in question.
        ignore: boolean -- Wether to ignore this robot or not.
        """
        command = msg.StrategyIgnoreRobot()
        command.id = bot_id
        command.ignore = ignore

        self.strategy_ignore_topic.publish(command)



    def is_test_running(self):
        """
        Checks whether the testx process is running.
        Returns true for running, false for not running or nonexistent.
        """
        if self.testx_thread:
            if self.testx_thread.isAlive():
                # The test is still running.
                return True
        # Fell through, test is not running.
        return False


    def callback_on_test_exit(self):
        """Callback that gets called when the test process exits."""
        self.test_stopped_signal.emit()


    def slot_on_test_exit(self):
        """Slot to be called when the test process exits."""
        self.testx_thread = None
        # Release controll of the robot.
        self.publish_strategy_ignore_command(self.bot_test_id, False)
        # Change the ui to reflect the stopped state.
        self.change_ui_to_idle()


    def slot_test_button_pushed(self):
        """This slot is called when the "Test" button is pressed."""
        self.toggle_testing_process()
