
from python_qt_binding import QtWidgets
from python_qt_binding.QtCore import Qt

from widget_skill_tester import WidgetSkillTester


class WidgetMultiSkillTester(QtWidgets.QFrame):

    # strategy_ignore_topic: rospy.Publisher => Topic to notify strategy nodes on that they should ignore a robot.
    def __init__(self, strategy_ignore_topic):
        super(WidgetMultiSkillTester, self).__init__()

        self.strategy_ignore_topic = strategy_ignore_topic

        self.setLayout(QtWidgets.QGridLayout())

        self.testers = []

        # ---- Start and stop all buttons ----

        self.start_all_button = QtWidgets.QPushButton("Start all")
        self.start_all_button.clicked.connect(self.slot_start_all)
        self.layout().addWidget(self.start_all_button, 0, 0)

        self.stop_all_button = QtWidgets.QPushButton("Stop all")
        self.stop_all_button.clicked.connect(self.slot_stop_all)
        self.layout().addWidget(self.stop_all_button, 0, 1)

        # ---- /Start and stop all buttons ----

        # ---- Add tester button ----

        self.add_tester_button = QtWidgets.QPushButton("Add tester")
        self.add_tester_button.clicked.connect(self.slot_add_tester)
        self.layout().addWidget(self.add_tester_button, 1, 0, 1, 2)

        # ---- /Add tester button ----

        # ---- Scroll area ----

        self.scroll_area = QtWidgets.QScrollArea()
        self.scroll_frame = QtWidgets.QFrame();
        self.scroll_layout = QtWidgets.QVBoxLayout(self.scroll_frame)
        self.scroll_area.setWidget(self.scroll_frame)

        self.scroll_area.setContentsMargins(0, 0, 0, 0)
        self.scroll_frame.setContentsMargins(0, 0, 0, 0)
        self.scroll_layout.setContentsMargins(0, 0, 5, 0)

        self.scroll_area.setSizePolicy(self.scroll_frame.sizePolicy().MinimumExpanding, self.scroll_frame.sizePolicy().Expanding)
        # This shouldn't be necessary.
        # A scroll area with no horizontal scroll bar should automatically resize to the minimum size of it's contents.
        # It doesn't, so this arbitrary width is needed.
        self.scroll_area.setMinimumWidth(300)

        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll_area.setFrameStyle(QtWidgets.QFrame.NoFrame)

        self.scroll_layout.addStretch(1)

        self.layout().addWidget(self.scroll_area, 2, 0, 1, 2)

        # ---- /Scroll area ----

# ------------------------------------------------------------------------------
# ---------- Button slots ------------------------------------------------------
# ------------------------------------------------------------------------------

    # Gets called when the "Add tester" button is clicked.
    def slot_add_tester(self):
        tester = WidgetSkillTester(self.strategy_ignore_topic)

        tester.setFrameStyle(QtWidgets.QFrame.Panel)

        self.scroll_layout.insertWidget(0, tester)
        self.testers.append(tester)

    def slot_start_all(self):
        for tester in self.testers:
            tester.start_test()

    def slot_stop_all(self):
        for tester in self.testers:
            tester.stop_test()
