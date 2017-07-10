
from python_qt_binding import QtWidgets
from python_qt_binding import QtCore

from widget_robot_data import WidgetRobotData
from widget_strategy_data import WidgetStrategyData
from widget_all_robot_data import WidgetAllRobotData


class WidgetRobotList(QtWidgets.QFrame):

    def __init__(self, robot_map, config):
        super(WidgetRobotList, self).__init__()

        self.setLayout(QtWidgets.QVBoxLayout())

        self.robot_map = robot_map
        self.config = config

        self.data_widgets = dict()

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
        self.scroll_area.setMinimumWidth(100)

        self.scroll_area.setWidgetResizable(True)
        # self.scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.scroll_area.setFrameStyle(QtWidgets.QFrame.NoFrame)

        self.all_robot_widget = WidgetAllRobotData(self.robot_map.get_general_data(), self.config)
        self.scroll_layout.addWidget(self.all_robot_widget)

        self.strategy_data_widget = WidgetStrategyData(self.robot_map, self.config)
        self.scroll_layout.addWidget(self.strategy_data_widget)

        self.scroll_layout.addStretch(1)

        self.layout().addWidget(self.scroll_area)

        # ---- /Scroll area ----


    def update(self):
        self.all_robot_widget.update()

        for bot_id, bot in self.robot_map.get_robots().items():
            if bot_id not in self.data_widgets:
                new_widget = WidgetRobotData(bot, self.config)

                insert_position = self.scroll_layout.count() - 1

                self.scroll_layout.insertWidget(insert_position, new_widget)
                self.data_widgets[bot_id] = new_widget

                # Connect the close callback.
                new_widget.connect_delete_callback(self.remove_robot)

            self.data_widgets[bot_id].update()


    def remove_robot(self, bot_id):
        self.data_widgets[bot_id].deleteLater()
        del self.data_widgets[bot_id]
        self.robot_map.remove_robot(bot_id)
