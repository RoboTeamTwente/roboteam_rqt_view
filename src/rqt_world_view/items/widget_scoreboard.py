from python_qt_binding.QtWidgets import QFrame, QGridLayout, QLabel

from roboteam_msgs.msg import RefereeStage as Refs


# Scoreboard widget.
class WidgetScoreboard(QFrame):

    # us_color: QColor() => The color to use for our team info.
    # them_color: QColor() => The color to use for their team info.
    def __init__(self, us_color, them_color):
        super(WidgetScoreboard, self).__init__()

        self.setLayout(QGridLayout())

        self.us_info = WidgetTeamInfo(us_color)
        self.them_info = WidgetTeamInfo(them_color)

        self.stage = QLabel("Stage")
        self.stage_time_left = QLabel(millis_to_human_readable(0))

        # ---- Layout ----

        self.layout().addWidget(self.us_info, 0, 0, 1, 2)
        self.layout().addWidget(self.them_info, 0, 2, 1, 2)

        self.layout().addWidget(self.stage, 0, 4)
        self.layout().addWidget(self.stage_time_left, 0, 5)


    # Updates the score board with info from a message.
    # Expects a RefereeData message.
    def update_with_message(self, message):

        self.us_info.update_with_message(message.us)
        self.them_info.update_with_message(message.them)

        self.stage.setText(stage_to_string(message.stage))
        self.stage_time_left.setText(millis_to_human_readable(message.stage_time_left))


# Team info widget.
class WidgetTeamInfo(QFrame):

    # color: QColor() => The color to use for the border.
    def __init__(self, color):
        super(WidgetTeamInfo, self).__init__()

        self.setLayout(QGridLayout())

        self.setObjectName("score")
        self.setStyleSheet("#score { border: 3px solid rgb(" + str(color.red()) + "," + str(color.green()) + "," + str(color.blue()) + "); }")

        self.name = QLabel("NoName")
        self.score = QLabel(str(0))

        self.layout().addWidget(self.name, 0, 0)
        self.layout().addWidget(self.score, 0, 1)

    # Updates the score board with info from a message.
    # info => RefereeTeamInfoMessage
    def update_with_message(self, info):
        self.name.setText(info.name)
        self.score.setText(str(info.score))


# Converts a RefereeStage message into a string readable by humans.
def stage_to_string(stage):
    mapping = {
        Refs.NORMAL_FIRST_HALF_PRE: "Pre game",
        Refs.NORMAL_FIRST_HALF: "First half",
        Refs.NORMAL_HALF_TIME: "Half time",
        Refs.NORMAL_SECOND_HALF_PRE: "Pre second half",
        Refs.NORMAL_SECOND_HALF: "Second half",
        Refs.EXTRA_TIME_BREAK: "Overtime: Break",
        Refs.EXTRA_FIRST_HALF_PRE: "Overtime: Pre first half",
        Refs.EXTRA_FIRST_HALF: "Overtime: First half",
        Refs.EXTRA_HALF_TIME: "Overtime: Half time",
        Refs.EXTRA_SECOND_HALF_PRE: "Overtime: Pre second half",
        Refs.EXTRA_SECOND_HALF: "Overtime: Second half",
        Refs.PENALTY_SHOOTOUT_BREAK: "Penalty shootout break",
        Refs.PENALTY_SHOOTOUT: "Penalty shootout",
        Refs.POST_GAME: "Post game"
    }

    return mapping.get(stage.stage, "Strange stage")

# Converts a value in milliseconds (integer) into a human readable string.
# e.g. "12:4"
def millis_to_human_readable(millis):
    s=millis/1000000
    m,s=divmod(s,60)

    return str(m) + ":" + str(s)
