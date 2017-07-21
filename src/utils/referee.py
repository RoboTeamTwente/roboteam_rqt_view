from roboteam_msgs.msg import RefereeStage as Refs
from roboteam_msgs.msg import RefereeCommand as Refc


STAGE_MAPPING = {
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

COMMAND_MAPPING = {
    Refc.HALT: "Halt",
    Refc.STOP: "Stop",
    Refc.NORMAL_START: "Normal start",
    Refc.FORCE_START: "Force start",
    Refc.PREPARE_KICKOFF_US: "Prepare kickoff us",
    Refc.PREPARE_KICKOFF_THEM: "Prepare kickoff them",
    Refc.PREPARE_PENALTY_US: "Prepare penalty us",
    Refc.PREPARE_PENALTY_THEM: "Prepare penalty them",
    Refc.DIRECT_FREE_US: "Direct free us",
    Refc.DIRECT_FREE_THEM: "Direct free them",
    Refc.INDIRECT_FREE_US: "Indirect free us",
    Refc.INDIRECT_FREE_THEM: "Indirect free them",
    Refc.TIMEOUT_US: "Timeout us",
    Refc.TIMEOUT_THEM: "Timeout them",
    Refc.GOAL_US: "Goal us",
    Refc.GOAL_THEM: "Goal them",
    Refc.BALL_PLACEMENT_US: "Ball placement us",
    Refc.BALL_PLACEMENT_THEM: "Ball placement them",
}

# Converts a RefereeStage message into a string readable by humans.
def stage_to_string(stage):
    return STAGE_MAPPING.get(stage.stage, "Strange stage")

def command_to_string(command):
    return COMMAND_MAPPING.get(command.command, "Strange command")
