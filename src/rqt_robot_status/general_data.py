import utils.referee as referee


class GeneralData():

    def __init__(self):
        self.vision_status = False
        self.refbox_stage = ""
        self.refbox_command = ""


    def update_with_refbox_message(self, message):
        self.refbox_stage = referee.stage_to_string(message.stage)
        self.refbox_command = referee.command_to_string(message.command)


    def get_vision_status(self):
        return self.vision_status

    def get_refbox_stage(self):
        return self.refbox_stage

    def get_refbox_command(self):
        return self.refbox_command
