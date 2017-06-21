import robot_data


class RobotMap():

    def __init__(self):

        self.robots = dict()

        self.vision_status = False


    def get_vision_status(self):
        return self.vision_status


    def update_with_detections(self, detections):
        self.vision_status = True

        for bot in detections:
            if bot.id not in self.robots:
                self.create_new_robot(bot.id)

            self.robots[bot.id].update_with_detection(bot)

        for bot_id in self.robots:
            if bot_id not in [d.id for d in detections]:
                # Vision is lost for this bot.
                self.robots[bot_id].set_vision_lost()


    def update_with_serial_status(self, status):
        if status.id >= 0:
            if status.id not in self.robots:
                self.create_new_robot(status.id)

            self.robots[status.id].update_with_serial_status(status)


    def update_parameters_from_ros(self):
        for bot_id, bot in self.robots.items():
            bot.update_parameters_from_ros()


    def update_with_role_status(self, status):
        if status.bot_id >= 0:
            if status.bot_id not in self.robots:
                self.create_new_robot(status.bot_id)

            self.robots[status.bot_id].update_with_role_status(status)


    def create_new_robot(self, bot_id):
        """
        Create a new robot with id `bot_id` and add it to the robot map.
        Returns True on success, False when the bot already exists.
        """
        if bot_id in self.robots:
            return False
        else:
            new_bot = robot_data.RobotData(bot_id)
            self.robots[bot_id] = new_bot

            return True


    def get_robots(self):
        return self.robots


    def remove_robot(self, bot_id):
        del self.robots[bot_id]

    def change_all_robot_types(self, new_type):
        for bot_id, bot in self.robots.items():
            bot.change_robot_type(new_type)

    def detection_timed_out(self):
        self.vision_status = False
        for bot_id, bot in self.robots.items():
            bot.set_vision_lost()
