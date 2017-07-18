
class StrategyData():

    def __init__(self):
        self.interpreted_command = ""
        self.current_strategy = ""
        self.first_message_received = False
        self.plays = {}

    def update_with_strategy_debug_info(self, strategy_info):
        self.first_message_received = True
        self.interpreted_command = strategy_info.interpreted_ref_command
        self.current_strategy = strategy_info.current_strategy

        self.plays.clear()

        for play in strategy_info.active_plays:
            bots = []
            for bot in play.active_robots:
                bots.append(bot)
            self.plays[play.play_name] = bots

        print self.plays

    def get_interpreted_command(self):
        return self.interpreted_command

    def get_current_strategy(self):
        return self.current_strategy

    def is_first_message_received(self):
        return self.first_message_received
