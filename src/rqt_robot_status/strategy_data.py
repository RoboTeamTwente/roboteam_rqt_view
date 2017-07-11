
class StrategyData():

    def __init__(self):
        self.interpreted_command = ""
        self.current_strategy = ""

    def update_with_strategy_debug_info(self, strategy_info):
        self.interpreted_command = strategy_info.interpreted_ref_command
        self.current_strategy = strategy_info.current_strategy

    def get_interpreted_command(self):
        return self.interpreted_command

    def get_current_strategy(self):
        return self.current_strategy
