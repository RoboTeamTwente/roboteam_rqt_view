import json
import sys


class PluginConfig:

    def __init__(self):
        self.pattern_visible = True
        self.icons_visible = True
        self.value_display_visible = False
        self.robot_settings = True
        self.robot_role_visible = True
        self.referee_data_visible = True

        self.rolenode_stop_callback = None

    def set_pattern_visible(self, is_visible):
        self.pattern_visible = is_visible

    def is_pattern_visible(self):
        return self.pattern_visible

    def set_icons_visible(self, is_visible):
        self.icons_visible = is_visible

    def is_icons_visible(self):
        return self.icons_visible

    def set_value_display_visible(self, is_visible):
        self.value_display_visible = is_visible

    def is_value_display_visible(self):
        return self.value_display_visible

    def set_robot_settings_visible(self, is_visible):
        self.robot_settings = is_visible

    def is_robot_settings_visible(self):
        return self.robot_settings

    def set_robot_role_visible(self, is_visible):
        self.robot_role_visible = is_visible

    def is_robot_role_visible(self):
        return self.robot_role_visible

    def set_referee_data_visible(self, is_visible):
        self.referee_data_visible = is_visible

    def is_referee_data_visible(self):
        return self.referee_data_visible

    def set_stop_callback(self, callback):
        self.rolenode_stop_callback = callback

    def get_stop_callback(self):
        return self.rolenode_stop_callback

    def to_json(self):
        return json.dumps((self.pattern_visible, self.icons_visible, self.value_display_visible, self.robot_settings, self.robot_role_visible, self.referee_data_visible))

    def from_json(self, json_string):
        try:
            settings = json.loads(json_string)
            (self.pattern_visible, self.icons_visible, self.value_display_visible, self.robot_settings, self.robot_role_visible, self.referee_data_visible) = settings
        except ValueError, TypeError:
            print >> sys.stderr, "Warning: Status display couldn't load configuration: \"" + json_string + "\""
