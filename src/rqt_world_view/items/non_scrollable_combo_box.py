from python_qt_binding.QtWidgets import QComboBox
from python_qt_binding.QtCore import Qt

# A QComboBox that cannot be changed by scrolling
# Normally on mouseover, the user can scroll through
# the options in the combobox. In rqt this can easily
# screw up the command when scrolling through the list
# of commands. That is why we prefer to disable it.
class NonScrollableQComboBox(QComboBox):

	def __init__(self, *args, **kwargs):
		super(NonScrollableQComboBox, self).__init__(*args, **kwargs)
		self.setFocusPolicy(Qt.StrongFocus)

	def wheelEvent(self, *args, **kwargs):
		# pass the event to the parent so that the box does not block scrolling alltogether
		return self.parent().wheelEvent(*args, **kwargs)
