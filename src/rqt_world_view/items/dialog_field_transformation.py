
import rospy

from python_qt_binding import QtWidgets, QtGui, QtCore


PARAM_GROUP = "transform_field/"


class DialogFieldTransformation(QtWidgets.QDialog):

    def __init__(self):
        super(DialogFieldTransformation, self).__init__()

        self.setLayout(QtWidgets.QGridLayout())
        self.setWindowTitle("Field transformation")

        # ---- Wigets ----

        self.enabled_checkbox = QtWidgets.QCheckBox("Enable transformation")
        self.layout().addWidget(self.enabled_checkbox, 0, 0, 1, 4)

        self.rotate_checkbox = QtWidgets.QCheckBox("Rotate 90 degrees")
        self.layout().addWidget(self.rotate_checkbox, 1, 0, 1, 4)

        self.top_label = QtWidgets.QLabel("Top")
        self.layout().addWidget(self.top_label, 2, 0)

        self.top_edit = QtWidgets.QDoubleSpinBox()
        self.top_edit.setMinimum(-99.99)
        self.layout().addWidget(self.top_edit, 2, 1)

        self.bottom_label = QtWidgets.QLabel("Bottom")
        self.layout().addWidget(self.bottom_label, 2, 2)

        self.bottom_edit = QtWidgets.QDoubleSpinBox()
        self.bottom_edit.setMinimum(-99.99)
        self.layout().addWidget(self.bottom_edit, 2, 3)

        self.left_label = QtWidgets.QLabel("Left")
        self.layout().addWidget(self.left_label, 3, 0)

        self.left_edit = QtWidgets.QDoubleSpinBox()
        self.left_edit.setMinimum(-99.99)
        self.layout().addWidget(self.left_edit, 3, 1)

        self.right_label = QtWidgets.QLabel("Right")
        self.layout().addWidget(self.right_label, 3, 2)

        self.right_edit = QtWidgets.QDoubleSpinBox()
        self.right_edit.setMinimum(-99.99)
        self.layout().addWidget(self.right_edit, 3, 3)

        self.apply_button = QtWidgets.QPushButton("Apply")
        self.layout().addWidget(self.apply_button, 4, 2, 1, 2)
        self.apply_button.clicked.connect(self.apply_config)

        # ---- /Wigets ----

    def apply_config(self):
        rospy.set_param(PARAM_GROUP + "enabled", self.enabled_checkbox.isChecked())
        rospy.set_param(PARAM_GROUP + "rotate", self.rotate_checkbox.isChecked())
        rospy.set_param(PARAM_GROUP + "offset/top", self.top_edit.value())
        rospy.set_param(PARAM_GROUP + "offset/bottom", self.bottom_edit.value())
        rospy.set_param(PARAM_GROUP + "offset/left", self.left_edit.value())
        rospy.set_param(PARAM_GROUP + "offset/right", self.right_edit.value())
