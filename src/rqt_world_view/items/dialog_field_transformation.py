
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

        self.reset_button = QtWidgets.QPushButton("Reset")
        self.layout().addWidget(self.reset_button, 4, 0, 1, 2)
        self.reset_button.clicked.connect(self.reset_config)

        # ---- /Wigets ----

    def show(self):
        super(DialogFieldTransformation, self).show()

        enabled = rospy.get_param(PARAM_GROUP + "enabled", False)
        if isinstance(enabled, bool):
            self.enabled_checkbox.setChecked(enabled)

        rotate = rospy.get_param(PARAM_GROUP + "rotate", False)
        if isinstance(rotate, bool):
            self.rotate_checkbox.setChecked(rotate)

        top = rospy.get_param(PARAM_GROUP + "offset/top", 0.0)
        try:
            top = float(top)
        except ValueError:
            top = 0.0
        self.top_edit.setValue(top)

        bottom = rospy.get_param(PARAM_GROUP + "offset/bottom", 0.0)
        try:
            bottom = float(bottom)
        except ValueError:
            bottom = 0.0
        self.bottom_edit.setValue(bottom)

        left = rospy.get_param(PARAM_GROUP + "offset/left", 0.0)
        try:
            left = float(left)
        except ValueError:
            left = 0.0
        self.left_edit.setValue(left)

        right = rospy.get_param(PARAM_GROUP + "offset/right", 0.0)
        try:
            right = float(right)
        except ValueError:
            right = 0.0
        self.right_edit.setValue(right)

    def apply_config(self):
        rospy.set_param(PARAM_GROUP + "enabled", self.enabled_checkbox.isChecked())
        rospy.set_param(PARAM_GROUP + "rotate", self.rotate_checkbox.isChecked())
        rospy.set_param(PARAM_GROUP + "offset/top", self.top_edit.value())
        rospy.set_param(PARAM_GROUP + "offset/bottom", self.bottom_edit.value())
        rospy.set_param(PARAM_GROUP + "offset/left", self.left_edit.value())
        rospy.set_param(PARAM_GROUP + "offset/right", self.right_edit.value())

    def reset_config(self):
        self.enabled_checkbox.setChecked(False)
        self.rotate_checkbox.setChecked(False)

        self.top_edit.setValue(0.0)
        self.bottom_edit.setValue(0.0)
        self.left_edit.setValue(0.0)
        self.right_edit.setValue(0.0)

        self.apply_config()
