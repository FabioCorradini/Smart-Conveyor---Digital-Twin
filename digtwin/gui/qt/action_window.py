from PyQt5 import QtWidgets
from digtwin.gui.models.dt_models import DTStatefulModel
from PyQt5.QtCore import Qt


class DTActionWindow(QtWidgets.QWidget):
    def __init__(self,model: DTStatefulModel, parent=None):
        super().__init__(parent)
        self.model = model
        self.setWindowTitle(model.name)
        self.setWindowFlag(Qt.Window, True)
        self.setMaximumSize(200, 100)
        v_box = QtWidgets.QVBoxLayout()
        title_label = QtWidgets.QLabel()
        title_label.setAlignment(Qt.AlignLeft)
        title_label.setStyleSheet("QLabel { font-weight: bold; }")
        title_label.setTextFormat(Qt.PlainText)
        title_label.setText(model.name)
        v_box.addWidget(title_label)

        if len(model) == 2:  # if only 2 state I can use a checkbox
            check_box = QtWidgets.QCheckBox()
            check_box.setText("Active")
            check_box.toggled.connect(self.toggle_state)
            v_box.addWidget(check_box)

        self.setLayout(v_box)

    def toggle_state(self):
        if self.model.state_id == 0:
            self.model.state_id = 1
        else:
            self.model.state_id = 0

        self.model.to_state()


