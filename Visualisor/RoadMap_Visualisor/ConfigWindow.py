from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class ConfigWindow(QWidget):
    def __init__(self, road_name, road_data, MAP, parent=None):
        super(QWidget, self).__init__(parent)
        self.setAttribute(Qt.WA_QuitOnClose, False)

        self.MAP = MAP
        self.road_name = road_name
        self.road_data = road_data
        self._layout = QGridLayout(self)

        self.minSpd_input = QLineEdit()
        self.minSpd_input.setText("1")
        self.minSpd_input.setValidator(QIntValidator())
        self.maxSpd_input = QLineEdit()
        self.maxSpd_input.setText("10")
        self.maxSpd_input.setValidator(QIntValidator())

        self.validate_btn = QPushButton("Validate")
        self.validate_btn.clicked.connect(self.validate)
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.clicked.connect(self.cancel)

        self._layout.addWidget(QLabel("Simulation parameters of " + road_name), 0, 0, 1, 2)
        self._layout.addWidget(QLabel("Min Speed"), 1, 0)
        self._layout.addWidget(self.minSpd_input, 1, 1)
        self._layout.addWidget(QLabel("Max Speed"), 2, 0)
        self._layout.addWidget(self.maxSpd_input, 2, 1)
        self._layout.addWidget(self.validate_btn, 3, 0)
        self._layout.addWidget(self.cancel_btn, 3, 1)

    def validate(self):
        minSpeed = int(self.minSpd_input.text())
        maxSpeed = int(self.maxSpd_input.text())
        error = False
        error_text = ""
        
        if minSpeed < 0 or maxSpeed < 0:
            error = True
            error_text += "Speeds should not be lower than 0 !\n"
        if minSpeed > maxSpeed:
            error = True
            error_text += "Minimun speed should be lower than maximum speed !\n"

        if error:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle("Error in parameters")
            msg.setText("Please verify your parameters")
            msg.setInformativeText(error_text)
            retval = msg.exec_()

        self.MAP.show_road(self.road_name)
        self.close()

    def cancel(self):
        self.close()
