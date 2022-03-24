from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class Start(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        
        self.file_name = None
        
        self.layout = QGridLayout(self)
        self.layout.setAlignment(Qt.AlignCenter)
        
        self.create_button = QPushButton("Create Map")
        self.create_button.setDisabled(True)
        
        self.import_button = QPushButton("Import Map")
        self.import_button.clicked.connect(self.get_file)
        
        self.layout.addWidget(QLabel("RoadMap Editor v2 Beta 0.1"), 0, 0, 1, 2)
        self.layout.addWidget(self.create_button, 1, 0)
        self.layout.addWidget(self.import_button, 1, 1)
        
    def get_file(self):
        self.file_name = QFileDialog.getOpenFileName(self, "Select JSON File", "", "JSON files (*.json)")[0]