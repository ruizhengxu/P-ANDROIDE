from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class Start(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        
        self.file_name = None
        title = QLabel("RoadMap Visualisor")
        title.setStyleSheet("font-size: 24px; font-weight: bold")
        
        self.layout = QGridLayout(self)
        self.layout.setAlignment(Qt.AlignCenter)
        
        self.import_builded = QPushButton("Import builded Map")
        
        self.import_new = QPushButton("Import new map")
        self.import_new.setDisabled(True)
        
        self.layout.addWidget(title, 0, 0, 1, 3)
        self.layout.addWidget(self.import_builded, 1, 0)
        self.layout.addWidget(self.import_new, 1, 1)
        self.layout.addWidget(QLabel("Version Beta"), 2, 2)