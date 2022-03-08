from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class Start(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        
        self.file_name = None
        
        self.layout = QVBoxLayout(self)
        
        self.import_button = QPushButton("Test")
        self.import_button.clicked.connect(self.get_file)
        
        self.layout.addWidget(self.import_button)
        
    def get_file(self):
        self.file_name = QFileDialog.getOpenFileName(self, "Select JSON File", "", "JSON files (*.json)")[0]