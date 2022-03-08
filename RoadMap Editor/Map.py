import json
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from Canvas import *

INTERSECTION_WIDTH = 4

class Map(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        
        self.map_data = {}

        self.layout = QVBoxLayout(self)
        
        self.canvas = Canvas()
        
        self.test_button = QPushButton("Exit")
        
        self.layout.addWidget(self.canvas)
        self.layout.addWidget(self.test_button)
    
    def read_map(self, file_name):
        # Read file
        try:
            f = open(file_name, "r")
            self.map_data = json.load(f)
            f.close()
        except Exception as e:
            print("Cannot read selected file :", e.__doc__)
            
    def render_map(self, file_name):
        self.read_map(file_name)
        self.canvas.draw(self.map_data)