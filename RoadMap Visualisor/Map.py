import Utils
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from MapCanvas import *
from Road import *

INTERSECTION_WIDTH = 4

class Map(QWidget):
    def __init__(self, parent=None):
        super(QWidget, self).__init__(parent)
        
        self.file_data = {}
        self.map_data = {}
        self.folder_path = ""

        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self.top_layout = QVBoxLayout()
        self.top_layout.setContentsMargins(10, 5, 10, 5)
        self.bottom_layout = QHBoxLayout()
        self.legend_layout = QGridLayout()
        
        self.map_name = QLabel("")
        self.map_canvas = MapCanvas(self)
        
        self.legend_layout.addWidget(QLabel("Legends :"), 0, 0, 1, 3)
        self.mainMenu_button = QPushButton("Exit")
        self.statusbar = QStatusBar()
        self.statusbar.showMessage("Ready")
        
        self.bottom_layout.addLayout(self.legend_layout, stretch=8)
        self.bottom_layout.addWidget(self.mainMenu_button, stretch=2)
        self.top_layout.addWidget(self.map_name, stretch=1, alignment=Qt.AlignCenter)
        self.top_layout.addWidget(self.map_canvas, stretch=70)
        self.top_layout.addLayout(self.bottom_layout, stretch=29)
        self._layout.addLayout(self.top_layout)
        self._layout.addWidget(self.statusbar)
        
        # Road Window
        self.road_window = Road()
            
    def read_json(self, file_name):
        return Utils.read_json(file_name)
            
    def render_map(self, folder_path, file_name):
        self.folder_path = folder_path
        file_data = self.read_json(folder_path+file_name)
        map_data = self.read_json(folder_path+self.file_data["map"])
        
        self.map_name.setText(file_data["map"])
        self.map_canvas.render(map_data)
        
        self.road_window.show()

    def setMessage(self, msg):
        self.statusbar.showMessage(msg)
        
    def show_road(self, section_name):
        file_name = self.file_data["sections"][section_name]
        self.road_window.render_road(self.folder_path, file_name, section_name)
