import Utils, os, time
from pathlib import Path
from functools import partial
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from MapCanvas import *
from Road import *

INTERSECTION_WIDTH = 4
MAP_ELEMENTS = {
    "ROBOT": "robots",
    "DRUG STORES": "drugStores",
    "GARAGES": "garages",
    "RESTAURANTS": "restaurants",
    "DEPOTS": "depots",
    "ZONES": "zones",
}

class Map(QWidget):
    def __init__(self, parent=None):
        super(QWidget, self).__init__(parent) 
        self.setAttribute(Qt.WA_QuitOnClose, False)
        
        self.file_data = {}
        self.map_data = {}
        self.folder_path = ""

        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self.top_layout = QVBoxLayout()
        self.top_layout.setContentsMargins(10, 5, 10, 5)
        self.bottom_layout = QHBoxLayout()
        self.element_layout = QGridLayout()
        
        self.map_name = QLabel("")
        self.map_canvas = MapCanvas(self)
        
        self.robot_cb = QCheckBox("ROBOT", self)
        self.robot_cb.setChecked(False)
        self.robot_cb.stateChanged.connect(lambda: self.show_map_elements(self.robot_cb))
        self.drugstore_cb = QCheckBox("DRUG STORES", self)
        self.drugstore_cb.stateChanged.connect(lambda: self.show_map_elements(self.drugstore_cb))
        self.garage_cb = QCheckBox("GARAGES", self)
        self.garage_cb.stateChanged.connect(lambda: self.show_map_elements(self.garage_cb))
        self.restaurant_cb = QCheckBox("RESTAURANTS", self)
        self.restaurant_cb.stateChanged.connect(lambda: self.show_map_elements(self.restaurant_cb))
        self.depot_cb = QCheckBox("DEPOTS", self)
        self.depot_cb.stateChanged.connect(lambda: self.show_map_elements(self.depot_cb))
        self.zone_cb = QCheckBox("ZONES", self)
        self.zone_cb.stateChanged.connect(lambda: self.show_map_elements(self.zone_cb))
        self.element_layout.addWidget(QLabel("Structures :"), 0, 0, 1, 3)
        self.element_layout.addWidget(self.robot_cb, 1, 0)
        self.element_layout.addWidget(self.drugstore_cb, 1, 1)
        self.element_layout.addWidget(self.garage_cb, 1, 2)
        self.element_layout.addWidget(self.restaurant_cb, 2, 0)
        self.element_layout.addWidget(self.depot_cb, 2, 1)
        self.element_layout.addWidget(self.zone_cb, 2, 2)
        
        self.mainMenu_button = QPushButton("Exit")
        self.mainMenu_button.clicked.connect(self.exit)
        
        self.statusbar = QStatusBar()
        self.statusbar.showMessage("Ready")
        
        self.bottom_layout.addLayout(self.element_layout, stretch=8)
        self.bottom_layout.addWidget(self.mainMenu_button, stretch=2)
        self.top_layout.addWidget(self.map_name, stretch=1, alignment=Qt.AlignCenter)
        self.top_layout.addWidget(self.map_canvas, stretch=70)
        self.top_layout.addLayout(self.bottom_layout, stretch=29)
        self._layout.addLayout(self.top_layout)
        self._layout.addWidget(self.statusbar)

    def read_json(self, file_name):
        return Utils.read_json(file_name)
            
    def render_map(self, folder_path, file_name):
        self.folder_path = folder_path
        self.file_data = self.read_json(folder_path + file_name)
        self.map_data = self.read_json(self.folder_path + self.file_data["map"])
        self.map_name.setText(self.file_data["map"])
        self.map_canvas.render(self.map_data)

    def setMessage(self, msg):
        self.statusbar.showMessage(msg)

    def get_road_data(self, section_name):
        file_name = self.file_data["sections"][section_name]
        return file_name, Utils.read_json(self.folder_path+file_name)
        
    def show_road(self, road_name):
        path = os.path.abspath(__file__)
        path = str(Path(path).parent)
        setup_map_path = path + "/../map/setup_map.py"
        file_path = path + "/data/" + road_name
        setup_path = path + "/../setup.sh"

        # Handle debug mod
        if len(sys.argv) > 1:
            if sys.argv[1] == "-d":
                setup_path += " -d"

        os.system("python3 " + setup_map_path + " " + file_path)
        print("map switched to", road_name)

        os.system(setup_path + " &")
        print("setup success")

        self.road_window = Road()
        self.road_window.show()
        self.road_window.render_road(self.folder_path, road_name)
        
    def show_map_elements(self, cb):
        self.map_canvas.show_element(cb.text(), cb.isChecked())
        
    def exit(self):
        self.road_window = None
