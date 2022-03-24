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

        self.layout = QGridLayout(self)
        
        self.canvas = Canvas(self)
        
        self.legend_layout = QGridLayout(self)
        self.legend_layout.addWidget(QLabel("Legends :"), 0, 0, 1, 3)
        
        self.drugstore_legend = QPushButton("DRUG STORE")
        self.drugstore_legend.setDisabled(True)
        self.drugstore_legend.setStyleSheet("background-color : green; color: black")
        self.legend_layout.addWidget(self.drugstore_legend, 1, 0)
        
        self.depot_legend = QPushButton("DEPOTS")
        self.depot_legend.setDisabled(True)
        self.depot_legend.setStyleSheet("background-color : magenta; color: black")
        self.legend_layout.addWidget(self.depot_legend, 1, 1)
        
        self.garage_legend = QPushButton("GARAGES")
        self.garage_legend.setDisabled(True)
        self.garage_legend.setStyleSheet("background-color : cyan; color: black")
        self.legend_layout.addWidget(self.garage_legend, 1, 2)
        
        self.retaurant_legend = QPushButton("RETAURANTS")
        self.retaurant_legend.setDisabled(True)
        self.retaurant_legend.setStyleSheet("background-color : yellow; color: black")
        self.legend_layout.addWidget(self.retaurant_legend, 2, 0)
        
        self.zone_legend = QPushButton("ZONES")
        self.zone_legend.setDisabled(True)
        self.zone_legend.setStyleSheet("background-color : lightgray; color: black")
        self.legend_layout.addWidget(self.zone_legend, 2, 1)
        
        self.robot_legend = QPushButton("ROBOT")
        self.robot_legend.setDisabled(True)
        self.robot_legend.setStyleSheet("background-color : white; color: black")
        self.legend_layout.addWidget(self.robot_legend, 2, 2)
        
        self.test_button = QPushButton("Exit")
        
        self.layout.addWidget(self.canvas, 0, 0, 1, 2)
        self.layout.addLayout(self.legend_layout, 1, 0)
        self.layout.addWidget(self.test_button, 1, 1)
    
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