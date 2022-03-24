from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from xarray import Coordinate

INTERSECTION_WIDTH = 5
INTEREST_PT_WIDTH = 3
SECTION_WIDTH = 30

class Canvas(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background-color: black")
                
        self.init_obj()

    def paintEvent(self, event):
        painter = QPainter(self)
        
        # Draw sections
        for section in self.sections.values():
            i1 = self.intersections[section["intersection1"]]
            i2 = self.intersections[section["intersection2"]]
            control = section["control"]
        
            start_pt = QPoint(i1["x"], i1["y"])
            end_pt = QPoint(i2["x"], i2["y"])
            control_pt = QPoint(control["x"], control["y"])
            
            # Draw section's control point
            painter.setPen(QPen(Qt.red, INTERSECTION_WIDTH))
            painter.setBrush(QBrush(Qt.red))
            painter.drawEllipse(control_pt, INTERSECTION_WIDTH, INTERSECTION_WIDTH)
            
            path = QPainterPath()
            path.moveTo(start_pt)
            path.quadTo(control_pt, end_pt)
            
            # Draw section's curve
            painter.setPen(QPen(Qt.gray, SECTION_WIDTH))
            painter.setBrush(QBrush(Qt.NoBrush))
            painter.drawPath(path)
            
        # Draw intersections
        painter.setPen(QPen(Qt.white, INTERSECTION_WIDTH))
        painter.setBrush(QBrush(Qt.white))  
        for intersection in self.intersections.values():
            pt = QPoint(intersection["x"], intersection["y"])
            painter.drawEllipse(pt, INTERSECTION_WIDTH, INTERSECTION_WIDTH)
          
        # Draw drugstore
        self.draw_interest_pt(painter, self.drugstores, Qt.green)
        
        # Draw depots
        self.draw_interest_pt(painter, self.depots, Qt.magenta)
        
        # Draw garage
        self.draw_interest_pt(painter, self.garages, Qt.cyan)
        
        # Draw restaurants
        self.draw_interest_pt(painter, self.restaurants, Qt.yellow)
        
        # # Draw zones
        # # self.draw_interest_pt(painter, self.zones, Qt.lightgray)
        
        # Draw robot
        painter.setPen(QPen(Qt.black, INTEREST_PT_WIDTH))
        painter.setBrush(QBrush(Qt.black))
        coordinate = self.intersections[self.robot["intersection"]]
        painter.drawEllipse(QPoint(coordinate["x"], coordinate["y"]), INTEREST_PT_WIDTH, INTEREST_PT_WIDTH)
       
    def init_obj(self):
        self.intersections = {}
        self.sections = {}
        self.drugstores = {}
        self.depots = {}
        self.garages = {}
        self.restaurants = {}
        self.zones = {}
        self.robot = None
            
    '''
    Extract all objects from list and stock it in a new dict
    with its ID as key.
    Params:
        - data: list
    Return:
        - extracted_data: dict
    '''
    def extract_data(self, data):
        extracted_data = {}
        for obj in data:
            extracted_data[obj.pop("id")] = obj
        return extracted_data
        
    def draw_interest_pt(self, painter, list_interest_pt, color):
        for interest_pt in list_interest_pt:
            coordinates = self.intersections[interest_pt["intersection"]]
            pt = QPoint(coordinates["x"], coordinates["y"])
            painter.setPen(QPen(color, INTEREST_PT_WIDTH))
            painter.setBrush(QBrush(color))
            painter.drawEllipse(pt, INTEREST_PT_WIDTH, INTEREST_PT_WIDTH)
            
        
    def draw(self, map_data):
        # Processing map data
        self.intersections = self.extract_data(map_data["intersections"])
        self.sections = self.extract_data(map_data["sections"])
        self.drugstores = map_data["drugStores"]
        self.depots = map_data["depots"]
        self.garages = map_data["garages"]
        self.restaurants = map_data["restaurants"]
        # self.zones = map_data["zones"]
        self.robot = map_data["robot"]
        self.update() # Update canvas