from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

INTERSECTION_WIDTH = 3
SECTION_WIDTH = 30

class Canvas(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        
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
            
        painter.setPen(QPen(Qt.blue, INTERSECTION_WIDTH))
        painter.setBrush(QBrush(Qt.blue))
        # Draw intersections
        for intersection in self.intersections.values():
            pt = QPoint(intersection["x"], intersection["y"])
            painter.drawEllipse(pt, INTERSECTION_WIDTH, INTERSECTION_WIDTH)
            
       
    def init_obj(self):
        self.intersections = {}
        self.sections = {}
        self.drugstores = {}
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
        
            
    def draw(self, map_data):
        # Processing map data
        self.intersections = self.extract_data(map_data["intersections"])
        self.sections = self.extract_data(map_data["sections"])
        self.robot = map_data["robot"]
        self.drugstores = self.extract_data(map_data["drugStores"])
        self.update() # Update canvas