from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class RoadCanvas(QWidget):
    
    def __init__(self, parent):
        QWidget.__init__(self, parent)
        
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background-color: black")
        
        self.init_obj()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(QPen(Qt.white))
        painter.drawRect(QRect(20, 40, 40, 20))
        self.update()
        
    def mousePressEvent(self, event):
        self.update()
    
    def mouseMoveEvent(self, event):
        self.update()
    
    def mouseReleaseEvent(self, event):
        self.update()
        
    def init_obj(self):
        self.intersections = {}
        self.sections = {}
        self.drugstores = {}
        self.depots = {}
        self.garages = {}
        self.restaurants = {}
        self.zones = {}
        self.robot = None