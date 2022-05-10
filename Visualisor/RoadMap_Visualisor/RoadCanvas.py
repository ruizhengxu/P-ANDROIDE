from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from MainWindow import WIDTH, HEIGHT

ROAD_WIDTH = 3
ROAD_SIZE = 4

import json
import threading
import time
import numpy as np
import os

class RoadCanvas(QWidget):
    
    def __init__(self, parent):
        QWidget.__init__(self, parent)

        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background-color: black")
        self.setMouseTracking(True)

        self.init_obj()

        self.data = None
        self.trajectories = []
        self.ev1 = threading.Event()
        self.ev1.set()

        self.T = threading.Thread(target=self.get_data_from_file)
        self.T.start()
    
    def get_data_from_file(self):

        while True:
            if not self.ev1.wait(0): break 
            try:
                fp = open('data.json')
                self.data = json.load(fp)
                fp.close()
                time.sleep(1/60)
                self.update()
            except:
                pass
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.translate(self.offset+self.current_offset)
        painter.scale(self.scale, self.scale)
        path = QPainterPath()
        
        if len(self.list_curves) != 0:
            path.moveTo(QPointF(self.white_pts[0]["x"], self.white_pts[0]["y"]))
            for i in range(0, len(self.white_pts), 2):
                # Draw white points
                painter.setPen(QPen(Qt.white, ROAD_WIDTH))
                pt1 = QPointF(self.white_pts[i]["x"], self.white_pts[i]["y"])
                pt2 = QPointF(self.white_pts[i+1]["x"], self.white_pts[i+1]["y"])
                path.quadTo(pt1, pt2)
            painter.drawPath(path)
            path.clear()
            
            path.moveTo(QPointF(self.yellow_pts[0]["x"], self.yellow_pts[0]["y"]))
            for i in range(0, len(self.yellow_pts), 2):
                # Draw yellow points
                painter.setPen(QPen(Qt.yellow, ROAD_WIDTH))
                pt1 = QPointF(self.yellow_pts[i]["x"], self.yellow_pts[i]["y"])
                pt2 = QPointF(self.yellow_pts[i+1]["x"], self.yellow_pts[i+1]["y"])
                path.quadTo(pt1, pt2)
            painter.drawPath(path)
            path.clear()
        
        #print(self.data)
        #painter.setPen(Qt.yellow)
        #painter.setBrush(Qt.yellow)
        if len(self.trajectories) > 0:
            painter.setPen(QPen(QColor("EA6E19")))
            for trajectory in self.trajectories:
                painter.drawEllipse(QPoint(trajectory[0], trajectory[1], 1, 1))
        
        if self.data != None:
            # print(QPoint(((ROAD_SIZE/2) + self.data["pose"]["position"]["x"])*(WIDTH/ROAD_SIZE), ( (ROAD_SIZE/2) + self.data["pose"]["position"]["y"])*(HEIGHT/ROAD_SIZE)))
            painter.setPen(QPen(Qt.red))
            x = ((ROAD_SIZE/2) + self.data["pose"]["position"]["x"])*(WIDTH/ROAD_SIZE)
            y = ((ROAD_SIZE/2) + self.data["pose"]["position"]["y"])*(HEIGHT/ROAD_SIZE)
            painter.drawEllipse(QPoint(x, y, 5, 5))
            self.trajectories.append([x, y])

        #self.update()
        
    def mousePressEvent(self, event):
        self.move_canvas = event.button() == Qt.RightButton
        self.begin = event.pos()
        self.update()
    
    def mouseMoveEvent(self, event):
        self.parent().setMessage("x : " + str(event.pos().x()) + ",  y : " + str(event.pos().y()))
        if self.move_canvas:
            self.current_offset = event.pos() - self.begin
        self.update()
    
    def mouseReleaseEvent(self, event):
        self.offset = self.offset+self.current_offset
        self.current_offset = QPoint(0, 0)
        self.move_canvas = False
        self.update()
        
    def wheelEvent(self, event):
        angle = event.angleDelta().y()
        if angle < 0:
            self.scale -= 0.02
        elif angle > 0:
            self.scale += 0.01
        self.update()
        
    def init_obj(self):
        self.move_canvas = False
        self.begin = None
        self.offset = QPoint(0, 0)
        self.current_offset = QPoint(0, 0)
        self.scale = 1
        self.list_curves = []
        self.white_pts = []
        self.yellow_pts = []
        
    def render(self, section_data):
        self.list_curves = section_data["list_man"]
        self.white_pts.clear()
        self.yellow_pts.clear()
        for curve in self.list_curves:
            self.white_pts += curve["white"]
            self.yellow_pts += curve["yellow"]
        
