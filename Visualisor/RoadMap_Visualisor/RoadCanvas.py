from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from MainWindow import WIDTH, HEIGHT

ROAD_WIDTH = 3
ROAD_SIZE = 6

INIT_SIZE = 1535

import json
import threading
import time
import numpy as np
import os

class RoadCanvas(QWidget):
    
    def __init__(self, parent, histories):
        QWidget.__init__(self, parent)

        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background-color: black")
        self.setMouseTracking(True)
        #self.setSize(1535,1535)
        #self.setFixedHeight(1535)
        
        self.init_obj()

        self.data = None
        self.histories = histories
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
            path.moveTo(QPointF(self.white_pts[0]["x"]*(self.width()/INIT_SIZE), self.white_pts[0]["y"]*(self.height()/INIT_SIZE)))
            for i in range(0, len(self.white_pts), 2):
                # Draw white points
                painter.setPen(QPen(Qt.white, ROAD_WIDTH))
                pt1 = QPointF(self.white_pts[i]["x"]*(self.width()/INIT_SIZE), self.white_pts[i]["y"]*(self.height()/INIT_SIZE))
                pt2 = QPointF(self.white_pts[i+1]["x"]*(self.width()/INIT_SIZE), self.white_pts[i+1]["y"]*(self.height()/INIT_SIZE))
                path.quadTo(pt1, pt2)
            painter.drawPath(path)
            path.clear()
            
            path.moveTo(QPointF(self.yellow_pts[0]["x"]*(self.width()/INIT_SIZE), self.yellow_pts[0]["y"]*(self.height()/INIT_SIZE)))
            for i in range(0, len(self.yellow_pts), 2):
                # Draw yellow points
                painter.setPen(QPen(Qt.yellow, ROAD_WIDTH))
                pt1 = QPointF(self.yellow_pts[i]["x"]*(self.width()/INIT_SIZE), self.yellow_pts[i]["y"]*(self.height()/INIT_SIZE))
                pt2 = QPointF(self.yellow_pts[i+1]["x"]*(self.width()/INIT_SIZE), self.yellow_pts[i+1]["y"]*(self.height()/INIT_SIZE))
                path.quadTo(pt1, pt2)
            painter.drawPath(path)
            path.clear()
        
        ############################################
        # draw last 5 histories' trajectory
        ############################################
        if len(self.histories) > 0:
            i = 0
            for history in self.histories:
                c = QColor(Qt.darkCyan)
                c.setAlpha(80)
                painter.setPen(QPen(c))
                traj = history["trajectory"]
                for pt in traj:
                    # print(pt)
                    painter.drawEllipse(QPointF(((ROAD_SIZE/2) + pt[0])*(self.width()/ROAD_SIZE), ((ROAD_SIZE/2) - pt[1])*(self.height()/ROAD_SIZE)), 1, 1)
                    

        ############################################
        # draw trajectory
        ############################################
        if len(self.trajectories) > 0:
            c = QColor(Qt.green)
            c.setAlpha(80)
            painter.setPen(QPen(c))
            for trajectory in self.trajectories:
                painter.drawEllipse(QPointF(((ROAD_SIZE/2) + trajectory[0])*(self.width()/ROAD_SIZE), ((ROAD_SIZE/2) - trajectory[1])*(self.height()/ROAD_SIZE)), 1, 1)
        
        if self.data != None:
            # print(QPoint(((ROAD_SIZE/2) + self.data["pose"]["position"]["x"])*(WIDTH/ROAD_SIZE), ( (ROAD_SIZE/2) + self.data["pose"]["position"]["y"])*(HEIGHT/ROAD_SIZE)))
            painter.setPen(QPen(Qt.red))
            x = self.data["pose"]["position"]["x"]
            y = self.data["pose"]["position"]["y"]
            xx = ((ROAD_SIZE/2) + x)*(self.width()/ROAD_SIZE)
            yy = ((ROAD_SIZE/2) - y)*(self.height()/ROAD_SIZE)
            painter.drawEllipse(QPointF(xx, yy), 5, 5)
            
            # add trajectory
            self.trajectories.append([x, y])
            if (len(self.trajectories) > 0 and self.trajectories[len(self.trajectories)-1][0] != x and self.trajectories[len(self.trajectories)-1][1] != y):
                self.trajectories.remove(len(self.trajectories)-1)
            ############################################
            # draw orientation
            ############################################
            if (len(self.trajectories)>2):
                painter.setPen(QPen(Qt.red, 2))
                s_pt = [((ROAD_SIZE/2) + self.trajectories[len(self.trajectories)-1][0])*(self.width()/ROAD_SIZE), 
                        ((ROAD_SIZE/2) - self.trajectories[len(self.trajectories)-1][1])*(self.width()/ROAD_SIZE)]
                e_pt = [(s_pt[0] - ((ROAD_SIZE/2) + self.trajectories[len(self.trajectories)-2][0])*(self.width()/ROAD_SIZE))*20 + s_pt[0], 
                        (s_pt[1] - ((ROAD_SIZE/2) - self.trajectories[len(self.trajectories)-2][1])*(self.width()/ROAD_SIZE))*20 + s_pt[1]]
                s_pt = QPointF(s_pt[0], s_pt[1])
                e_pt = QPointF(e_pt[0], e_pt[1])
                # draw line of arrow
                arrow = QPainterPath(s_pt)
                arrow.lineTo(e_pt)
                painter.drawPath(arrow)
                # draw arrow head
                dx, dy = s_pt.x()-e_pt.x(), s_pt.y()-e_pt.y()
                length = np.sqrt(dx**2 + dy**2)
                normX, normY = dx/length, dy/length
                perpX, perpY = -normY, normX
                left_pt = QPointF(e_pt.x() + 3*normX + 3*perpX, e_pt.y() + 3*normY + 3*perpY)
                right_pt = QPointF(e_pt.x() + 3*normX - 3*perpX, e_pt.y() + 3*normY - 3*perpY)
                painter.drawPolyline(QPolygonF([left_pt, e_pt, right_pt]))

        
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

    def moveEvent(self, event):
        self.setFixedWidth(self.height())
        if self.parent().width() > self.height():
            self.move(int((self.parent().width() - self.height())/2), self.pos().y())
        
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
        
