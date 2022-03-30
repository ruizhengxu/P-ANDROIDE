from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from MainWindow import *

INTERSECTION_WIDTH = 20
INTEREST_PT_WIDTH = 3
SECTION_WIDTH = 5
L_SPACING = 120
C_SPACING = 300

ELEMENT_COLOR = {
    "ROBOT": Qt.blue,
    "DRUG STORES": Qt.green,
    "GARAGES": Qt.white,
    "RESTAURANTS": Qt.red,
    "DEPOTS": Qt.magenta,
    "ZONES": Qt.yellow,
}

class MapCanvas(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
                
        self.pen = QPen(Qt.gray)
        self.brush = QBrush(Qt.gray)
        
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("background-color: black")
        self.setMouseTracking(True)

        self.init_obj()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(self.pen)
        painter.setBrush(self.brush)
        
        # Draw sections
        for key, section in self.sections.items():
            if key == self.selected_section: painter.setBrush(QBrush(Qt.red))
            else: painter.setBrush(QBrush(Qt.lightGray))
            
            i1 = self.intersections[section["intersection1"]]
            i2 = self.intersections[section["intersection2"]]
            posX = QPoint(int(i1["x"]-SECTION_WIDTH/2), int(i1["y"]-SECTION_WIDTH/2))
            posY = QPoint(int(i2["x"]+SECTION_WIDTH/2), int(i2["y"]+SECTION_WIDTH/2))
            painter.drawRect(QRect(posX, posY))
        
        # Draw intersections
        offset = INTERSECTION_WIDTH/2
        painter.setPen(self.pen)
        painter.setBrush(self.brush)
        for key, i in self.intersections.items():
            painter.drawEllipse(QPoint(i["x"], i["y"]), INTERSECTION_WIDTH, INTERSECTION_WIDTH)
            painter.setPen(QPen(Qt.black))
            painter.drawText(QPoint(int(i["x"]-offset), int(i["y"]+offset)), str(i["l"])+","+str(i["c"]))
        
        # Draw elements
        for key, element in self.all_elements.items():
            ele = element[0]
            show = element[1]
            if len(ele) > 0 and show:
                painter.setPen(QPen(ELEMENT_COLOR[key]))
                painter.setBrush(QBrush(ELEMENT_COLOR[key]))
                if type(ele) == dict: # If element is robot
                    i = self.intersections[ele["intersection"]] # Intersection of the element
                    painter.drawEllipse(QPoint(i["x"], i["y"]), INTERSECTION_WIDTH/3, INTERSECTION_WIDTH/3)
                else: # Others elements
                    for e in ele:
                        i = self.intersections[e["intersection"]] # Intersection of the element
                        painter.drawEllipse(QPoint(i["x"], i["y"]), INTERSECTION_WIDTH/3, INTERSECTION_WIDTH/3)
                        
        
    
    def mousePressEvent(self, event):
        self.selected_section = self.get_pointed_element(event.pos(), only_section=True)
        if self.selected_section != "":
            self.parent().show_road(self.selected_section)
        self.update()
        
    def mouseMoveEvent(self, event):
        self.parent().setMessage(self.get_pointed_element(event.pos()))
            
    def init_obj(self):
        self.sections = {}
        self.intersections = {}
        self.all_elements = {}
        self.selected_section = ""
    
    def render(self, map_data):
        self.get_sections(map_data)
        self.get_intersections(map_data)
        self.get_map_elements(map_data)
        self.update()
        
    def get_sections(self, map_data):
        for section in map_data["sections"]:
            key = section.pop("$$hashKey")
            self.sections[key] = section
    
    def get_intersections(self, map_data):
        for i, intersection in enumerate(map_data["intersections"]):
            intersection["x"] = (intersection["c"]+1)*C_SPACING
            intersection["y"] = (intersection["l"]+1)*L_SPACING
            self.intersections["~intersections~" + str(i)] = intersection
            
    def get_map_elements(self, map_data):
        robot = map_data["robot"]
        drugStores = map_data["drugStores"]
        garages = map_data["garages"]
        restaurants = map_data["restaurants"]
        depots = map_data["depots"]
        zones = map_data["zones"]
        self.all_elements = {
            "ROBOT": [robot, True],
            "DRUG STORES": [drugStores, False],
            "GARAGES": [garages, False],
            "RESTAURANTS": [restaurants, False],
            "DEPOTS": [depots, False],
            "ZONES": [zones, False]
        }
            
    def get_pointed_element(self, point, only_section=False):
        pointed_element = ""
        
        if not only_section:
            for key, intersection in self.intersections.items():
                ele = QRect(intersection["x"]-INTERSECTION_WIDTH, intersection["y"]-INTERSECTION_WIDTH,
                            INTERSECTION_WIDTH*2, INTERSECTION_WIDTH*2)
                if ele.contains(point): pointed_element = key
                
        if pointed_element == "":
            for key, section in self.sections.items():
                i1 = self.intersections[section["intersection1"]]
                i2 = self.intersections[section["intersection2"]]
                posX = QPoint(int(i1["x"]-SECTION_WIDTH/2+INTERSECTION_WIDTH), 
                              int(i1["y"]-SECTION_WIDTH/2+INTERSECTION_WIDTH))
                posY = QPoint(int(i2["x"]+SECTION_WIDTH/2-INTERSECTION_WIDTH), 
                              int(i2["y"]+SECTION_WIDTH/2-INTERSECTION_WIDTH))
                ele = QRect(posX, posY)
                if ele.contains(point): pointed_element = key
        
        return pointed_element
    
    def show_element(self, element, show):
        tmp = self.all_elements[element]
        tmp[1] = show
        self.all_elements[element] = tmp
            
        self.update()