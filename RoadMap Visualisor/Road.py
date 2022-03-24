import Utils
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from RoadCanvas import *

SIZE = 1535
WIDTH = 1280
HEIGHT = 720
OFFSET = 30

class Road(QWidget):
    def __init__(self, parent=None):
        super(QWidget, self).__init__(parent)
        self.setWindowTitle("Road Visualisor")
        screen = QApplication.primaryScreen()
        self.setGeometry(int(screen.size().width()/2)-int(WIDTH/2)+OFFSET,
                         int(screen.size().height()/2)-int(HEIGHT/2)+OFFSET, 
                         WIDTH, HEIGHT)
        
        self.road = None
        self.road_name = QLabel("")
        
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self.top_layout = QVBoxLayout()
        self.top_layout.setContentsMargins(10, 5, 10, 5)
        
        self.init_road_canvas()
        self.init_road_properties()
        self.statusBar = QStatusBar()
        
        self.top_layout.addWidget(self.road_name, stretch=1, alignment=Qt.AlignCenter)
        self.top_layout.addWidget(self.road_canvas, stretch=85)
        self.top_layout.addWidget(self.road_properties, stretch=14)
        self._layout.addLayout(self.top_layout)
        self._layout.addWidget(self.statusBar)

        self.setLayout(self._layout)
        
    def init_road_canvas(self):
        self.road_canvas = RoadCanvas(self)
        
    def init_road_properties(self):
        self.road_properties = QTabWidget()
        
        self.road_properties.setSizePolicy(QSizePolicy.Preferred,
                QSizePolicy.Ignored)
        
        tab1 = QWidget()
        tableWidget = QTableWidget(10, 10)

        tab1hbox = QHBoxLayout()
        tab1hbox.setContentsMargins(5, 5, 5, 5)
        tab1hbox.addWidget(tableWidget)
        tab1.setLayout(tab1hbox)

        tab2 = QWidget()
        textEdit = QTextEdit()

        textEdit.setPlainText("Twinkle, twinkle, little star,\n"
                              "How I wonder what you are.\n" 
                              "Up above the world so high,\n"
                              "Like a diamond in the sky.\n"
                              "Twinkle, twinkle, little star,\n" 
                              "How I wonder what you are!\n")

        tab2hbox = QHBoxLayout()
        tab2hbox.setContentsMargins(5, 5, 5, 5)
        tab2hbox.addWidget(textEdit)
        tab2.setLayout(tab2hbox)

        self.road_properties.addTab(tab1, "&Table")
        self.road_properties.addTab(tab2, "Text &Edit")
        
    def render_road(self, folder_path, file_name, section_name):
        section_data = self.load_road(folder_path+file_name)
        self.road_name.setText(section_name)
        
        self.road_canvas.render(section_data)
    
    def load_road(self, file_name):
        return Utils.read_json(file_name)
    