import sys, os
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from Map import *
from Start import *

WIDTH = 1280
HEIGHT = 720
LOG_FNAME = "log.txt"

class MainWindow(QMainWindow):

    def __init__(self, parent = None ):
        QMainWindow.__init__(self, parent)
        self.setWindowTitle("RoadMap Visualisor")
        screen = QApplication.primaryScreen()
        self.setGeometry(int(screen.size().width()/2)-int(WIDTH/2),
                         int(screen.size().height()/2)-int(HEIGHT/2), 
                         WIDTH, HEIGHT)
        
        # Initialize class variables
        self.folder_path = ""
        self.file_name = ""
    
        # Initialize different windows
        self.start_window = Start()
        self.start_window.import_new.clicked.connect(self.show_map)
        self.start_window.import_builded.clicked.connect(self.show_map)
        
        self.map = Map()
        self.map.mainMenu_button.clicked.connect(self.main_window)
        
        self.stack = QStackedWidget()
        self.stack.addWidget(self.start_window)
        self.stack.addWidget(self.map)
        
        self.setCentralWidget(self.stack)
      
    def main_window(self):
        self.map = None
        self.stack.setCurrentWidget(self.start_window)
        
    def init_map(self):
        self.map = Map()
        self.map.mainMenu_button.clicked.connect(self.main_window)
    
    def show_map(self):
        try:
            if self.map is None:
                self.init_map()
            self.folder_path, self.file_name = self.get_file()
            self.map.render_map(self.folder_path, self.file_name)
            self.stack.addWidget(self.map)
            self.stack.setCurrentWidget(self.map)
        except Exception as e:
            print("File error :", e.__doc__)
            
    def get_file(self):
        path = QFileDialog.getOpenFileName(self, "Select JSON File", "", "JSON files (*.json)")[0]
        path = os.path.split(path)
        return path[0]+"/", path[1]

if __name__=="__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())