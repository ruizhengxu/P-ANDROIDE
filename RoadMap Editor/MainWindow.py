import sys
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
        
        screen = QApplication.primaryScreen()
        self.setGeometry(int(screen.size().width()/2)-int(WIDTH/2),
                         int(screen.size().height()/2)-int(HEIGHT/2), 
                         WIDTH, HEIGHT)
        
        self.start = Start()
        self.start.import_button.clicked.connect(self.show_map)
        
        self.map = Map()
        self.map.test_button.clicked.connect(self.main_window)
        
        self.stack = QStackedWidget()
        self.stack.addWidget(self.start)
        self.stack.addWidget(self.map)
        
        self.setCentralWidget(self.stack)
      
    def main_window(self):
        self.stack.setCurrentWidget(self.start)
      
    def show_map(self):
        self.map.render_map(self.start.file_name)
        self.stack.setCurrentWidget(self.map)

if __name__=="__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())