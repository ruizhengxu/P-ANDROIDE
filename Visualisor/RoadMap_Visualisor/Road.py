import Utils, os, sys
from pathlib import Path
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from RoadCanvas import *

SIZE = 1535
WIDTH = 1280
HEIGHT = 720
OFFSET = 30

class Road(QWidget):
    def __init__(self, road_name, parent=None):
        super(QWidget, self).__init__(parent)
        self.setAttribute(Qt.WA_QuitOnClose, False)

        self.setWindowTitle("Road Visualisor")
        screen = QApplication.primaryScreen()
        self.setGeometry(int(screen.size().width()/2)-int(WIDTH/2)+OFFSET,
                         int(screen.size().height()/2)-int(HEIGHT/2)+OFFSET, 
                         WIDTH, HEIGHT)
        
        self.path = os.path.abspath(__file__)
        self.path = str(Path(self.path).parent)
        self.road = None
        self.road_name = road_name
        self.road_label = QLabel("")
        
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self.bottom_layout = QHBoxLayout()
        self.bottom_layout.setContentsMargins(0, 0, 0, 0)
        self.btn_layout = QVBoxLayout()
        self.top_layout = QVBoxLayout()
        self.top_layout.setContentsMargins(10, 5, 10, 5)
        
        self.init_road_canvas()
        self.init_road_properties()
        
        self.stop_btn = QPushButton("Stop Simulation")
        self.stop_btn.clicked.connect(self.stop_simulation)
        self.stop_btn.setDisabled(True)
        self.simulate_btn = QPushButton("Simulate")
        self.simulate_btn.clicked.connect(self.simulate)
        self.btn_layout.addWidget(self.stop_btn)
        self.btn_layout.addWidget(self.simulate_btn)

        self.statusbar = QStatusBar()
        self.statusbar.showMessage("Ready")

        self.bottom_layout.addLayout(self.params_layout, stretch=80)
        self.bottom_layout.addLayout(self.btn_layout, stretch=20)
        
        self.top_layout.addWidget(self.road_label, stretch=1, alignment=Qt.AlignCenter)
        self.top_layout.addWidget(self.road_canvas, stretch=85)
        self.top_layout.addLayout(self.bottom_layout, stretch=14)
        self._layout.addLayout(self.top_layout)
        self._layout.addWidget(self.statusbar)

        self.setLayout(self._layout)
        
    def init_road_canvas(self):
        self.road_canvas = RoadCanvas(self)
        
    def init_road_properties(self):
        speed_path = self.path + "/../catkin_ws/src/turtlebot3_autorace/turtlebot3_autorace_control/nodes/control_lane"
        with open(speed_path) as f:
            for line in f:
                line = line.rstrip()
                if line.startswith("MIN_LIN"): self.minSpeed = line.split("=")[1].strip()
                if line.startswith("MAX_LIN"): self.maxSpeed = line.split("=")[1].strip()
        
        self.params_layout = QGridLayout()
        self.minSpd_input = QLineEdit()
        self.minSpd_input.setText(self.minSpeed)
        self.maxSpd_input = QLineEdit()
        self.maxSpd_input.setText(self.maxSpeed)

        self.params_layout.addWidget(QLabel("Simulation parameters"), 0, 0, 1, 2)
        self.params_layout.addWidget(QLabel("Min Speed"), 1, 0)
        self.params_layout.addWidget(self.minSpd_input, 1, 1)
        self.params_layout.addWidget(QLabel("Max Speed"), 2, 0)
        self.params_layout.addWidget(self.maxSpd_input, 2, 1)
        
    def render_road(self, folder_path, file_name):
        section_data = self.load_road(folder_path+file_name)
        self.road_label.setText(file_name)

        self.road_canvas.render(section_data)
    
    def load_road(self, file_name):
        return Utils.read_json(file_name)
    
    def stop_simulation(self):
        self.stop_btn.setDisabled(True)
        self.simulate_btn.setDisabled(False)
        
        Utils.save_data_as_json(self.road_canvas.trajectories, self.road_name)
        
        quit_path = self.path + "/../quit.sh"
        os.system("bash " + quit_path + " &")
        print("Stop simulation")
    
    def simulate(self):
        if self.checkParams():
            path = os.path.abspath(__file__)
            path = str(Path(path).parent)
            setup_map_path = path + "/../map/setup_map.py"
            setup_pose_path = path + "/../pose/setup_pose.py"
            file_path = path + "/data/" + self.road_name
            setup_path = path + "/../setup.sh"
            start_path = self.path + "/../start.sh"

            # Handle debug mod
            if len(sys.argv) > 1:
                if sys.argv[1] == "-d":
                    setup_path += " -d"

            os.system("python3 " + setup_map_path + " " + file_path)
            print("map switched to", self.road_name)

            os.system("python3 " + setup_pose_path + " " + file_path)
            print("position initialized")

            os.system(setup_path + " &")
            print("setup success")
            
            time.sleep(5) # Wait for 5 seconds before launch autorace
            
            os.system("bash " + start_path + " &")
            self.simulate_btn.setDisabled(True)
            self.stop_btn.setDisabled(False)
            print("robot launch with success")
            
    def checkParams(self):
        error = False
        error_text = ""
        
        try:
            minSpeed = float(self.minSpd_input.text())
            maxSpeed = float(self.maxSpd_input.text())
            
            if minSpeed < 0 or maxSpeed < 0:
                error = True
                error_text += "Speeds should not be lower than 0 !\n"
            if minSpeed > maxSpeed:
                error = True
                error_text += "Minimun speed should be lower than maximum speed !\n"
        except Exception as e:
            error = True
            error_text = "Speed should be a decimal or integer number !\n"   
        
        if error:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle("Error in parameters")
            msg.setText("Please verify your parameters")
            msg.setInformativeText(error_text)
            retval = msg.exec_()
            return False
        
        # Edit file when no error
        speed_path = self.path + "/../catkin_ws/src/turtlebot3_autorace/turtlebot3_autorace_control/nodes/control_lane"
        # speed_path = "control_lane"
        with open(speed_path, "r") as f:
            f_data = f.read()
        f_data = f_data.replace("MIN_LIN = " + self.minSpeed, "MIN_LIN = " + str(minSpeed))
        f_data = f_data.replace("MAX_LIN = " + self.maxSpeed, "MAX_LIN = " + str(maxSpeed))
        with open(speed_path, "w") as f:
            f.write(f_data)
            
        self.minSpeed = str(minSpeed)
        self.maxSpeed = str(maxSpeed)
        
        return True

    def setMessage(self, msg):
        self.statusbar.showMessage(msg)

    def closeEvent(self, event):
        self.road_canvas.ev1.clear()
