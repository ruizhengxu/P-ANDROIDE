import json
from math import atan2
import os
import sys
import xml.etree.ElementTree as ET

dir_path = os.path.dirname(os.path.realpath(__file__))
source = sys.argv[1]
target = dir_path+"/../catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_autorace.launch"

scale_json = 1535
scale_env = 6

# Calculate pose(x,y,z,R,P,Y) for robot initialization
file = open(source)
data = json.load(file)
file.close()

def transform(x_,y_):
    x = ((int(x_)*scale_env/scale_json)-scale_env/2)
    y = -((int(y_)*scale_env/scale_json)-scale_env/2)
    z = 0
    return x,y,z

pos = data['drags'][2]['pos']
x,y,z = transform(pos['x'],pos['y']) #this will be the position of the robot in gazebo

pt_w = data['list_man'][0]['white'][1]
pt_y = data['list_man'][0]['yellow'][1]
x_,y_,z_ = transform((pt_w['x']+pt_y['x'])/2,(pt_w['y']+pt_y['y'])/2)

R,P,Y = 0,0,atan2(y_-y, x_-x) #this will be the orientation of the robot in gazebo

# Write pose in init launch file
mytree = ET.parse(target)
myroot = mytree.getroot()

for arg in myroot.iter("arg"):
    name = arg.get("name")
    if(name == "x_pos"):
        arg.set("default",str(x))
    elif(name == "y_pos"):
        arg.set("default",str(y))
    elif(name == "z_pos"):
        arg.set("default",str(z))
    elif(name == "roll"):
        arg.set("default",str(R))
    elif(name == "pitch"):
        arg.set("default",str(P))
    elif(name == "yaw"):
        arg.set("default",str(Y))

mytree.write(target)


