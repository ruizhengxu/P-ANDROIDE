import json
from PIL import Image, ImageDraw
from math import ceil
import numpy as np
import sys
import os

resize = 4
line_width = resize*8
w=3071*2
h=3071*2
r = line_width/2

dir_path = os.path.dirname(os.path.realpath(__file__))
source = sys.argv[1]
target = dir_path+"/../catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_autorace/course/materials/textures/course.png"

file = open(source)
data = json.load(file)
file.close()

white = []
yellow = []
for u in data['list_man']:
    for v in u['white']:
        white += [(resize*v['x'],resize*v['y'])]
    for v in u['yellow']:
        yellow += [(resize*v['x'],resize*v['y'])]
len_white = len(white)
len_yellow = len(yellow)    

img = Image.new("RGB", (w, h))
img1 = ImageDraw.Draw(img)

for i in range(len_white-1):
    p1,p2 = white[i],white[i+1]
    img1.line([p1,p2], fill ="white", width = line_width)
    x,y = p2
    img1.ellipse([(x-r,y-r),(x+r,y+r)], fill = "white")

for i in range(len_yellow-1):
    p1,p2 = yellow[i],yellow[i+1]
    img1.line([p1,p2], fill ="yellow", width = line_width)
    x,y = p2
    img1.ellipse([(x-r,y-r),(x+r,y+r)], fill = "yellow")

#img.save(source[:-5]+".png")
img.save(target)
