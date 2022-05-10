import numpy as np
import sys
import json
from PIL import Image, ImageDraw
import os
from numpy.linalg import norm, det

'''
Classifies a whole folder of json curves
Folder path should be given in parameter when calling classify.py
'''

def display(curve):
    '''
    str (source file) -> void

    Function to display a json curve
    '''

    #Constants
    line_width = 7
    r = line_width/2
    w,h = 1535,1535

    #Open and read JSON file
    file = open(curve)
    data = json.load(file)
    file.close()

    #Construct lines
    white = []
    yellow = []
    for u in data['list_man']:
        for v in u['white']:
            white += [(v['x'],v['y'])]
        for v in u['yellow']:
            yellow += [(v['x'],v['y'])]

    #Init image
    img = Image.new("RGB", (w, h), color="black")
    img1 = ImageDraw.Draw(img)

    #Draw white line
    for i in range(len(white)-1):
        p1,p2 = white[i],white[i+1]
        img1.line([p1,p2], fill ="white", width = line_width)
        x,y = p2
        img1.ellipse([(x-r,y-r),(x+r,y+r)], fill = "white")

    #Draw yellow line
    for i in range(len(yellow)-1):
        p1,p2 = yellow[i],yellow[i+1]
        img1.line([p1,p2], fill ="yellow", width = line_width)
        x,y = p2
        img1.ellipse([(x-r,y-r),(x+r,y+r)], fill = "yellow")

    img.show()

def getClass(A,B,C):
    '''
    [int,int],[int,int],[int,int] -> int/float

    Function to get the class of a bezier quadratic curve from its control points
    '''

    # ANGLE DEGREE
    # BA = A - B
    # BC = C - B
    #
    # angle = np.arccos(np.dot(BA, BC) / (np.linalg.norm(BA) * np.linalg.norm(BC)))
    # value = round(np.degrees(angle))

    # CURVATURE
    curvature = 1/abs(norm(C - 2*B + A)**2 / (2 * det(np.array([B - A, C - B]))))
    if(curvature < 0.1):
        value = 0
    elif(curvature < 0.2):
        value = 1
    elif(curvature < 0.3):
        value = 3
    elif(curvature < 0.6):
        value = 4
    elif(curvature < 1):
        value = 5
    elif(curvature < 2):
        value = 6
    elif(curvature < 3):
        value = 7
    elif(curvature < 6):
        value = 8
    else:
        value = 9

    return value

def classify(source):
    '''
    str (source file) -> void

    Function to classify one json file (curve)
    adds the calculated class of the curve to the json file
    '''

    #Open and read JSON file
    file = open(source,'r')
    data = json.load(file)
    file.close()

    #Get the 3 control points
    A = data['drags'][0]['pos']
    B = data['drags'][1]['pos']
    C = data['drags'][2]['pos']

    A = np.array([A['x'],A['y']])
    B = np.array([B['x'],B['y']])
    C = np.array([C['x'],C['y']])

    #Get the class of the curve
    W = getClass(A,B,C)

    #Add the class in the json data
    data.update({"class":W})

    #Rewrite json file
    file = open(source,'w')
    file.write(json.dumps(data))

    #Print
    print(source + " successfully updated " + str(W))

def classifyFolder(source):
    '''
    str (source folder) -> void

    Function to classify a whole folder of json files (curves)
    calling the classidy function
    '''

    source = os.path.normpath(source)

    for name in os.listdir(source):
        curve = source + "/" + name
        classify(curve)



classifyFolder(sys.argv[1])