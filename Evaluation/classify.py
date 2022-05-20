# Ce script permet d'attribuer une classe à chaque route d'un dossier passé en argument.
#
# Il prend donc un dossier en argument contenant les routes à classer au format .json (du Road Editor).
# La classe associée à la route est directement ajoutée à son fichier.

import numpy as np
import sys
import json
import os
from numpy.linalg import norm, det
from PIL import Image, ImageDraw

# Get point of bezier curve at t
def P(p0, p1, p2, t):
    return (1-t)**2 * p0 + 2 * (1-t) * t * p1 + t**2 * p2

def P_first(p0, p1, p2, t):
    return 2 * (1 - t) * (p1 - p0) + 2*t*(p2 - p1)

def compute_k(B1, B2):
    # return (np.linalg.det([B1, B2])) / (B1[0]**2 + B1[1]**2)**(3/2)
    return ( np.abs(np.linalg.det([B1, B2])) ) / (B1[0]**2 + B1[1]**2)**(3/2)

# Get distance of two points
def distance(a, b):
    return np.hypot(b[0]-a[0], b[1]-a[1])
    # return np.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)

def find_nearest_point_to_control_point(p0, p1, p2, eps=10e-16):
    t = 0
    step = 0.001
    while np.abs(t-(t+step)) > eps:
        # Compute current and next point bezier curve
        current_point = P(p0, p1, p2, t)
        next_point = P(p0, p1, p2, t+step)
        # Compute distance from both point to control point
        current_dist = distance(current_point, p1)
        next_dist = distance(next_point, p1)
        # Compare both distances
        if next_dist < current_dist:
            t += step # Update t
        else:
            step /= 10 # Decrease steps if next distance is higher
    return t

# Get curvature of the curve at t
def compute_curvarture(p0, p1, p2, t):
    Pp = lambda t : p0*(2+2*t) + p1*(2-4*t) + p2*(2*t) # Derivative of bezier curve function
    Ppp = lambda t : p0*(2) + p1*(4) + p2*(2) # Second derivative of bezier curve function
    
    Pp2 = lambda t : p0*(-2+2*t) + p1*(2-4*t) + p2*(2*t)
    Ppp2 = lambda t : p0*(2) + p1*(-4) + p2*(2)

    b1, b2 = Pp(t), Ppp(t)
    b1, b2 = Pp2(t), Ppp2(t)
    k = compute_k(b1, b2)
    return np.power(k, -1)

def getClass(A,B,C):
    '''
    [int,int],[int,int],[int,int] -> int

    Function to get the class of a bezier quadratic curve from its control points
    '''

    # # ANGLE AND DISTANCE TO CRITIC POINT
    # BA = A - B
    # BC = C - B
    # angle = int(round(np.degrees(np.arccos(np.dot(BA, BC) / (np.linalg.norm(BA) * np.linalg.norm(BC))))))

    # f = lambda t : A*(1-t)**2+2*B*t*(1-t)+C*t**2
    # t_ = 0.5
    # for t in np.linspace(0,1,5000):
    #     if norm(f(t) - B) < norm(f(t_) - B):
    #         t_ = t
    # distance = int(round(norm(A - f(t_))))
    # value = distance,angle
    # print(distance,angle)
    
    # score = round(distance*angle/100)
    # scores = [100, 200, 300, 800, 1100, 1400, 1700, 2000]

    # for i in range(len(scores)):
    #     if(score <= scores[i]):
    #         return i
    # return len(scores)

    # CURVATURE
    # curvature = 1/abs(norm(C - 2*B + A)**2 / (2 * det(np.array([B - A, C - B]))))
    t_star = find_nearest_point_to_control_point(A, B, C)
    curvature = compute_curvarture(A, B, C, t_star)
    print(curvature)
    # scores = [0.12, 0.16, 0.24, 0.34, 0.44, 0.47, 0.56, 0.97, 1.33]
    # scores = [0.16, 0.34, 0.47, 0.97, 1.33]
    scores = [100, 200, 400, 800, 2400]

    for i in range(len(scores)):
        if(curvature <= scores[i]):
            return i
    return len(scores)

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
        # save(curve,name)

def save(curve, name):
    '''
    str (source file), str (name to save) -> void

    Function to save a reconstructed json curve
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

    img.save('visuals/'+name[:-5]+'.png')


classifyFolder(sys.argv[1])

source = sys.argv[1]
classes = [[] for i in range(10)]
for name in os.listdir(source):
    curve = source + "/" + name
    file = open(curve)
    data = json.load(file)
    file.close()
    classes[data['class']] += [name]


print("")
for cla in range(len(classes)):
    print("CLASS :",cla)
    for curve in classes[cla]:
        print(curve)
    print("")
