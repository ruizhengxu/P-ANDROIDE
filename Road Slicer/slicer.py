import json
from PIL import Image, ImageDraw
from math import ceil
import numpy as np

line_width = 20
resize = 10
A4 = (1463, 1034)

def getGeometry(A4,white,yellow):
    '''
    A4 : dimensions feuille A4 en nombre de pixels (1463x1034px ; 2633x1861px ; 3510x2481px)
    white : liste des coordonnees des points de la bordure blanche
    yellow : liste des coordonnees des points de la bordure jaune

    Renvoie le decoupage a faire sur les abscisses et ordonnees
    '''
    w,h = 0,0
    all = white+yellow
    for x,y in all:
        if x > w:
            w = x
        if y > h:
            h = y
    return ceil(w/A4[0]),ceil(h/A4[1])

def slice(source,target,A4=A4,resize=resize,width_line=line_width):
    '''
    source : fichier .json de la route a imprimer
    target : dossier de sauvegarde des images creees

    A4 : dimensions feuille A4 en nombre pixels (1463x1034px ; 2633x1861px ; 3510x2481px)
    resize : facteur echelle
    width : epaisseur des traits de bordure de la route

    Sauvegarde les images a imprimer pour representer le circuit au sol a la bonne echelle
    '''

    if(target[-1] != "/"):
        target += "/"

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

    gw, gh = getGeometry(A4,white,yellow)
    w,h = gw*A4[0],gh*A4[1]

    img = Image.new("RGB", (w, h))
    img1 = ImageDraw.Draw(img)

    for i in range(len_white-1):
        p1,p2 = white[i],white[i+1]
        img1.line([p1,p2], fill ="white", width = line_width)

    for i in range(len_yellow-1):
        p1,p2 = yellow[i],yellow[i+1]
        img1.line([p1,p2], fill ="yellow", width = line_width)

    image = np.array(img)

    for x in range(gw):
        for y in range(gh):
            print(x,y)
            subimg = image[y*A4[1]:(y+1)*A4[1],x*A4[0]:(x+1)*A4[0],:]
            if(np.max(subimg) > 0): # Sauvegarder seulement si l'image nest pas vide
                Image.fromarray(subimg).save(target+str(y)+"_"+str(x)+".png")

    # Save the entire road
    # img.save(source[:-5]+".png")
