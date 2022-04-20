# Uniquement necessaire la premiere fois
Dans le directory `simulation/catkin_ws/`
1) Supprimer `devel/`, `build/`, `src/CMakeLists.txt`
2) Taper la commande `catkin_make`

# Pour lancer la simulation
Dans le directory `simulation/`
Taper la commande `bash setup.sh`
Taper la commande `bash start.sh`

# Pour changer la carte
Taper la commande `python3 setup_map.py [fichier]`
ou `[fichier]` correspond au nom du fichier genere par le roadmap editor en .json

Certains exemples de cartes sont disponibles dans `simulation/map/maps/`

# Infos en plus
1) Il est important de placer manuellement le robot depuis gazebo dès l'ouverture de la fenetre.
Pour cela on peut mettre la simulation sur pause (en bas à gauche),
placer le robot entre la bande jaune (à gauche) et la blanche (à droite)
puis relancer la simulation en cliquant à nouveau sur le bouton pause.

2) Les vitesses minimales et maximales du robot dans la simulation sont les variables
`MAX_VEL_LIN` `MIN_VEL_LIN` dans le fichier `simulation/catkin_ws/src/turtlebot3_autorace/turtlebot3_autorace_control/nodes/control_lane.py`.
Elles sont respectivement set à 0.2 et 10 de base.

3) Les deux points precedents ont vocation a etre ameliores dans la suite du projet
