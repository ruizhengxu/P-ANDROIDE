## Initialisation du projet
- Vérifier que Gazebo est bien installé
- Vérifier que ROS Kinetic est bien installé (http://wiki.ros.org/kinetic/Installation/Ubuntu)
- Vérifier que catkin pour ROS Kinetic est bien installé (http://wiki.ros.org/catkin)

Lors d'une première utilisation, il est nécessaire de taper les deux commandes ci-dessous.

### Initialisation de catkin
Pour initialiser catkin taper la commande : `bash setup_catkin.sh` depuis le dossier `Visualisor/`.

### Modification des fichiers de catkin
Pour adapter les fichiers dans `catkin_ws/` à notre projet taper la commande : `bash setup_project.sh` depuis le dossier `Visualisor/`.




## Lancer le visualisor
Pour lancer l'interface, taper la commande : `python3 RoadMap_Visualisor/MainWindow.py` depuis le dossier `Visualisor/`. Les premières fois Gazebo peut paraitre capricieux donc ne pas hésiter à relancer le visualisor.

Lorsque Gazebo est ouvert, il est important de placer le turtlbot sur une ligne droite de manière à avoir la ligne jaune à sa gauche et la ligne blanche à sa droite. Après quelques instants il se met en route.