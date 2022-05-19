## Initialisation du projet
Le Visualisor est développé pour Linux 16.04. Une autre version de Linux ne fonctionnera pas à cause des dépences des packages ROS utilisés.

- Vérifier que Gazebo est bien installé
- Vérifier que ROS Kinetic est bien installé (http://wiki.ros.org/kinetic/Installation/Ubuntu)
- Vérifier que catkin pour ROS Kinetic est bien installé (http://wiki.ros.org/catkin)

Lors d'une première utilisation, il est nécessaire de taper les deux commandes ci-dessous.

### Initialisation de catkin
Pour initialiser catkin taper la commande : `bash setup_catkin.sh` depuis le dossier `Visualisor/`.
Ceci permet d'installer toutes les dépendances ROS du projet en local et initialise le catkin workspace.
À la suite de cette procédure, un dossier `catkin_ws/` à dû apparaitre dans `Visualisor/`.

### Modification des fichiers de catkin
Pour adapter les fichiers dans `catkin_ws/` à notre projet taper la commande : `bash modif_catkin.sh` depuis le dossier `Visualisor/`.
En pratique, cette commande permet de remplacer certain fichiers du catkin workspace local par les fichiers modifiers du dossier `.stuff` nécessaires au bon fonctionnement du Visualisor.

## Lancer le visualisor (commandless)
<del>Pour lancer l'interface, taper la commande : `python3 RoadMap_Visualisor/MainWindow.py` depuis le dossier `Visualisor/`. Les premières fois Gazebo peut paraitre capricieux donc ne pas hésiter à relancer le visualisor ou à lancer une fois Gazebo seul puis fermer la fenêtre avant de réessayer.</del>

* `./visualisor.app` pour lancer l'application: `--debug` ou `-d` pour avoir les affichages des différentes commandes **ROS** dans des terminaux (deviens vite chaotique).
* Une fois la carte choisi, nous voyons notre robot sur la carte (représenté par un anneau jaune).
* Ensuite cliquer sur le bouton ***Simuler*** de l'application pour lancer la simulation. --> WIP: automatiser le placement / lancement [cf issue de Luka](https://github.com/ruizhengxu/P-ANDROIDE/issues/1)
* Pour quitter l'application proprement, il suffit de fermer les fenêtre de l'application (tous les programmes invoqués comme les processus **ROS**, **Gazebo** etc. seront tués).
* Pour changer de carte / simulation, il faut relancer l'application. WIP changer dynamiquement sans relancer l'appli [cf issue d'Etienne](https://github.com/ruizhengxu/P-ANDROIDE/issues/6) & [cf issue de Ruizheng](https://github.com/ruizhengxu/P-ANDROIDE/issues/13)