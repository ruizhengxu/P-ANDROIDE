`slice.py` a pour but de découper une route du Road Editor (format .json) pour l'imprimer et l'utiliser avec le turtlebot3 grandeur nature.

Pour lancer la découpe taper `python3 slice.py path.json` où `path.json` est le chemin vers le fichier désiré. Les images seront sauvegardées au format A4 dans le dossier `output/`.

Il est possible de changer certain paramètres (couleurs, échelle, etc...) du slicer directement en haut du fichier `slice.py`.