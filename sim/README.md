# Lancer la simulation

Premièrement, il faut build et run l'image docker en étant placer dans le root du dossier

Pour que le script de seulement un des deux robots soit lancé, il est possible d'ajouter la variable
d'environement `-e "USE_ONLY=ROBOT"` ou le ROBOT est le robot voulant être lancé seul écrit en majuscule (LIMO ou COGNIFLY)

```
docker compose run --build -p 5901:5901 sim
```

Ensuite exécuter ces lignes de commande dans le bash shell ouvert

```
cd /root
source .profile
TVNC_WM=openbox-session vncserver -securitytypes tlsnone,x509none,none
```

Ensuite ouvrir **TurboVNC Viewer**
Puis se connecter à `localhost:5901`
Dans TurboVNC, click droit pour ouvrir un nouveau terminal puis executer:

```
/inf3995_ws/src/launch_simulation.sh
```

Remarque: Si vous avez une erreur où il ne peux pas trouver le .sh, tapez cette commande `sed -i -e 's/\r$//' /inf3995_ws/src/launch_simulation.sh`
Cela lancera la simulation et tous les launch files nécessaires.

## Lancement manuel de tout les fichiers de simulation

```
roslaunch limo_gazebo_sim gmapping.launch
```

Puis pour lancer l'exploration autonome manuellement plutot que d'utiliser la groundstation:

```
roslaunch limo_gazebo_sim explore.launch
```

Pour lancer les fichier main de chaque robot ouvrir un nouveau terminal (dans TurboVNC) et executer

```
python3 /inf3995_ws/src/main_robot1.py
```

Puis dans un autre terminal

```
python3 /inf3995_ws/src/main_robot2.py
```

Si la station au sol roule également, il est possible d'utiliser les bouton disponible pour interagir avec la simulation

Pour ouvrir un éditeur de texte et modifier des fichiers, gedit est installé et peux être utilisé `gedit ../inf3995_ws/src/main_robot1.py`ou `gedit ../inf3995_ws/src/main_robot2.py` pour le second robot.

## Pour lancer les tests

Lorsque la simulation a été lancé dans TurboVNC, vous pouvez lancez les tests pour le rover et le drone en mettant les commandes suivantes dans un terminal:

```
python3 /inf3995_ws/src/main_robot1.test.py
```

```
python3 /inf3995_ws/src/main_robot2.test.py
```
