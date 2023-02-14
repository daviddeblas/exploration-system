# Lancer la simulation

Premièrement, il faut build et run l'image docker en étant placer dans le root du dossier

```
docker compose run --build -p 5901:5901 sim
```

Ensuite exécuter ces lignes de commande dans le bash shell ouvert,et ajouter un mot de passe pour le Client VNC
```
cd /root
source .profile
TVNC_WM=openbox-session vncserver -securitytypes tlsnone,x509none,none
```
Ensuite ouvrir **TurboVNC Viewer**
Puis se connecter à `localhost:5901` et mettre sont mot de passe
Dans TurboVNC, click droit pour ouvrir un nouveau terminal puis executer:
```
cd ../inf3995_ws
catkin_make && source devel/setup.bash
./src/launch_simulation.sh
```

Finalement pour lancer les fichier main de chaque robot ouvrir un nouveau terminal (dans TurboVNC) et executer
```
python3 ../inf3995_ws/src/main_robot1.py
```

Puis dans un autre terminal

```
python3 ../inf3995_ws/src/main_robot2.py
```

Si la station au sol roule également, il est possible d'utiliser les bouton disponible pour interagir avec la simulation

Pour ouvrir un éditeur de texte et modifier des fichiers, gedit est installé et peux être utilisé `gedit ../inf3995_ws/src/main_robot1.py`ou `gedit ../inf3995_ws/src/main_robot2.py` pour le second robot.  