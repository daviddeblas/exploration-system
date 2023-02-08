# Lancer la simulation

Premièrement, il faut build et run l'image docker en étant placer dans le root du dossier

```
docker compose run --build -p 5901:5901 sim
```
Ensuite executer ces lignes de commande dans le bash shell ouvert,et ajouter un mot de passe pour le Client VNC
```
cd /root
source .profile
TVNC_WM=openbox-session vncserver # will prompt for a password
```
Ensuite ouvrir **TurboVNC Viewer**
Puis se connecter à `localhost:5901` et mettre sont mot de passe
Dans TurboVNC, click droit pour ouvrir un nouveau terminal puis executer:
```
cd ../inf3995_ws
catkin_make && source devel/setup.bash
roslaunch limo_gazebo_sim limos.launch &
```

Finalement pour lancer le fichier main.py ouvrir un nouveau terminal (dans TurboVNC) et executer
```
python3 ../inf3995_ws/src/main.py
```
Si la station au sol roule également, il est possible d'utiliser les buton disponible pour interagir avec la simulation
