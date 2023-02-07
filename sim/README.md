# Lancer la simulation

Premièrement, il faut build et run l'image docker en étant placer dans le dossier sim

```
docker build -t inf3995-equipe101-simulation .
docker run -p 5901:5901 -it inf3995-equipe101-simulation bash
```

Ensuite éxecuter ces lignes de commande dans le bash shell ouvert,et ajouter un mot de passe pour le Client VNC
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
