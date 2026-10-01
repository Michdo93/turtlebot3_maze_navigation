# turtlebot3_maze_navigation

Wall-Following und Pledge-Algorithmus für den TurtleBot3 Burger mit ROS 2 Humble.
Löst Labyrinthe reaktiv (ohne Karte) per LiDAR.

---

## Voraussetzungen

### 1. ROS 2 Humble installieren (Remote PC & TurtleBot)

```bash
# ROS 2 Humble (falls noch nicht installiert)
sudo apt update && sudo apt install -y ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. TurtleBot3-Pakete installieren

```bash
# Offizielle TB3-Pakete
sudo apt install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-bringup

# TurtleBot3-Modell setzen (in ~/.bashrc eintragen!)
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### 3. Build-Tools installieren

```bash
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip

# rosdep initialisieren (einmalig)
sudo rosdep init
rosdep update
```

---

## Installation

### Workspace erstellen und Package bauen

```bash
# Workspace anlegen
mkdir -p ~/tb3_maze_ws/src
cd ~/tb3_maze_ws/src

# Dieses Repository klonen (oder Dateien manuell hineinkopieren)
git clone <deine-repo-url> tb3_maze_navigation
# ODER: Ordner manuell nach ~/tb3_maze_ws/src/tb3_maze_navigation/ kopieren

# Dependencies automatisch installieren
cd ~/tb3_maze_ws
rosdep install --from-paths src --ignore-src -r -y

# Package bauen
colcon build --packages-select tb3_maze_navigation --symlink-install

# Workspace sourcen (in ~/.bashrc eintragen!)
echo "source ~/tb3_maze_ws/install/setup.bash" >> ~/.bashrc
source ~/tb3_maze_ws/install/setup.bash
```

### Warum --symlink-install?
Mit diesem Flag werden Python-Dateien verlinkt statt kopiert.
Du kannst den Code direkt editieren ohne nach jeder Änderung neu zu bauen.

---

## Verwendung

### Terminal 1 – TurtleBot starten (auf dem Robot oder Remote PC)

```bash
# Bringup auf dem TurtleBot (SSH ins Gerät)
ssh ubuntu@<turtlebot-ip>
ros2 launch turtlebot3_bringup robot.launch.py

# ROS_DOMAIN_ID muss auf beiden Geräten gleich sein!
export ROS_DOMAIN_ID=30
```

### Terminal 2 – Algorithmus starten (Remote PC)

```bash
# Wall-Following (einfacher, nur für einfach-zusammenhängende Labyrinthe)
ros2 run tb3_maze_navigation wall_following

# ODER: Pledge (robuster, funktioniert auch mit Inseln im Labyrinth)
ros2 run tb3_maze_navigation pledge

# ODER: Per Launch-File mit Algorithmus-Auswahl
ros2 launch tb3_maze_navigation maze.launch.py algorithm:=pledge
ros2 launch tb3_maze_navigation maze.launch.py algorithm:=wall_following
```

### Terminal 3 – Optional: Visualisierung in RViz2

```bash
rviz2
# Hinzufügen: LaserScan (/scan), TF, RobotModel
```

---

## Package-Struktur

```
tb3_maze_navigation/
├── package.xml                          # ROS2 Package-Metadaten & Dependencies
├── setup.py                             # Python-Package-Konfiguration & Entry Points
├── setup.cfg                            # Installationspfade
├── resource/
│   └── tb3_maze_navigation              # Marker-Datei für ament Index
├── launch/
│   └── maze.launch.py                   # Launch-File (algorithm:= Parameter)
├── tb3_maze_navigation/
│   ├── __init__.py
│   ├── wall_following.py                # Wall-Following Node
│   └── pledge.py                        # Pledge-Algorithmus Node
└── test/
    └── ...
```

---

## Algorithmen

### Wall-Following (wall_following.py)
- Subscribt: `/scan` (LaserScan)
- Publisht:  `/cmd_vel` (Twist)
- Strategie: Linke Wand in konstantem Abstand folgen (P-Regler)
- Limitierung: **Versagt bei Labyrinthen mit Inseln** (zirkuliert endlos)

### Pledge (pledge.py)
- Subscribt: `/scan` (LaserScan), `/odom` (Odometry)
- Publisht:  `/cmd_vel` (Twist)
- Strategie: Geradeaus bis Hindernis → Wand folgen bis Winkelzähler = 0
- Vorteil: **Funktioniert auch bei Inseln** im Labyrinth

---

## Tuning

Die wichtigsten Parameter stehen als Konstanten am Anfang jeder Datei:

| Parameter         | Default | Beschreibung                              |
|-------------------|---------|-------------------------------------------|
| DESIRED_WALL_DIST | 0.25 m  | Gewünschter Abstand zur Wand              |
| FRONT_CLEAR_DIST  | 0.35 m  | Ab wann gilt Weg vorne als blockiert      |
| LINEAR_SPEED      | 0.12 m/s| Vorwärtsgeschwindigkeit                   |
| ANGULAR_SPEED     | 0.7 r/s | Drehgeschwindigkeit beim Abbiegen         |
| KP_WALL           | 1.8     | P-Regler Verstärkung (Wandabstandsregel.) |

---

## Häufige Probleme

**Bot dreht sich nicht / bewegt sich nicht**
```bash
# Prüfen ob /scan Daten kommen
ros2 topic echo /scan --once

# Prüfen ob /cmd_vel published wird
ros2 topic echo /cmd_vel
```

**ROS_DOMAIN_ID Fehler (Bot und PC sehen sich nicht)**
```bash
export ROS_DOMAIN_ID=30   # Auf BEIDEN Geräten gleich setzen
```

**colcon build schlägt fehl**
```bash
# Dependencies nochmal installieren
rosdep install --from-paths src --ignore-src -r -y
```

**Bot fährt zu schnell / zu langsam**
→ `LINEAR_SPEED` und `ANGULAR_SPEED` in den jeweiligen Dateien anpassen.
Mit `--symlink-install` reicht es die Datei zu speichern, kein rebuild nötig.
