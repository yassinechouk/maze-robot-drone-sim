# maze_robot_sim — Simulation Gazebo Harmonic + ROS 2 Jazzy

Simulation d'un robot mobile **différentiel** (0.25 m × 0.25 m) naviguant dans un **labyrinthe** (cellules 0.4 m × 0.4 m) du **start** au **goal**, avec **IMU** + **2 encodeurs de roue** (via `ros2_control`), un **marqueur ArUco ID=0**, et un **drone Crazyflie optionnel** (caméra top-down, suivi temps réel et atterrissage automatique).

Le système réutilise le format de grille (bitmask N/E/S/W) compatible avec le pipeline de vision et de planification du projet d'origine (`config.py`, `path_planning.py`).

---

## 1. Architecture du Système

- **Robot au sol (`maze_robot`)** :
  - Dimensions : 0.25 m × 0.25 m × 0.08 m.
  - Marqueur ArUco ID 0 (dictionnaire `DICT_4X4_50`) fixé sur le toit (`aruco_marker_link`).
  - Capteurs : IMU (`/imu`) et encodeurs de roues (`/joint_states`).
  - Contrôle : `diff_drive_controller` (`ros2_control`) recevant des commandes `TwistStamped` sur `/diff_drive_controller/cmd_vel`.
  - Navigation : `maze_navigator_node` avec auto-calibration de rotation et loi de commande continue ultra-fluide sans collision avec les murs (`math.cos(yaw_err)^8`).

- **Drone aérien (`crazyflie`)** *(Optionnel via `spawn_drone:=true`)* :
  - Modèle quadricoptère léger équivalent Crazyflie 2.x.
  - Vol contrôlé via le plugin native Gazebo Harmonic `MulticopterVelocityControl` (`gz::sim::systems::MulticopterVelocityControl`) sur `/drone/cmd_vel`.
  - Caméra orientée vers le bas (`/drone/camera/image_raw`).
  - Machine à états (`drone_mapper_node`) : `TAKEOFF` → `HOVER_MAP` (cartographie/observation) → `TRACKING` (asservissement temps réel sur le marqueur ArUco du robot) → `LANDING` (atterrissage sur le robot à l'arrivée au goal) → `DONE`.

---

## 2. Dépendances (Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic)

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-tf-transformations \
  ros-jazzy-cv-bridge \
  python3-transforms3d \
  python3-opencv
```

Si le dictionnaire ArUco d'OpenCV n'est pas inclus par défaut, assurez-vous d'avoir `opencv-contrib-python` :
```bash
pip install opencv-contrib-python
```

---

## 3. Compilation et Build

```bash
mkdir -p ~/maze_ws/src
cp -r maze_robot_sim ~/maze_ws/src/
cd ~/maze_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## 4. Lancer la Simulation

### Option A : Via le script d'exécution automatique (Recommandé)

Un script d'aide `run_sim.sh` est fourni à la racine. Il nettoie automatiquement les processus Gazebo fantômes en arrière-plan avant le démarrage.

**Robot seul :**
```bash
./run_sim.sh
```

**Robot + Drone Crazyflie :**
```bash
./run_sim.sh spawn_drone:=true
```

---

### Option B : Via la commande ROS 2 standard

**Lancement standard (Robot seul) :**
```bash
ros2 launch maze_robot_sim maze_sim.launch.py rows:=4 cols:=4 cell_size:=0.4 seed:=1
```

**Lancement avec Drone Crazyflie :**
```bash
ros2 launch maze_robot_sim maze_sim.launch.py spawn_drone:=true
```

---

## 5. Table des Arguments de Launch

| Argument | Défaut | Description |
|---|---|---|
| `rows` | `4` | Nombre de lignes de la grille du labyrinthe |
| `cols` | `4` | Nombre de colonnes de la grille |
| `cell_size` | `0.4` | Taille d'une cellule en mètres |
| `seed` | `1` | Graine pour le générateur aléatoire de labyrinthe (DFS backtracker) |
| `generate_world` | `true` | Si `true`, génère le monde SDF et le fichier JSON de métadonnées au lancement |
| `world_file` | (généré) | Chemin vers un monde `.sdf` pré-existant si `generate_world:=false` |
| `x_spawn` | `0.0` | Position X initiale du robot au sol |
| `y_spawn` | `1.2` | Position Y initiale du robot au sol (`(rows - 1) * cell_size` par défaut) |
| `invert_angular` | `""` (auto) | `""` = auto-calibration du sens de rotation au démarrage ; `"true"`/`"false"` pour forcer |
| `spawn_drone` | `false` | Activer le spawn du drone Crazyflie avec caméra et suivi ArUco (`true`/`false`) |
| `drone_x` | `0.0` | Position X initiale du drone |
| `drone_y` | `1.2` | Position Y initiale du drone (`(rows - 1) * cell_size` par défaut) |
| `drone_z` | `1.0` | Altitude de spawn initiale du drone en mètres |

---

## 6. Détails du Drone & Suivi ArUco (Nœud `drone_mapper_node`)

Lorsque `spawn_drone:=true` est activé, le nœud `drone_mapper_node` orchestre la séquence suivante :

1. **`TAKEOFF`** : Le drone décolle verticalement jusqu'à l'altitude de cartographie (par défaut `1.0 m`).
2. **`HOVER_MAP`** : Le drone maintient un vol stationnaire au-dessus du labyrinthe pendant 3 secondes pour capturer la vue top-down.
3. **`TRACKING`** : 
   - Le drone souscrit au flux vidéo `/drone/camera/image_raw`.
   - Il utilise `cv2.aruco.ArucoDetector` (dictionnaire `DICT_4X4_50`, ID 0) pour détecter la position du robot au sol.
   - Il applique une régulation P proportionnelle en XY pour maintenir l'image du robot centrée sous sa caméra pendant tout le parcours.
4. **`LANDING`** :
   - Lorsque `maze_navigator_node` signale la fin du parcours en publiant `GOAL_REACHED` sur `/maze_navigator/status`, le drone amorce une descente progressive tout en maintenant son asservissement XY sur le marqueur ArUco.
5. **`DONE`** : Le drone se pose sur le robot et coupe ses moteurs.

---

## 7. Navigation & Pilotage du Robot Au Sol

Le nœud `maze_navigator_node` gère le suivi autonome de la trajectoire A* :

- **Auto-calibration** : Effectue une légère rotation au départ pour mesurer le sens effectif de l'odométrie et ajuster `angular_sign`.
- **Suivi de trajectoire ultra-fluide** : Utilise un profil de vitesse linéaire pondéré par l'erreur de cap :
  $$v = v_{max} \cdot \cos(\text{yaw\_err})^8$$
  Cela permet au robot d'avancer de manière continue tout en réduisant automatiquement sa vitesse dans les virages pour éviter toute collision avec les murs.

---

## 8. Principaux Topics ROS 2

### Robot Au Sol
- `/diff_drive_controller/cmd_vel` (`geometry_msgs/msg/TwistStamped`) : Commande de vitesse du robot.
- `/diff_drive_controller/odom` (`nav_msgs/msg/Odometry`) : Odométrie des roues.
- `/imu` (`sensor_msgs/msg/Imu`) : Données IMU.
- `/maze_navigator/status` (`std_msgs/msg/String`) : Statut de navigation (`CALIBRATING`, `NAVIGATING`, `GOAL_REACHED`).

### Drone Crazyflie (avec `spawn_drone:=true`)
- `/drone/camera/image_raw` (`sensor_msgs/msg/Image`) : Flux vidéo top-down de la caméra embarquée.
- `/drone/cmd_vel` (`geometry_msgs/msg/Twist`) : Commande de vitesse du drone (bridgée vers Gazebo).
- `/drone/mapper/status` (`std_msgs/msg/String`) : État de la machine à états du drone (`TAKEOFF`, `MAPPING`, `TRACKING`, `LANDING`, `DONE`).

---

## 9. Correspondance avec le Projet Original

| Projet Vision (Python/Streamlit) | Ce Package ROS 2 / Gazebo |
|---|---|
| `config.py` : Bitmask N/E/S/W, `RobotParams` | `maze_world_generator.py` (même bitmask), `robot_core.xacro` (0.25 m × 0.25 m) |
| `computer_vision.py` : `image_to_walls()` → `walls`, `start`, `goal` | `--walls-json` accepte directement la grille bitmask exportée |
| `path_planning.py` : `astar()` | `astar()` réimplémenté à l'identique dans `maze_world_generator.py` |
| ArUco Detection (`cv2.aruco`) | `drone_mapper_node.py` (détection temps réel sur `/drone/camera/image_raw`) |
| Simulation GUI | Gazebo Harmonic 8.x + RViz2 + Topics ROS 2 |
