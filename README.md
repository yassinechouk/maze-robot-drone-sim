# maze_robot_sim — Simulation Gazebo Harmonic + ROS 2 Jazzy

Simulation d'un robot mobile **différentiel** (0.25 m × 0.25 m) naviguant dans un **labyrinthe** (cellules 0.4 m × 0.4 m) du **start** au **goal**, avec **IMU** + **2 encodeurs de roue** (via `ros2_control`), un **marqueur ArUco ID=0**, et un **drone Crazyflie optionnel** (caméra top-down, suivi temps réel et atterrissage automatique).

---

## 🎯 L'Approche Utilisée : Ce qui est Indépendant vs Coordonné

Le système est conçu autour d'une architecture modulaire où certaines briques fonctionnent en totale autonomie, tandis que d'autres coopèrent via le système de messagerie ROS 2.

### 1. Ce qui est INDÉPENDANT (Découplage)
* **Génération du Labyrinthe (`maze_world_generator.py`)** : Ce script mathématique génère le monde 3D (fichier `.sdf`) et calcule le chemin idéal via l'algorithme A* (fichier `.json`). Il n'a besoin ni de ROS ni de Gazebo pour fonctionner.
* **Le Robot au Sol (`maze_navigator_node`)** : Le robot est complètement **aveugle au drone**. Son seul objectif est de lire le chemin A* généré, et d'utiliser ses roues (odométrie) pour suivre les waypoints jusqu'à la ligne d'arrivée. Que le drone soit là, en panne, ou absent, le robot fera son parcours de manière autonome.
* **Le Modèle de Vol du Drone** : Le drone utilise un plugin Gazebo pur (`gz-sim-velocity-control-system`) qui réagit à des commandes géométriques. Il n'a aucune intelligence embarquée dans son modèle physique.

### 2. Ce qui FONCTIONNE ENSEMBLE (Coopération)
* **L'Asservissement Visuel (Le Drone suit le Robot)** : Le nœud du drone (`drone_mapper_node`) est totalement **dépendant** du robot. Il regarde le sol avec sa caméra, détecte le marqueur ArUco collé sur le toit du robot, et calcule l'erreur en X/Y pour ajuster sa propre vitesse et rester parfaitement au-dessus de lui. Si le robot accélère, le drone accélère. Si le robot tourne, le drone corrige sa trajectoire.
* **La Synchronisation de Fin de Parcours** : Comment le drone sait-il quand atterrir ? Le robot et le drone se parlent via un topic ROS (`/maze_navigator/status`). Quand le robot atteint le but, il crie `"GOAL_REACHED"`. Le drone écoute ce topic, comprend que la mission est terminée, et déclenche sa séquence de descente (`LANDING`) pile sur le robot.

---

## ⚙️ Workflow et Processus : Comment ça se déroule ?

Voici la chronologie exacte de ce qui se passe quand vous lancez la simulation (via `./run_sim.sh spawn_drone:=true`) :

### Phase 1 : Initialisation & Création du Monde
1. **Génération** : Le script de lancement appelle le générateur Python. Un labyrinthe aléatoire est créé (SDF) et la solution A* est sauvegardée en JSON.
2. **Apparition (Spawn)** : Gazebo démarre. Le robot apparaît au point de départ (start). Le drone apparaît exactement au-dessus du robot, à 1 mètre d'altitude.

### Phase 2 : Décollage et Cartographie
3. **Le Robot s'auto-calibre** : Le robot fait une micro-rotation sur lui-même pour vérifier le sens de ses encodeurs de roues (auto-calibration odométrique).
4. **Le Drone décolle (`TAKEOFF`)** : Pendant que le robot se prépare, le drone allume ses moteurs virtuels et monte à 1 mètre d'altitude.
5. **Observation (`HOVER_MAP`)** : Le drone se stabilise pendant 3 secondes pour (théoriquement) photographier le labyrinthe.

### Phase 3 : La Traque (Tracking)
6. **Navigation du Robot** : Le robot entame son parcours de manière fluide, ajustant sa vitesse automatiquement dans les virages pour ne pas toucher les murs.
7. **Suivi du Drone (`TRACKING`)** : La caméra du drone (qui pointe désormais parfaitement vers le sol grâce à une rotation de Pitch=90°, Yaw=180°) repère le marqueur ArUco du robot à 30 images par seconde. 

#### 🔍 Comment fonctionne le suivi d'objet (Visual Servoing) ?
Le drone utilise un **asservissement visuel en boucle fermée** :
- **Détection** : L'image est convertie en noir & blanc et `cv2.aruco.ArucoDetector` trouve le centre exact du marqueur ID 0.
- **Erreur (ex, ey)** : Le drone calcule l'écart entre ce marqueur et le centre parfait de sa caméra, normalisé de -1 à 1. 
  - `ex > 0` : le marqueur est à droite.
  - `ey < 0` : le marqueur est en haut de l'image.
- **Contrôle Proportionnel (P)** : Le nœud transforme cette erreur d'image en commande physique :
  - Si le marqueur est "en haut" (`ey < 0`, donc le robot avance physiquement), le drone commande `vx > 0` pour avancer et le rattraper.
  - Si le marqueur est "à droite" (`ex > 0`, donc le robot est décalé à gauche), le drone commande `vy > 0` pour translater vers la gauche.
- **Résultat** : Plus le marqueur s'éloigne du centre, plus le drone corrige vite (jusqu'à une vitesse max). Dès qu'il se rapproche du centre, le drone ralentit. C'est ce qui lui permet de "surfer" au-dessus du robot avec autant de fluidité !

### Phase 4 : Atterrissage
8. **Fin du Labyrinthe** : Le robot atteint la dernière cellule et s'arrête. Il publie le statut `GOAL_REACHED`.
9. **Descente (`LANDING`)** : Le drone reçoit le signal. Il abaisse doucement son altitude cible (0.12m) tout en continuant à centrer le marqueur ArUco pour atterrir précisément sur le toit du robot.
10. **Mission Terminée (`DONE`)** : Les moteurs du drone se coupent.

---

## 🛠 Architecture du Système (Technique)

- **Robot au sol (`maze_robot`)** :
  - Dimensions : 0.25 m × 0.25 m × 0.08 m.
  - Marqueur ArUco ID 0 (dictionnaire `DICT_4X4_50`) fixé sur le toit (`aruco_marker_link`).
  - Capteurs : IMU (`/imu`) et encodeurs de roues (`/joint_states`).
  - Contrôle : `diff_drive_controller` (`ros2_control`) via `/diff_drive_controller/cmd_vel`.

- **Drone aérien (`crazyflie`)** *(Optionnel via `spawn_drone:=true`)* :
  - Modèle quadricoptère léger équivalent Crazyflie 2.x.
  - Vol contrôlé via le plugin native Gazebo Harmonic `VelocityControl` sur `/crazyflie/cmd_vel`.
  - Caméra orientée vers le bas (`/drone/camera/image_raw`).
  - Machine à états (`drone_mapper_node`) avec contrôle Deadband d'altitude.

---

## 📦 Dépendances (Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic)

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
*(Si le dictionnaire ArUco n'est pas inclus : `pip install opencv-contrib-python`)*

---

## 🚀 Lancer la Simulation

### Via le script d'exécution automatique (Recommandé)

Un script `run_sim.sh` est fourni. Il nettoie automatiquement les processus fantômes avant démarrage.

**Robot seul :**
```bash
./run_sim.sh
```

**Robot + Drone Crazyflie :**
```bash
./run_sim.sh spawn_drone:=true
```

### Table des Arguments de Launch principaux (`maze_sim.launch.py`)

| Argument | Défaut | Description |
|---|---|---|
| `rows` | `4` | Nombre de lignes du labyrinthe |
| `cols` | `4` | Nombre de colonnes du labyrinthe |
| `cell_size` | `0.4` | Taille d'une cellule en mètres |
| `seed` | aléatoire | Graine pour générer un labyrinthe spécifique |
| `spawn_drone` | `false` | Activer le spawn du drone Crazyflie avec caméra et suivi ArUco |

---

## 📡 Principaux Topics ROS 2 (Communication)

### Robot Au Sol
- `/diff_drive_controller/cmd_vel` : Commande de vitesse des roues.
- `/maze_navigator/status` : Statut public du robot (`CALIBRATING`, `NAVIGATING`, `GOAL_REACHED`). C'est le **canal de communication clé** entre le robot et le drone.

### Drone Crazyflie
- `/drone/camera/image_raw` : L'œil du drone (Flux vidéo top-down).
- `/crazyflie/cmd_vel` : Les muscles du drone (Commande de vitesse envoyée à Gazebo).
- `/drone/mapper/status` : L'état du cerveau du drone (`TAKEOFF`, `TRACKING`, `LANDING`).
