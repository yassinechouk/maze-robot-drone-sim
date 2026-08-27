<div align="center">

# maze-robot-drone-sim

### Gazebo Harmonic + ROS 2 Jazzy — Un drone cartographie, un robot navigue

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?logo=gazebo)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.12-yellow?logo=python)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-vision-green?logo=opencv)](https://opencv.org/)
[![CasADi](https://img.shields.io/badge/CasADi-MPC%20%2F%20IPOPT-purple)](https://web.casadi.org/)
[![License](https://img.shields.io/badge/License-MIT-lightgrey)](LICENSE)

</div>

---

## Simulation en action

| Phase 1 — Démarrage & Décollage | Phase 2 — Navigation du Robot |
|:---:|:---:|
| ![start](media/sim_01_start.png) | ![takeoff](media/sim_02_takeoff.png) |
| Le drone décolle au-dessus du robot | Le robot suit le chemin qu'il a planifié |

| Phase 3 — Suivi ArUco (Tracking) | Phase 4 — Arrivée au Goal |
|:---:|:---:|
| ![tracking](media/sim_03_tracking.png) | ![goal](media/sim_04_goal.png) |
| Le drone suit le robot via sa caméra | Le robot atteint le goal (🔴) et le drone atterrit |

> **Vert** = Start · **Rouge** = Goal · **Petit cube bleu** = Drone Crazyflie · **Robot gris** = Robot différentiel avec marqueur ArUco

---

## Vue d'ensemble

Un **drone Crazyflie** décolle au-dessus d'un labyrinthe qu'il n'a jamais vu, se
cadre tout seul jusqu'à en voir la totalité, en extrait la carte par vision, la
publie sur ROS 2 — puis un **robot différentiel** reçoit cette carte, planifie
son propre chemin et le suit en **commande prédictive**, dans des couloirs où il
ne lui reste que 2,3 cm de jeu de chaque côté.

Le point important est ce qui *ne* circule *pas* : le robot ne lit aucun fichier,
et le drone ne reçoit ni les dimensions du labyrinthe, ni la taille des cellules,
ni la position du départ ou de l'arrivée. Tout cela est redécouvert depuis une
image caméra.

### Fonctionnalités clés

| Feature | Description |
|---|---|
| **Cartographie aérienne** | Le drone extrait la grille de murs d'une seule image nadir, avec vote sur plusieurs prises |
| **Dimensions auto-détectées** | `rows`, `cols` et la taille de cellule sont déduits des lignes de grille, pas fournis |
| **Cadrage autonome** | Le drone ajuste position et altitude jusqu'à cadrer tout le labyrinthe, quelle que soit sa taille |
| **Carte ROS 2 standard** | `nav_msgs/OccupancyGrid` latchée, directement visualisable dans RViz |
| **Planification embarquée** | Le robot rejoue son propre A* sur la carte reçue |
| **Commande prédictive** | MPC CasADi/IPOPT sur horizon 1,8 s, avec pénalités de murs |
| **Rendez-vous avant départ** | Le drone revient à 1 m au-dessus du robot et ne donne le départ qu'une fois stabilisé sur lui |
| **Suivi visuel** | Asservissement ArUco ID 0 (`DICT_4X4_50`) du drone sur le robot |
| **Validation intégrée** | Un nœud de diagnostic compare la carte vue à la grille réellement construite |

---

## Architecture

```
maze_robot_sim/
├── maze_robot_sim/
│   ├── maze_map.py               # Format d'échange de la carte + A* (partagé)
│   ├── maze_vision.py            # Image nadir → grille de murs (pipeline OpenCV)
│   ├── mpc_controller.py         # MPC CasADi + suivi de trajectoire
│   ├── drone_mapper_node.py      # Drone : cadrage, cartographie, suivi, atterrissage
│   ├── maze_navigator_node.py    # Robot : réception carte, A*, MPC
│   ├── maze_map_validator.py     # Diagnostic : carte vue vs vérité terrain
│   └── maze_world_generator.py   # Génération du monde SDF (DFS)
├── description/                  # URDF/SDF robot + drone
├── launch/maze_sim.launch.py     # Launch principal
├── config/diff_drive_controller.yaml
├── worlds/                       # Monde SDF généré automatiquement
└── run_sim.sh
```

### Qui sait quoi

```
┌─────────────────────────────────────────────────────────────────────┐
│  maze_world_generator   (hors ROS, hors Gazebo)                     │
│  DFS → monde SDF. Écrit aussi un JSON de vérité terrain, utilisé    │
│  uniquement par le validateur — jamais par le robot ni le drone.    │
└─────────────────────────────────────────────────────────────────────┘
                                  │ construit le monde
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│  drone_mapper_node                    │  maze_navigator_node        │
│                                       │                             │
│  décolle, se cadre, extrait la grille │  attend la carte            │
│  ────────── /maze/occupancy_grid ─────┼──▶ décode, A*, MPC          │
│  ────────── /maze/start_pose ─────────┼──▶                          │
│  ────────── /maze/goal_pose ──────────┼──▶                          │
│  revient à 1 m au-dessus du robot     │                             │
│  ────────── /drone/mapper/ready ──────┼──▶ démarre                  │
│                                       │                             │
│  suit le marqueur ArUco  ◀── caméra ──┼─── robot en mouvement       │
│  atterrit                        ◀────┼─── /maze_navigator/status   │
│                                       │       GOAL_REACHED          │
└─────────────────────────────────────────────────────────────────────┘
```

### Séquence complète

```
① Génération  Labyrinthe aléatoire (DFS) → monde SDF
② Spawn       Robot au départ · Drone posé au-dessus de lui
③ Décollage   Le drone monte, et relève au passage la position du robot
              (le marqueur de 12 cm est illisible depuis l'altitude de carto)
④ Cadrage     Il se centre sur les murs et ajuste son altitude jusqu'à ce que
              tout le labyrinthe tienne dans l'image avec de la marge
⑤ Vision      5 extractions successives → vote → grille de murs consensuelle
⑥ Publication OccupancyGrid + poses départ/arrivée, latchées
⑦ Retour      Le drone revient au-dessus du robot et descend à 1 m de lui.
              C'est SEULEMENT une fois stabilisé là qu'il publie READY
⑧ Navigation  Le robot calibre sa convention angulaire, planifie son A*,
              et suit le chemin en MPC · Le drone le suit à l'ArUco
⑨ Arrivée     Le robot publie GOAL_REACHED · Le drone se pose
```

---

## Méthodes implémentées

Chaque brique et son point d'entrée dans le code.

### Vision — [`maze_vision.py`](maze_robot_sim/maze_vision.py)

| Méthode | Rôle | Fonction |
|---|---|---|
| Seuillage HSV bi-plage | Isoler les murs rouges malgré la discontinuité de teinte du rouge | `red_mask` / `green_mask` |
| Analyse de composantes connexes par forme | Séparer la pastille d'arrivée des murs, qui ont la même couleur | `find_disc` |
| Ouverture morphologique linéaire | Séparer traits horizontaux et verticaux de l'image | `_line_masks` |
| Dimensionnement du noyau par la géométrie | Choisir une longueur d'ouverture supérieure à la largeur apparente d'un mur, inférieure à une cellule | `_line_kernel_length` |
| Profils de projection + détection de plages | Localiser les lignes de la grille | `_runs_above`, `_detect_lines` |
| Résolution de la parallaxe à deux bords | Ramener chaque ligne au plan du sol sans connaître l'épaisseur des murs | `_run_edges`, `_wall_baseline` |
| Inférence conjointe des dimensions sous contrainte de cellules carrées | Déduire `rows` et `cols` sans qu'on les lui donne | `_score_count`, `_candidate_counts`, `infer_grid_shape` |
| Régression aux moindres carrés de la grille | Ajuster le pas sur toutes les lignes vues, pas sur les deux extrêmes | `_regular_lines` |
| Test de présence par couverture maximale sur bande | Lire chaque arête de cellule sans être sensible à la largeur de bande | `_edge_present`, `_read_walls` |
| Projection pinhole nadir + correction d'assiette | Convertir pixel ↔ monde, en tenant compte du tangage résiduel | `image_to_world`, `nadir_ground_point` |
| Détection ArUco `DICT_4X4_50` sous-pixellique | Situer le robot dans le repère monde | `locate_robot_world` |
| Vote majoritaire multi-images | Éliminer les murs manqués sur une image isolée | `vote_observations` |
| Deux passes auto-calibrées | Découvrir la taille de cellule, puis rejouer la segmentation avec des noyaux calibrés dessus | `extract_maze` |

### Carte et planification — [`maze_map.py`](maze_robot_sim/maze_map.py)

| Méthode | Rôle | Fonction |
|---|---|---|
| Encodage bitmask → `OccupancyGrid` | Format de fil unique, partagé par les deux nœuds | `walls_to_grid_data` / `grid_data_to_walls` |
| A* Manhattan sur grille bitmask | Planification du chemin par le robot lui-même | `astar` |
| Blocage symétrique | Un mur déclaré d'un seul côté bloque quand même : une grille issue de la vision peut être asymétrique, et traverser coûte plus cher qu'un détour | `astar.blocked` |
| Conversion grille ↔ monde | Centres de cellules, origine, indices | `cell_center_world`, `world_to_cell`, `grid_origin` |
| Génération des segments de murs | Géométrie consommée par le MPC | `wall_segments` |

### Commande — [`mpc_controller.py`](maze_robot_sim/mpc_controller.py)

| Méthode | Rôle | Fonction |
|---|---|---|
| MPC non linéaire CasADi/IPOPT | Suivi de trajectoire anticipatif | `MPCController` |
| Modèle unicycle, intégration point-milieu | Dynamique du différentiel, exacte en virage | `MPCController.__init__` |
| Pénalité d'alignement `(v·sin Δθ)²` | Formalisation continue du `cos^8` historique | idem |
| Pénalités douces de murs, nombre de créneaux fixe | Anti-collision sans rendre le problème infaisable, et sans changer la structure du NLP | `_segment_distance`, `select_walls` |
| Réamorçage à chaud décalé d'un pas | Diviser le temps de résolution ; conservé même après un échec pour éviter les cascades | `solve` |
| Invalidation explicite du réamorçage | Éviter le *tunneling* à travers un mur quand le problème change | `reset` |
| Plafond de temps de résolution | Empêcher le solveur de figer la boucle mono-thread | option `ipopt.max_wall_time` |
| Suivi par abscisse curviligne, projection vers l'avant | Générer la référence sans accrocher un couloir parallèle | `PathTracker` |
| Repli proportionnel « tourner puis avancer » | Garder le robot sûr le temps d'un cycle raté | `fallback_command` |

### Drone — [`drone_mapper_node.py`](maze_robot_sim/drone_mapper_node.py)

| Méthode | Rôle | Fonction |
|---|---|---|
| Cadrage autonome par barycentre et taux de remplissage | Trouver seul la position et l'altitude qui cadrent tout le labyrinthe | `_do_framing` |
| Correction d'altitude géométrique directe | Le remplissage varie en 1/altitude : une règle de trois converge en un pas là où un pas fixe oscille | idem |
| Retour à double repère (position monde puis asservissement visuel) | Revenir au-dessus du robot avant de donner le départ | `_do_return` |
| Asservissement visuel proportionnel sur ArUco | Suivi du robot et centrage à l'atterrissage | `_aruco_error`, `_aruco_velocity` |
| Rejet des images trop inclinées | Le modèle affine ne vaut que drone à plat | `_camera_ground_frame` |
| Publication latchée (`TRANSIENT_LOCAL`) | Rendre l'ordre de démarrage des nœuds indifférent | `latched_qos` |

---

## Comment le drone lit le labyrinthe

### Géométrie image ↔ monde

La caméra du Crazyflie pointe vers le bas (`rpy = 0 π/2 π` dans le repère du
drone). En déroulant la convention optique de Gazebo, la correspondance est
purement affine tant que le drone est à plat :

```
u (colonne image, vers la droite)  →  +Y monde
v (ligne image, vers le bas)       →  +X monde

u = u₀ + (Y − cam_y) · f / (alt − z)
v = v₀ + (X − cam_x) · f / (alt − z)
```

Le drone connaît sa propre pose (`/model/crazyflie/pose`), donc tout pixel du sol
se convertit directement en coordonnées monde. C'est ce qui permet de publier une
carte géoréférencée plutôt qu'une grille flottante.

`cam_x` et `cam_y` désignent le point du sol visé par l'axe optique, pas la
position du drone : un multirotor en stationnaire garde un ou deux degrés
d'assiette résiduelle, ce qui déplace déjà le point visé de plusieurs centimètres
à trois mètres d'altitude. Négliger l'assiette translate toute la carte de ~10 cm ;
la corriger ramène l'erreur d'origine à ~3 mm. Au-delà de quelques degrés
l'image est franchement déformée et non plus seulement décalée : le nœud écarte
alors la prise et attend que le drone se stabilise.

### La parallaxe des murs, et pourquoi elle compte

Les murs ne sont pas plats : hauts de 15 cm, ils apparaissent *étalés vers
l'extérieur* de l'image. Un mur d'épaisseur `t` à la distance radiale `R` occupe
la plage allant de la base de son flanc intérieur à la crête de son flanc
extérieur :

```
d_proche = (R − t/2) · f / alt          d_loin = (R + t/2) · f / (alt − h)
```

Deux mesures, deux inconnues : on en tire `R·f = (alt·d_proche + (alt−h)·d_loin)/2`
**sans avoir à connaître l'épaisseur des murs**. Se contenter du bord intérieur
laisserait un biais de `t/2` vers le centre sur chaque ligne, donc un labyrinthe
reconstruit systématiquement trop petit — et avec 2,3 cm de jeu, quelques
millimètres d'erreur par cellule suffisent à envoyer le robot dans une cloison.

### Pipeline

```
image BGR
   │
   ├─ masque HSV du rouge ─────────────► murs + pastille d'arrivée
   │
   ├─ la pastille est isolée par sa FORME (composante compacte et pleine),
   │  pas par sa couleur : elle est rouge exactement comme les murs
   │
   ├─ ouvertures morphologiques linéaires ─► traits horizontaux / verticaux
   │  (longueur du noyau calculée depuis la parallaxe : plus large qu'un mur
   │   vu de biais, plus courte qu'une cellule)
   │
   ├─ profils de projection ─► plages ─► position de chaque ligne de grille
   │
   ├─ choix conjoint de (rows, cols) : le critère décisif est que les cellules
   │  sont CARRÉES — une erreur de facteur 2 sur un seul axe est trahie par le
   │  rapport des pas
   │
   ├─ lecture des murs arête par arête (couverture maximale sur une bande)
   │
   └─ départ = ArUco du robot · arrivée = pastille rouge · origine et taille
      de cellule en mètres
```

Chaque extraction est indépendante ; le nœud en accumule cinq et n'en retient
qu'un consensus. Une image isolée peut manquer un mur, et **manquer un mur est
bien plus grave qu'en inventer un** : cela autorise un chemin qui traverse une
cloison réelle.

En cas d'échec, `extract_maze` lève une exception explicite plutôt que de rendre
une grille douteuse — le drone réessaie sur l'image suivante, et remonte s'il
n'y arrive pas. Sur banc de test hors Gazebo (labyrinthes 2×2 à 10×10, altitudes
et décentrages variés), 114 cas sur 120 aboutissent et **aucun ne produit de
carte fausse**.

### Le retour au-dessus du robot

Pour cartographier, le drone doit s'éloigner : il se place au centre du
labyrinthe et monte à trois mètres ou plus. Donner le départ depuis là ferait
s'élancer le robot alors que le drone est encore à plusieurs cellules de
distance — le suivi commencerait en rattrapage, marqueur hors champ.

Le drone revient donc d'abord, et deux repères se relaient pendant la descente :

- **De haut**, le marqueur de 12 cm est trop petit pour être lu. Le drone vise la
  position monde du robot, relevée pendant la montée initiale et toujours valable
  puisque le robot attend précisément ce signal pour bouger.
- **En descendant**, le marqueur redevient lisible et l'asservissement visuel
  prend le relais : plus précis, et insensible à une erreur sur la position
  mémorisée.

`READY` n'est publié qu'une fois la hauteur de suivi atteinte *et* le drone
stabilisé au-dessus du marqueur. Une image sans détection ne remet pas le
compteur de stabilisation à zéro : c'est une absence d'information, pas une
preuve que le drone a bougé. Un délai de garde donne le départ malgré tout si le
retour n'aboutit pas — le robot ne doit jamais rester bloqué à cause du drone.

> **Un cube de 2 cm faisait rater une image sur deux.** Le visuel du lien IMU
> était posé sur la face supérieure du châssis et dépassait de 4 mm au-dessus du
> plan du marqueur, pile en son centre : le drone voyait un carré rouge au milieu
> du motif et les bits étaient corrompus. Loger l'IMU au centre du châssis — sa
> place naturelle — a fait passer le taux de détection de **40 % à 91 %** et
> supprimé les pertes de marqueur pendant le suivi.

---

## Comment le robot suit son chemin

### La contrainte qui dicte tout

| Grandeur | Valeur |
|---|---|
| Châssis du robot | 0,25 × 0,25 m |
| Demi-diagonale (disque circonscrit) | **0,177 m** |
| Demi-largeur de couloir | 0,20 m |
| **Jeu résiduel** | **0,023 m** |

Un différentiel qui pivote sur place balaie son disque circonscrit : c'est donc
0,177 m qu'il faut faire tenir dans 0,20 m. Il ne reste rien pour un dépassement
en virage, ce qui exclut tout lissage par courbes continues (le rayon de courbure
admissible serait de 2,3 cm) et impose un contrôleur qui *anticipe*.

### Le MPC

À chaque cycle, le contrôleur résout :

```
min  Σ_k  ‖x_k − x_ref,k‖²_Q + ‖u_k‖²_R + ‖u_k − u_{k−1}‖²_R∆
          + w_align · (v_k · sin(θ_k − θ_ref,k))²
          + w_mur   · Σ_murs  max(0, marge − d(p_k, mur))²

s.c. x_{k+1} = f(x_k, u_k)        modèle unicycle, intégration point-milieu
     0 ≤ v ≤ v_max,  |ω| ≤ ω_max
     |Δv| ≤ a_max·Δt,  |Δω| ≤ α_max·Δt
```

Deux choix méritent d'être explicités.

**Le terme d'alignement** `(v · sin(θ − θ_ref))²` pénalise le fait d'avancer vite
en étant mal orienté. C'est la version continue et optimisable du facteur
`cos(erreur)^8` du contrôleur historique : là où celui-ci réagissait à l'erreur
courante, celui-ci la voit venir sur tout l'horizon et freine *avant* le virage.

**Les murs sont des pénalités douces, pas des contraintes dures.** Une contrainte
dure `d ≥ marge` rend le problème infaisable dès que le robot dérive à l'intérieur
de la marge, et le solveur renvoie une erreur au lieu d'une commande — exactement
au moment où l'on en a le plus besoin. La pénalité douce reste toujours faisable.
En contrepartie elle expose au *tunneling* : un réamorçage à chaud issu d'un autre
problème peut figer l'optimiseur de l'autre côté d'un mur, là où la pénalité
redevient nulle. D'où `MPCController.reset()`, appelé dès que le problème change
de nature.

> Aucune formulation ne garantit mathématiquement zéro collision si l'odométrie
> dérive. Le MPC anticipe mieux qu'un correcteur proportionnel parce qu'il
> intègre la prédiction, mais la garantie repose sur la qualité du modèle et des
> capteurs.

Le solveur est plafonné en temps de calcul (`mpc_max_solve_time`). Le nœud est
mono-thread : une résolution qui s'éternise fige aussi la lecture de l'odométrie,
et le robot continue à l'aveugle sur son dernier ordre. Mieux vaut abandonner et
basculer sur un repli proportionnel le temps d'un cycle.

Un échec ne remet pas le réamorçage à zéro. Il provient du même problème décalé
d'un pas et reste le meilleur point de départ connu ; l'effacer condamnerait le
cycle suivant à repartir à froid, donc à être plus lent, donc à échouer aussi —
un échec isolé se transformerait en cascade. C'est ce qui rend la dégradation
progressive : sur une machine saturée, le robot bascule simplement sur le repli
et avance plus lentement au lieu de s'arrêter.

---

## Ce qui a été écarté, et pourquoi

Documenter les impasses vaut souvent mieux que documenter la solution : elles
expliquent pourquoi le code a la forme qu'il a.

| Approche | Verdict | Raison |
|---|---|---|
| **Lissage par clothoïdes** (courbure continue en virage) | Écartée | Le rayon de courbure admissible vaut `0,20 − 0,177 = 2,3 cm`. Un différentiel ne peut pas décrire une courbe continue dans un virage à 90° d'une cellule de 40 cm : il doit pivoter quasi sur place. Mathématiquement séduisant, physiquement inapplicable à cette géométrie. Il faudrait des cellules de 80 cm. |
| **Profil de vitesse trapézoïdal** | Non implémenté | Le MPC produit le même effet — décélération anticipée avant les virages — mais depuis la structure du problème plutôt qu'un profil pré-calculé. Le maintenir en parallèle ferait deux sources de vérité sur la vitesse. |
| **Nav2 / MPPI** | Écartée | Costmaps, lifecycle manager, behaviour tree : une pile industrielle entière pour un labyrinthe statique de 16 cellules. Disproportionné, et la contrainte à 2,3 cm demande un contrôle explicite des marges que la pénalité de costmap ne donne pas. |
| **Contraintes de murs *dures* dans le MPC** | Écartée | `d ≥ marge` rend le problème infaisable dès que le robot dérive dans la marge — IPOPT renvoie alors une erreur au lieu d'une commande, précisément quand on en a le plus besoin. Les pénalités douces dégradent proprement. |
| **Vérificateur de collision indépendant** | Fondu dans le MPC | Il lirait la même odométrie que le MPC et serait aveugle de la même façon. La surveillance des murs vit donc *dans* le problème d'optimisation, évaluée sur toute la trajectoire prédite. |
| **Repli JSON pour le robot** | Supprimé | Le garder aurait dilué l'architecture : tant qu'un chemin de secours lit le fichier, on ne démontre pas que la vision suffit. |
| **Réglage plus permissif du détecteur ArUco** | Sans effet | Mesuré sur le vrai flux : 39,7 % contre 41,3 %. Le problème était géométrique — le cube de l'IMU au centre du marqueur — pas algorithmique. Chercher d'abord la cause, pas le paramètre. |
| **Dimensions du labyrinthe passées au drone** | Écartée | C'est ce qui reste à démontrer. La contrainte de cellules carrées rend la détection conjointe fiable sans cette béquille. |

---

## Prérequis

**Système : Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic**

```bash
sudo apt update && sudo apt install -y \
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
  ros-jazzy-tf2-ros \
  ros-jazzy-cv-bridge \
  python3-transforms3d \
  python3-opencv
```

CasADi n'a pas de paquet ROS ; il s'installe avec pip :

```bash
pip install --user --break-system-packages casadi
```

> Si les dictionnaires ArUco manquent à votre OpenCV : `pip install opencv-contrib-python`

---

## Lancer la simulation

### 1. Build

```bash
cd maze_robot_sim
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Lancer

```bash
./run_sim.sh
```

Avec un labyrinthe reproductible et le rapport de validation de la carte :

```bash
./run_sim.sh seed:=42 rows:=5 cols:=5 validate_map:=true
```

> Le drone est **indispensable** : c'est lui qui fournit la carte. Avec
> `spawn_drone:=false`, le robot attend une carte que personne ne publie et ne
> bouge pas — le launch le signale explicitement au démarrage.

### Arguments disponibles

| Argument | Défaut | Description |
|---|---|---|
| `rows` | `4` | Lignes du labyrinthe **généré** (le drone les redécouvre) |
| `cols` | `4` | Colonnes du labyrinthe **généré** |
| `cell_size` | `0.4` | Taille d'une cellule en mètres |
| `seed` | aléatoire | Graine pour reproduire un labyrinthe |
| `mapping_altitude` | `2.5` | Altitude initiale de cartographie, ensuite ajustée seule |
| `validate_map` | `false` | Compare la carte vue à la grille réellement construite |
| `spawn_drone` | `true` | Spawn du drone (sans lui, rien ne démarre) |
| `invert_angular` | `""` | Forcer l'inversion angulaire (`true`/`false`/auto) |

---

## Topics ROS 2

### Carte (drone → robot)

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/maze/occupancy_grid` | `nav_msgs/OccupancyGrid` | latché | Grille extraite, résolution `cell/4` |
| `/maze/start_pose` | `geometry_msgs/PoseStamped` | latché | Centre de la cellule de départ |
| `/maze/goal_pose` | `geometry_msgs/PoseStamped` | latché | Centre de la cellule d'arrivée |
| `/drone/mapper/ready` | `std_msgs/String` | latché | `READY`, publié après la carte |

Le latching (`TRANSIENT_LOCAL`) fait que l'ordre de démarrage des nœuds n'a pas
d'importance : un abonné tardif reçoit quand même la dernière carte publiée.

### Robot (`maze_navigator_node`)

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` | ← Sub | Odométrie roues |
| `/diff_drive_controller/cmd_vel` | `geometry_msgs/TwistStamped` | → Pub | Commande vitesse |
| `/maze_navigator/status` | `std_msgs/String` | → Pub | `CALIBRATING` / `NAVIGATE_x%` / `GOAL_REACHED` |
| `/maze_navigator/path` | `nav_msgs/Path` | → Pub | Chemin A* planifié (latché) |
| `/maze_navigator/diagnostics` | `std_msgs/String` | → Pub | `v`, `ω`, distance aux murs, temps de résolution MPC |
| `/imu` | `sensor_msgs/Imu` | ← Sub | IMU (souscrit, non exploité) |

### Drone (`drone_mapper_node`)

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/drone/camera/image_raw` | `sensor_msgs/Image` | ← Sub | Flux caméra nadir |
| `/model/crazyflie/pose` | `geometry_msgs/Pose` | ← Sub | Pose réelle Gazebo |
| `/crazyflie/cmd_vel` | `geometry_msgs/Twist` | → Pub | Commandes vitesse |
| `/crazyflie/enable` | `std_msgs/Bool` | → Pub | Activation du contrôleur de vol |
| `/drone/mapper/status` | `std_msgs/String` | → Pub | `TAKEOFF` / `FRAMING` / `MAPPING` / `RETURN` / `TRACKING` / `LANDING` / `DONE` |
| `/maze_navigator/status` | `std_msgs/String` | ← Sub | Signal `GOAL_REACHED` |

---

## Paramètres

### `maze_navigator_node`

| Paramètre | Défaut | Description |
|---|---|---|
| `mpc_horizon` | `18` | Pas de prédiction (1,8 s à `mpc_dt=0.10`) |
| `mpc_dt` | `0.10 s` | Pas de discrétisation, aligné sur la période de commande |
| `mpc_decimation` | `2` | Résolution à 10 Hz, commande publiée à 20 Hz |
| `mpc_max_solve_time` | `0.15 s` | Plafond de temps de résolution avant repli |
| `max_linear_speed` | `0.30 m/s` | Vitesse linéaire maximale |
| `max_angular_speed` | `1.0 rad/s` | Vitesse angulaire maximale (bridée : voir dérive) |
| `max_acceleration` | `0.6 m/s²` | Aligné sur `diff_drive_controller.yaml` |
| `robot_radius` | `0.177 m` | Disque circonscrit du châssis |
| `safety_margin` | `0.023 m` | Marge de pénalité au-delà du rayon |
| `wall_weight` | `25000` | Poids de la pénalité de mur |
| `align_weight` | `45` | Poids du terme « avancer désaligné » |

### `drone_mapper_node`

| Paramètre | Défaut | Description |
|---|---|---|
| `mapping_altitude` | `2.5 m` | Altitude initiale, ajustée par le cadrage |
| `mapping_altitude_min` / `_max` | `1.2` / `8.0 m` | Bornes de l'ajustement automatique |
| `frame_fill_min` / `_max` | `0.45` / `0.80` | Fraction d'image que doit occuper le labyrinthe |
| `frame_center_tol` | `0.05` | Tolérance de centrage (fraction de demi-image) |
| `vision_samples` | `5` | Extractions à fusionner par vote |
| `vision_min_agreement` | `0.6` | Fraction d'accord requise pour retenir un mur |
| `camera_hfov` | `1.047 rad` | Champ horizontal, doit suivre le SDF |
| `wall_height` | `0.15 m` | Hauteur des murs, utilisée pour la parallaxe |
| `max_tilt_rad` | `0.10 rad` | Assiette au-delà de laquelle une image est écartée |
| `tracking_height_above_robot` | `1.0 m` | Hauteur de vol **au-dessus du robot** pendant le suivi |
| `return_center_tol` | `0.08` | Décentrage toléré pour se déclarer au-dessus du robot |
| `return_settle_sec` | `1.0 s` | Stabilisation exigée avant de donner le départ |
| `return_timeout_sec` | `40 s` | Délai de garde : départ donné même si le retour échoue |
| `landing_altitude` | `0.12 m` | Altitude d'arrêt à l'atterrissage |

---

## Modèles physiques

### Robot différentiel (`maze_robot`)
- Dimensions : **0,25 × 0,25 × 0,08 m**, entraxe de roues 0,22 m, rayon 0,045 m
- Entraînement : `diff_drive_controller` via `ros2_control`
- Capteurs : IMU + encodeurs de roues (`/joint_states`)
- Marqueur : ArUco ID 0 (`DICT_4X4_50`), 12 cm, sur le toit à z = 0,086 m

### Drone Crazyflie
- Quadricoptère équivalent Crazyflie 2.x
- Vol contrôlé par `gz-sim-velocity-control-system`
- Caméra nadir 640 × 480 @ 30 Hz, HFOV 60°

---

## Validation

`validate_map:=true` lance `maze_map_validator`, qui confronte la carte publiée au
JSON de vérité terrain écrit par le générateur. Il ne participe à aucune décision
de navigation — c'est un instrument de mesure.

```
═══ Validation de la carte extraite par vision ═══
  dimensions : vue 5x5 | réelle 5x5 → OK
  cellule    : vue 400.8 mm | réelle 400.0 mm → écart 0.8 mm
  origine    : vue (-3, -0) mm | réelle (0, 0) mm → écart 3.4 mm
  murs       : 100/100 corrects → PARFAIT
  départ     : vu (0, 0) | réel (0, 0) → OK
  arrivée    : vu (4, 4) | réel (4, 4) → OK
  chemin A*  : 9 cellules vues | 9 réelles → IDENTIQUE
```

Un décalage d'origine résiduel serait d'ailleurs sans effet sur la navigation :
c'est une translation appliquée à la fois aux waypoints et à la pose de départ
dont le robot tire son origine d'odométrie, si bien que les deux se compensent
exactement. Ce qui compte vraiment est la **taille de cellule**, qui elle se
cumule le long du chemin — d'où le soin apporté à la correction de parallaxe.

### Ce qui est mesuré, et ce qui ne l'est pas

| Critère | Cible | Mesuré |
|---|---|---|
| Grille de murs correcte | 100 % | **100 %** (4×4, 5×5, 6×6 · 64, 100 et 144 arêtes) |
| Écart sur la taille de cellule | — | **0,4 à 0,8 mm** |
| Écart sur l'origine de la carte | — | **2 à 3 mm** |
| Chemin A* identique au réel | oui | **oui**, sur les trois tailles |
| Jeu robot/mur (disque circonscrit) | ≥ 1,5 cm | **1,0 à 1,6 cm** — un peu court |
| Temps de parcours 4×4 | < 60 s | **≈ 20 s** |
| Dérive odométrique à l'arrivée | ≤ 5 cm | **≈ 6,6 cm** — hors cible |

Les deux derniers points méritent d'être dits franchement.

La **dérive odométrique** est la limite dominante. Elle est essentiellement
longitudinale : le robot arrive quelques centimètres plus loin qu'il ne le croit.
La roue folle est quasi sans frottement (`mu = 0.001`), donc tout le freinage
repose sur les deux roues motrices, et chaque décélération de virage se solde par
un léger glissement vers l'avant que les codeurs ne voient pas. Adoucir la
dynamique angulaire (1,4 → 1,0 rad/s) l'a réduite d'un tiers ; descendre plus bas
n'apporte plus rien. La vraie correction serait une fusion odométrie + IMU (EKF),
délibérément hors périmètre ici.

Cette dérive explique aussi le **jeu résiduel** : le MPC raisonne sur la pose
*crue*, et tient effectivement le robot à ~1,5 cm des murs dans ce repère — mais
c'est le repère qui glisse. Un vérificateur de collision indépendant n'y
changerait rien : il lirait la même odométrie et serait aveugle de la même façon.
C'est pourquoi la surveillance des murs vit *dans* le MPC, évaluée sur toute la
trajectoire prédite, plutôt que dans un garde-fou réactif séparé.

Suivi en direct pendant la course :

```bash
ros2 topic echo /maze_navigator/diagnostics
```

```
v=0.298 w=-0.412 pose=(0.401,0.798,-88.7) s=1.62/3.20
d_wall=0.1934 clearance=0.0164 mpc_ok=1 solve_ms=11.4
```

---

## Fichiers générés automatiquement

À chaque lancement, le générateur produit dans `worlds/` :
- `generated_maze.sdf` — Monde Gazebo (murs, sol, pastilles départ/arrivée)
- `generated_maze.json` — Vérité terrain, lue **uniquement** par le validateur

---

## Dépannage

**Le robot ne bouge pas.** Il attend la carte. Regardez d'abord où en est le drone :

```bash
ros2 topic echo /drone/mapper/status
```

`FRAMING` qui dure : le drone n'arrive pas à cadrer tout le labyrinthe — vérifiez
que `mapping_altitude_max` est suffisant pour sa taille.

**La carte n'arrive pas.** Vérifiez que les trois topics sont bien publiés :

```bash
ros2 topic list | grep /maze/
ros2 topic echo /maze/occupancy_grid --no-arr
```

**La carte est fausse.** Relancez avec `validate_map:=true` : le rapport indique
précisément quels murs sont manqués ou inventés. Le drone journalise aussi la
grille extraite en ASCII à chaque publication.

**Le MPC ne converge pas.** Le nœud bascule automatiquement sur un repli
proportionnel et le signale. Des échecs répétés se lisent dans :

```bash
ros2 topic echo /maze_navigator/diagnostics
```

**Le drone ne décolle pas.**

```bash
ros2 topic echo /model/crazyflie/pose
```

**`ModuleNotFoundError: casadi`** — le MPC a besoin de CasADi dans le même
interpréteur que ROS :

```bash
pip install --user --break-system-packages casadi
```

---

## Licence

MIT © 2026 — yassinechouk
