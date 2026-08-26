#!/usr/bin/env python3
"""
Génère un monde SDF (Gazebo Harmonic) à partir d'une grille de murs bitmask,
exactement dans le même format que celui produit par computer_vision.py /
config.py du projet de vision (N=1, E=2, S=4, W=8 par cellule).

Ce module ne dépend PAS du reste du dépôt de vision : il redéfinit son propre
bitmask + un A* minimal pour rester un package ROS2 autonome, mais le format
de grille est strictement compatible avec `walls` tel que retourné par
`image_to_walls()` dans computer_vision.py. Pour utiliser une vraie grille
détectée par la caméra, il suffit de sérialiser `walls` (numpy array uint8)
en JSON (liste de listes d'entiers) et de la passer avec --walls-json.

Usage:
    python3 maze_world_generator.py --rows 4 --cols 4 --cell-size 0.4 \
        --wall-height 0.15 --wall-thickness 0.02 --seed 1 \
        --out /path/to/maze.sdf
"""
import argparse
import json
import random
import sys
from pathlib import Path

N, E, S, W = 1, 2, 4, 8
OPPOSITE = {N: S, S: N, E: W, W: E}
DELTA = {N: (-1, 0), S: (1, 0), E: (0, 1), W: (0, -1)}


def generate_random_maze(rows, cols, seed=None):
    """
    Génère un labyrinthe parfait (un seul chemin entre 2 cellules quelconques)
    via DFS récursif (randomized backtracker). Retourne une grille bitmask où
    walls[r][c] indique les murs PRÉSENTS (bit à 1 = mur présent).
    """
    rng = random.Random(seed)
    # Au départ, tous les murs sont présents partout.
    walls = [[N | E | S | W for _ in range(cols)] for _ in range(rows)]
    visited = [[False] * cols for _ in range(rows)]

    def carve(r, c):
        visited[r][c] = True
        dirs = [N, E, S, W]
        rng.shuffle(dirs)
        for d in dirs:
            dr, dc = DELTA[d]
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc]:
                walls[r][c] &= ~d
                walls[nr][nc] &= ~OPPOSITE[d]
                carve(nr, nc)

    carve(0, 0)

    # Ouvertures d'entrée/sortie sur le bord extérieur (start en haut-gauche,
    # goal en bas-droite), comme le fait _find_boundary_openings côté vision.
    walls[0][0] &= ~N
    walls[rows - 1][cols - 1] &= ~S
    return walls


def astar(walls, rows, cols, start, goal):
    """A* minimal (Manhattan) pour vérifier la connectivité / produire un chemin de référence."""
    import heapq
    h = lambda p: abs(p[0] - goal[0]) + abs(p[1] - goal[1])
    open_set = [(h(start), 0, start)]
    came_from = {}
    g = {start: 0}
    while open_set:
        _, g_cur, cur = heapq.heappop(open_set)
        if cur == goal:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            return path[::-1]
        r, c = cur
        for d in (N, E, S, W):
            if walls[r][c] & d:
                continue
            dr, dc = DELTA[d]
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            ng = g_cur + 1
            nb = (nr, nc)
            if ng < g.get(nb, float('inf')):
                g[nb] = ng
                came_from[nb] = cur
                heapq.heappush(open_set, (ng + h(nb), ng, nb))
    return []


def cell_center_xy(r, c, rows, cols, cell_size):
    """
    Convertit (row, col) en coordonnées monde (x, y) en mètres, origine au
    centre de la grille, x = vers l'avant (colonnes), y = vers la gauche
    (lignes, inversées pour un repère main droite classique ROS: x-avant,
    y-gauche, z-haut).
    """
    x = c * cell_size
    y = (rows - 1 - r) * cell_size
    return x, y


def wall_box_sdf(name, x, y, z, length, width, height, yaw=0.0):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{length:.4f} {width:.4f} {height:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{length:.4f} {width:.4f} {height:.4f}</size></box></geometry>
          <material>
            <ambient>0.55 0.15 0.15 1</ambient>
            <diffuse>0.75 0.2 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def marker_sdf(name, x, y, z, radius, r, g, b):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>{radius:.4f}</radius><length>0.005</length></cylinder></geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def build_maze_sdf(walls, rows, cols, cell_size, wall_height, wall_thickness,
                    start, goal, world_name="maze_world"):
    """
    Construit chaque segment de mur comme un modèle statique indépendant.
    Convention : on ne dessine que les murs N et W de chaque cellule, plus
    les murs S de la dernière ligne et E de la dernière colonne, pour éviter
    de dupliquer un mur partagé entre 2 cellules (même logique que
    wall_segments() dans path_planning.py).
    """
    models = []
    half = cell_size / 2.0

    for r in range(rows):
        for c in range(cols):
            m = walls[r][c]
            cx, cy = cell_center_xy(r, c, rows, cols, cell_size)

            # Mur Nord (haut, +y côté "N" dans le repère grille -> ici
            # N diminue r, donc le mur nord de la cellule est à cy+half)
            if m & N:
                models.append(wall_box_sdf(
                    f"wall_r{r}_c{c}_N", cx, cy + half, wall_height / 2.0,
                    cell_size + wall_thickness, wall_thickness, wall_height))
            if m & W:
                models.append(wall_box_sdf(
                    f"wall_r{r}_c{c}_W", cx - half, cy, wall_height / 2.0,
                    wall_thickness, cell_size + wall_thickness, wall_height, yaw=0.0))
            if (m & S) and r == rows - 1:
                models.append(wall_box_sdf(
                    f"wall_r{r}_c{c}_S", cx, cy - half, wall_height / 2.0,
                    cell_size + wall_thickness, wall_thickness, wall_height))
            if (m & E) and c == cols - 1:
                models.append(wall_box_sdf(
                    f"wall_r{r}_c{c}_E", cx + half, cy, wall_height / 2.0,
                    wall_thickness, cell_size + wall_thickness, wall_height))

    sx, sy = cell_center_xy(start[0], start[1], rows, cols, cell_size)
    gx, gy = cell_center_xy(goal[0], goal[1], rows, cols, cell_size)
    models.append(marker_sdf("start_marker", sx, sy, 0.003, cell_size * 0.3, 0.1, 0.85, 0.1))
    models.append(marker_sdf("goal_marker", gx, gy, 0.003, cell_size * 0.3, 0.85, 0.1, 0.1))

    floor_w = cols * cell_size + 2 * wall_thickness
    floor_h = rows * cell_size + 2 * wall_thickness
    floor_cx = (cols - 1) * cell_size / 2.0
    floor_cy = (rows - 1) * cell_size / 2.0

    models_xml = "\n".join(models)

    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{world_name}">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{floor_w + 4} {floor_h + 4}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{floor_w + 4} {floor_h + 4}</size></plane></geometry>
          <material>
            <ambient>0.85 0.85 0.85 1</ambient>
            <diffuse>0.9 0.9 0.9 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="maze_floor">
      <static>true</static>
      <pose>{floor_cx:.4f} {floor_cy:.4f} 0.001 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{floor_w:.4f} {floor_h:.4f} 0.002</size></box></geometry>
          <material>
            <ambient>0.95 0.95 0.9 1</ambient>
            <diffuse>0.97 0.97 0.92 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
{models_xml}

  </world>
</sdf>
"""
    return sdf


def main():
    ap = argparse.ArgumentParser(description="Générateur de monde labyrinthe SDF pour Gazebo Harmonic")
    ap.add_argument("--rows", type=int, default=4)
    ap.add_argument("--cols", type=int, default=4)
    ap.add_argument("--cell-size", type=float, default=0.4, help="Taille de cellule en mètres (défaut 0.4m)")
    ap.add_argument("--wall-height", type=float, default=0.15)
    ap.add_argument("--wall-thickness", type=float, default=0.015)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--walls-json", type=str, default=None,
                     help="Chemin vers un JSON contenant la grille bitmask (liste de listes), "
                          "au format walls[r][c] produit par computer_vision.image_to_walls()")
    ap.add_argument("--out", type=str, required=True)
    args = ap.parse_args()

    if args.walls_json:
        with open(args.walls_json) as f:
            data = json.load(f)
        walls = data["walls"] if isinstance(data, dict) and "walls" in data else data
        rows, cols = len(walls), len(walls[0])
        start = tuple(data.get("start", (0, 0))) if isinstance(data, dict) else (0, 0)
        goal = tuple(data.get("goal", (rows - 1, cols - 1))) if isinstance(data, dict) else (rows - 1, cols - 1)
    else:
        rows, cols = args.rows, args.cols
        walls = generate_random_maze(rows, cols, seed=args.seed)
        start, goal = (0, 0), (rows - 1, cols - 1)

    path = astar(walls, rows, cols, start, goal)
    if not path:
        print("ERREUR: labyrinthe non résoluble entre start et goal !", file=sys.stderr)
        sys.exit(1)
    print(f"Labyrinthe {rows}x{cols}, cell_size={args.cell_size}m, "
          f"start={start} goal={goal}, chemin A* = {len(path)} cellules")

    sdf = build_maze_sdf(walls, rows, cols, args.cell_size, args.wall_height,
                          args.wall_thickness, start, goal)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(sdf)
    print(f"Monde SDF écrit dans : {out_path}")

    # Sauvegarde aussi la grille + le chemin de référence à côté, pour que le
    # nœud de navigation (maze_navigator_node.py) puisse le recharger sans
    # dépendre du CV pipeline.
    meta_path = out_path.with_suffix(".json")
    meta = {
        "rows": rows, "cols": cols, "cell_size": args.cell_size,
        "walls": walls, "start": list(start), "goal": list(goal),
        "path": [list(p) for p in path],
    }
    meta_path.write_text(json.dumps(meta, indent=2))
    print(f"Métadonnées (grille + chemin A*) écrites dans : {meta_path}")


if __name__ == "__main__":
    main()
