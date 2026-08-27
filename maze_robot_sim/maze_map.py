#!/usr/bin/env python3
"""
maze_map.py — Format d'échange de la carte entre le drone (cartographe) et le
robot (navigateur), plus les primitives de grille partagées.

Le drone extrait la grille de murs par vision, l'encode en `nav_msgs/OccupancyGrid`
et la publie. Le robot la décode, rejoue un A* et navigue. Ce module est la
*seule* définition du format : les deux nœuds l'importent, donc il ne peut pas
y avoir de divergence d'encodage entre l'émetteur et le récepteur.

Encodage
--------
Chaque cellule du labyrinthe est représentée par un bloc de `SUB_RESOLUTION`
pixels de côté ; les murs occupent les *lignes de bord* partagées entre cellules
voisines. La grille pixel fait donc (cols*SUB+1) x (rows*SUB+1) : le +1 porte le
dernier bord (Est de la dernière colonne, Sud de la dernière ligne).

Conventions d'indices
---------------------
  - Grille labyrinthe : `walls[r][c]`, bitmask N=1, E=2, S=4, W=8 (bit à 1 = mur
    présent). r croît vers le Sud (y décroissant), c croît vers l'Est (x croissant).
  - OccupancyGrid : convention ROS standard, `data[i * width + j]` avec i qui
    croît selon +y et j selon +x, origine au coin (x_min, y_min).

La correspondance entre les deux est portée par `_y_index()` : la ligne
labyrinthe r=0 est celle de plus grand y, donc elle occupe le haut de la grille
ROS (i maximal).
"""
import math

N, E, S, W = 1, 2, 4, 8
OPPOSITE = {N: S, S: N, E: W, W: E}
DELTA = {N: (-1, 0), S: (1, 0), E: (0, 1), W: (0, -1)}

# Nombre de pixels OccupancyGrid par cellule de labyrinthe, sur chaque axe.
# C'est une constante du format de fil : changer cette valeur des deux côtés
# simultanément est sûr, la changer d'un seul côté casse le décodage.
SUB_RESOLUTION = 4

FREE, OCCUPIED = 0, 100

# Seuil de décision au décodage (les valeurs OccupancyGrid sont 0 ou 100).
_OCC_THRESHOLD = 50


def _y_index(r, rows):
    """Indice de ligne ROS (croissant selon +y) pour la ligne labyrinthe `r`."""
    return rows - 1 - r


def cell_center_world(r, c, rows, cols, cell_size, origin_x=0.0, origin_y=0.0):
    """
    Centre monde (x, y) de la cellule (r, c).

    `origin_x`/`origin_y` sont les coordonnées monde du centre de la cellule
    (rows-1, 0) — celle de plus petit x et plus petit y. Avec des origines
    nulles on retombe exactement sur la convention du générateur de monde.
    """
    x = origin_x + c * cell_size
    y = origin_y + _y_index(r, rows) * cell_size
    return x, y


def world_to_cell(x, y, rows, cols, cell_size, origin_x=0.0, origin_y=0.0):
    """Inverse de `cell_center_world`, bornée à la grille."""
    c = int(round((x - origin_x) / cell_size))
    yi = int(round((y - origin_y) / cell_size))
    r = rows - 1 - yi
    r = max(0, min(rows - 1, r))
    c = max(0, min(cols - 1, c))
    return r, c


def astar(walls, rows, cols, start, goal):
    """
    A* Manhattan sur la grille bitmask. Retourne la liste des cellules du chemin
    (start inclus, goal inclus), ou [] si le goal est inatteignable.

    Un mur bloque le déplacement dès qu'il est déclaré d'un côté *ou* de l'autre :
    une grille issue de la vision peut être légèrement asymétrique (mur vu depuis
    une cellule mais raté depuis sa voisine), et traverser un mur réel coûte
    beaucoup plus cher qu'un détour.
    """
    import heapq

    def blocked(r, c, d):
        if walls[r][c] & d:
            return True
        dr, dc = DELTA[d]
        nr, nc = r + dr, c + dc
        if not (0 <= nr < rows and 0 <= nc < cols):
            return True
        return bool(walls[nr][nc] & OPPOSITE[d])

    def h(p):
        return abs(p[0] - goal[0]) + abs(p[1] - goal[1])

    open_set = [(h(start), 0, start)]
    came_from = {}
    g = {start: 0}
    closed = set()
    while open_set:
        _, g_cur, cur = heapq.heappop(open_set)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == goal:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            return path[::-1]
        r, c = cur
        for d in (N, E, S, W):
            if blocked(r, c, d):
                continue
            dr, dc = DELTA[d]
            nb = (r + dr, c + dc)
            ng = g_cur + 1
            if ng < g.get(nb, float("inf")):
                g[nb] = ng
                came_from[nb] = cur
                heapq.heappush(open_set, (ng + h(nb), ng, nb))
    return []


def walls_to_grid_data(walls, rows, cols):
    """
    Sérialise la grille bitmask en (data, width, height) au format OccupancyGrid.

    `data` est une liste d'int8 (0 = libre, 100 = mur).
    """
    sub = SUB_RESOLUTION
    width = cols * sub + 1
    height = rows * sub + 1
    data = [FREE] * (width * height)

    def fill(i, j):
        data[i * width + j] = OCCUPIED

    for r in range(rows):
        yi = _y_index(r, rows)
        i0 = yi * sub          # bord Sud de la cellule
        i1 = (yi + 1) * sub    # bord Nord
        for c in range(cols):
            m = walls[r][c]
            j0 = c * sub       # bord Ouest
            j1 = (c + 1) * sub  # bord Est
            if m & N:
                for k in range(sub + 1):
                    fill(i1, j0 + k)
            if m & S:
                for k in range(sub + 1):
                    fill(i0, j0 + k)
            if m & W:
                for k in range(sub + 1):
                    fill(i0 + k, j0)
            if m & E:
                for k in range(sub + 1):
                    fill(i0 + k, j1)
    return data, width, height


def grid_data_to_walls(data, width, height):
    """
    Désérialise (data, width, height) vers (walls, rows, cols).

    La présence d'un mur est lue au *milieu* de l'arête plutôt qu'à ses
    extrémités : les coins sont partagés par jusqu'à quatre murs et sont donc
    ambigus, le milieu ne l'est pas.
    """
    sub = SUB_RESOLUTION
    cols = (width - 1) // sub
    rows = (height - 1) // sub
    half = sub // 2
    walls = [[0] * cols for _ in range(rows)]

    def occ(i, j):
        return data[i * width + j] > _OCC_THRESHOLD

    for r in range(rows):
        yi = _y_index(r, rows)
        i0 = yi * sub
        i1 = (yi + 1) * sub
        for c in range(cols):
            j0 = c * sub
            j1 = (c + 1) * sub
            m = 0
            if occ(i1, j0 + half):
                m |= N
            if occ(i0, j0 + half):
                m |= S
            if occ(i0 + half, j0):
                m |= W
            if occ(i0 + half, j1):
                m |= E
            walls[r][c] = m
    return walls, rows, cols


def grid_origin(cell_size, origin_x=0.0, origin_y=0.0):
    """
    Coin (x_min, y_min) de la grille OccupancyGrid, à partir du centre monde de
    la cellule (rows-1, 0). C'est ce qui va dans `info.origin.position`.
    """
    return origin_x - cell_size / 2.0, origin_y - cell_size / 2.0


def grid_origin_to_cell_origin(origin_x, origin_y, cell_size):
    """Inverse de `grid_origin`."""
    return origin_x + cell_size / 2.0, origin_y + cell_size / 2.0


def wall_segments(walls, rows, cols, cell_size, origin_x=0.0, origin_y=0.0):
    """
    Convertit la grille en segments de murs monde [(ax, ay, bx, by), ...].

    Un mur partagé entre deux cellules n'est émis qu'une fois (on ne dessine que
    les murs N et W de chaque cellule, plus les S de la dernière ligne et les E
    de la dernière colonne) — même convention que le générateur SDF, pour que le
    contrôleur voie exactement la géométrie que Gazebo simule.
    """
    half = cell_size / 2.0
    segs = []
    for r in range(rows):
        for c in range(cols):
            m = walls[r][c]
            cx, cy = cell_center_world(r, c, rows, cols, cell_size, origin_x, origin_y)
            if m & N:
                segs.append((cx - half, cy + half, cx + half, cy + half))
            if m & W:
                segs.append((cx - half, cy - half, cx - half, cy + half))
            if (m & S) and r == rows - 1:
                segs.append((cx - half, cy - half, cx + half, cy - half))
            if (m & E) and c == cols - 1:
                segs.append((cx + half, cy - half, cx + half, cy + half))
    return segs


def point_segment_distance(px, py, ax, ay, bx, by):
    """Distance euclidienne d'un point à un segment (version numérique scalaire)."""
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    denom = vx * vx + vy * vy
    t = 0.0 if denom < 1e-12 else max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
    dx = px - (ax + t * vx)
    dy = py - (ay + t * vy)
    return math.hypot(dx, dy)
