#!/usr/bin/env python3
"""
maze_vision.py — Extraction du labyrinthe depuis une image caméra nadir.

Entrée  : une image BGR prise par la caméra du drone pointée vers le bas, plus
          la pose monde de la caméra (x, y, altitude) et sa focale en pixels.
Sortie  : la grille de murs bitmask, la taille de cellule, l'origine monde de la
          grille, et les cellules start / goal — le tout *sans* connaître à
          l'avance les dimensions du labyrinthe.

Géométrie
---------
La caméra du Crazyflie a pour pose `rpy = (0, π/2, π)` dans le repère du drone,
ce qui l'oriente vers -Z. En déroulant la convention optique de Gazebo (X avant,
Y gauche, Z haut pour le lien ; Z avant, X droite, Y bas pour l'optique), on
obtient une correspondance image → monde purement affine tant que le drone est à
plat :

    u (colonne image, vers la droite)  →  +Y monde
    v (ligne image, vers le bas)       →  +X monde

Un point du monde à l'altitude z se projette donc en

    u = u0 + (Y - cam_y) * f / (alt - z)
    v = v0 + (X - cam_x) * f / (alt - z)

Parallaxe des murs
------------------
Les murs ont une hauteur non nulle : vus d'en haut, leur base et leur sommet ne
se projettent pas au même endroit, et un mur apparaît comme une bande étalée
*vers l'extérieur* de l'image (la base est toujours le bord le plus proche du
point principal). On localise donc chaque ligne de la grille sur le bord de sa
bande le plus proche du centre, ce qui la ramène exactement au plan du sol z=0
et rend l'échelle `alt / f` exacte au lieu d'approximative. C'est ce qui permet
d'obtenir une taille de cellule au centimètre près, indispensable vu la marge de
2,3 cm entre le robot et les murs.
"""
import math

import cv2
import numpy as np

from . import maze_map

# Bornes de recherche pour la détection automatique des dimensions.
MIN_CELLS, MAX_CELLS = 1, 16

# Un candidat (rows, cols) n'est retenu que si les lignes détectées tombent sur
# la grille régulière prédite et que la grille est suffisamment couverte.
_MAX_ALIGN_ERR = 0.05   # en fraction d'un intervalle de grille
_MIN_COVERAGE  = 0.60   # fraction des lignes prédites effectivement observées

# Hauteurs physiques (m) des éléments observés, utilisées pour la parallaxe.
DEFAULT_WALL_HEIGHT   = 0.15
DEFAULT_MARKER_HEIGHT = 0.086   # plan du marqueur ArUco sur le toit du robot
DEFAULT_FLOOR_HEIGHT  = 0.003   # pastilles start/goal peintes au sol

def _aruco_params():
    """
    Réglages du détecteur ArUco, ajustés pour une vue nadir lointaine.

    Deux écarts au défaut seulement : un seuil de périmètre minimal abaissé, car
    le marqueur de 12 cm ne fait plus que quelques dizaines de pixels quand le
    drone prend de la hauteur, et un raffinement sous-pixellique des coins, qui
    fait de la position du marqueur une mesure et plus seulement une détection —
    c'est elle qui sert à situer le robot dans le repère monde.
    """
    p = cv2.aruco.DetectorParameters()
    p.minMarkerPerimeterRate = 0.02
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return p


_ARUCO_DICT     = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_ARUCO_PARAMS   = _aruco_params()
_ARUCO_DETECTOR = cv2.aruco.ArucoDetector(_ARUCO_DICT, _ARUCO_PARAMS)


class MazeVisionError(Exception):
    """L'extraction a échoué ; l'appelant doit réessayer sur une autre image."""


class MazeObservation:
    """Résultat d'une extraction réussie."""

    def __init__(self, walls, rows, cols, cell_size, origin_x, origin_y,
                 start_cell, goal_cell, diagnostics):
        self.walls      = walls
        self.rows       = rows
        self.cols       = cols
        self.cell_size  = cell_size
        self.origin_x   = origin_x
        self.origin_y   = origin_y
        self.start_cell = start_cell
        self.goal_cell  = goal_cell
        self.diagnostics = diagnostics

    def __repr__(self):
        return (f"MazeObservation({self.rows}x{self.cols}, "
                f"cell={self.cell_size:.4f}m, "
                f"origin=({self.origin_x:.3f}, {self.origin_y:.3f}), "
                f"start={self.start_cell}, goal={self.goal_cell})")


def focal_px_from_hfov(width_px, hfov_rad):
    """Focale en pixels d'une caméra pinhole à partir de son champ horizontal."""
    return (width_px / 2.0) / math.tan(hfov_rad / 2.0)


# ──────────────────────────────────────────────────────────────────────────
#  Segmentation couleur
# ──────────────────────────────────────────────────────────────────────────

def red_mask(bgr):
    """
    Masque binaire des éléments rouges : murs du labyrinthe *et* pastille goal.

    Le rouge est à cheval sur la discontinuité de teinte HSV, d'où les deux
    plages. Le sol du labyrinthe (quasi blanc) et le plan de sol gris ont une
    saturation faible et sont donc naturellement exclus.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (0, 60, 40), (12, 255, 255))
    m |= cv2.inRange(hsv, (168, 60, 40), (180, 255, 255))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel, iterations=1)


def green_mask(bgr):
    """Masque binaire de la pastille verte de départ."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (40, 60, 40), (85, 255, 255))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel, iterations=1)


def find_disc(mask, cell_px=None):
    """
    Localise une pastille circulaire dans un masque, ou retourne (None, masque vide).

    La pastille d'arrivée est rouge comme les murs : c'est sa *forme* qui les
    sépare. Vue du dessus une pastille est une composante connexe compacte —
    boîte englobante quasi carrée et bien remplie — alors qu'un mur est un trait
    allongé et que l'ensemble des murs forme une composante ajourée. Ce critère
    ne dépend d'aucune échelle, ce qui permet de l'appliquer avant même de
    connaître la taille des cellules.

    Quand la taille de cellule est connue (deuxième passe), on exige en plus un
    diamètre cohérent, ce qui écarte définitivement toute ambiguïté.
    """
    n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    best, best_area = None, 0
    for i in range(1, n):
        x, y, w, h, area = (stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP],
                            stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT],
                            stats[i, cv2.CC_STAT_AREA])
        if area < 20 or w == 0 or h == 0:
            continue
        if min(w, h) / max(w, h) < 0.7:      # trait allongé → mur
            continue
        if area / float(w * h) < 0.6:        # boîte ajourée → structure de murs
            continue
        if cell_px is not None and not (0.25 * cell_px < max(w, h) < 1.05 * cell_px):
            continue
        if area > best_area:
            best, best_area = i, area
    if best is None:
        return None, np.zeros_like(mask)
    blob = np.where(labels == best, np.uint8(255), np.uint8(0))
    return (float(centroids[best][0]), float(centroids[best][1])), blob


def _line_masks(mask, line_len_px):
    """
    Sépare le masque en traits horizontaux et verticaux (au sens image).

    Une ouverture par un élément structurant linéaire ne conserve que les traits
    au moins aussi longs que lui dans cette direction : les murs perpendiculaires,
    larges de quelques pixels seulement, disparaissent.
    """
    L = max(5, int(line_len_px) | 1)
    kh = cv2.getStructuringElement(cv2.MORPH_RECT, (L, 1))
    kv = cv2.getStructuringElement(cv2.MORPH_RECT, (1, L))
    horiz = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kh)   # traits selon u → X constant
    vert  = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kv)   # traits selon v → Y constant
    return horiz, vert


# ──────────────────────────────────────────────────────────────────────────
#  Détection des lignes de grille
# ──────────────────────────────────────────────────────────────────────────

def _runs_above(profile, threshold, min_gap):
    """Plages contiguës (start, end) où le profil dépasse le seuil."""
    idx = np.flatnonzero(profile >= threshold)
    if idx.size == 0:
        return []
    runs = []
    start = prev = idx[0]
    for i in idx[1:]:
        if i - prev > min_gap:
            runs.append((start, prev))
            start = i
        prev = i
    runs.append((start, prev))
    return runs


def _run_edges(profile, run):
    """
    Bords sous-pixelliques d'une plage du profil, lus à mi-hauteur de son maximum.

    L'interpolation linéaire entre les deux échantillons qui encadrent ce niveau
    évite de quantifier la position d'une ligne au pixel entier, ce qui coûterait
    déjà plusieurs millimètres au sol.
    """
    a, b = run
    level = 0.5 * profile[a:b + 1].max()

    def cross(i, direction):
        j = i
        while 0 <= j + direction < len(profile) and profile[j] < level:
            j += direction
        lo, hi = (j - direction, j) if direction > 0 else (j, j - direction)
        lo = max(0, min(len(profile) - 1, lo))
        hi = max(0, min(len(profile) - 1, hi))
        if lo == hi or profile[hi] == profile[lo]:
            return float(j)
        t = (level - profile[lo]) / (profile[hi] - profile[lo])
        return float(lo) + max(0.0, min(1.0, t))

    return cross(a, +1), cross(b, -1)


def _wall_baseline(profile, run, principal, alt, wall_height):
    """
    Position image de l'axe d'un mur, ramenée au plan du sol.

    Un mur d'épaisseur t et de hauteur h, à la distance radiale R du nadir,
    occupe dans le profil la plage qui va de la base de son flanc intérieur à la
    crête de son flanc extérieur :

        d_proche = (R - t/2) · f / alt        d_loin = (R + t/2) · f / (alt - h)

    Deux mesures pour deux inconnues : on en tire R·f = (alt·d_proche
    + (alt-h)·d_loin) / 2 sans jamais avoir à connaître l'épaisseur des murs.
    Se contenter du bord intérieur laisserait au contraire un biais de t/2 vers
    le centre sur chaque ligne, donc un labyrinthe reconstruit trop petit — et
    quelques millimètres suffisent à consommer la marge robot/mur.
    """
    e0, e1 = _run_edges(profile, run)
    s0, s1 = e0 - principal, e1 - principal
    sign = 1.0 if (s0 + s1) >= 0 else -1.0
    d_near, d_far = min(abs(s0), abs(s1)), max(abs(s0), abs(s1))
    rho = (alt * d_near + max(alt - wall_height, 1e-3) * d_far) / 2.0
    return principal + sign * rho / alt


def _detect_lines(line_mask, axis, principal, alt, wall_height):
    """
    Positions des lignes de grille le long d'un axe image, au niveau du sol.

    `axis=0` projette sur v (lignes de X constant), `axis=1` projette sur u
    (lignes de Y constant).
    """
    profile = line_mask.astype(np.float64).sum(axis=1 - axis) / 255.0
    if profile.max() <= 0:
        return []
    # Seuil bas et relatif : une ligne intérieure peut ne porter qu'une seule
    # cellule de mur alors que le bord extérieur en porte `rows` ou `cols`.
    threshold = max(4.0, 0.05 * profile.max())
    runs = _runs_above(profile, threshold, min_gap=2)
    return [_wall_baseline(profile, r, principal, alt, wall_height) for r in runs]


def _score_count(lines, n):
    """
    Qualité d'un découpage des lignes observées en `n` intervalles réguliers.

    Retourne (erreur d'alignement normalisée, couverture, pas). L'erreur mesure
    à quel point les lignes vues tombent sur la grille prédite, la couverture
    combien de lignes prédites ont effectivement été vues.
    """
    lo, hi = min(lines), max(lines)
    step = (hi - lo) / n
    if step <= 1e-6:
        return 1.0, 0.0, 0.0
    predicted = [lo + k * step for k in range(n + 1)]
    err = sum(min(abs(x - p) for p in predicted) for x in lines) / (len(lines) * step)
    seen = sum(1 for p in predicted if min(abs(x - p) for x in lines) < 0.25 * step)
    return err, seen / (n + 1), step


def _candidate_counts(lines):
    """Découpages plausibles d'un jeu de lignes, du plus fin au plus grossier."""
    if len(lines) < 2:
        return []
    span = max(lines) - min(lines)
    out = []
    for n in range(MIN_CELLS, MAX_CELLS + 1):
        err, cov, step = _score_count(lines, n)
        if step < 4.0:          # une cellule de moins de 4 px n'est pas exploitable
            continue
        if err <= _MAX_ALIGN_ERR and cov >= _MIN_COVERAGE:
            out.append((n, err, cov, step))
    if not out and span > 0:
        return []
    return out


def infer_grid_shape(lines_v, lines_u):
    """
    Choisit conjointement (cols, rows) à partir des lignes détectées.

    Le critère décisif est que les cellules sont *carrées* : une erreur de
    facteur 2 sur un seul axe est immédiatement trahie par le rapport des pas.
    C'est ce couplage qui rend la détection automatique fiable là où compter les
    lignes axe par axe serait ambigu.
    """
    cand_v = _candidate_counts(lines_v)
    cand_u = _candidate_counts(lines_u)
    if not cand_v or not cand_u:
        raise MazeVisionError(
            f"aucun découpage régulier plausible "
            f"({len(lines_v)} lignes en v, {len(lines_u)} lignes en u)")

    best, best_score = None, -1e9
    for nv, ev, cv_, sv in cand_v:
        for nu, eu, cu_, su in cand_u:
            squareness = abs(sv - su) / max(sv, su)
            if squareness > 0.12:
                continue
            # L'alignement décide ; à alignement égal on préfère la grille la
            # plus fine : un découpage deux fois trop grossier explique aussi
            # bien les lignes vues tout en fusionnant des cellules réelles.
            score = -8.0 * (ev + eu) - 6.0 * squareness + 0.05 * (nv + nu)
            if score > best_score:
                best_score, best = score, (nv, nu, sv, su)
    if best is None:
        raise MazeVisionError("pas de couple (rows, cols) à cellules carrées")
    cols, rows, step_v, step_u = best
    return rows, cols, step_v, step_u


def _regular_lines(lines, n):
    """Grille régulière de n+1 lignes ajustée aux lignes observées (moindres carrés)."""
    lo, hi = min(lines), max(lines)
    step = (hi - lo) / n
    # Réajuste lo et le pas sur toutes les lignes observées plutôt que sur les
    # deux extrêmes seules, pour ne pas laisser une ligne de bord bruitée
    # décaler toute la grille.
    ks, xs = [], []
    for x in lines:
        k = round((x - lo) / step)
        if abs(x - (lo + k * step)) < 0.3 * step:
            ks.append(k)
            xs.append(x)
    if len(ks) >= 2:
        A = np.vstack([np.array(ks, dtype=float), np.ones(len(ks))]).T
        step, lo = np.linalg.lstsq(A, np.array(xs, dtype=float), rcond=None)[0]
    return [lo + k * step for k in range(n + 1)], step


# ──────────────────────────────────────────────────────────────────────────
#  Lecture des murs cellule par cellule
# ──────────────────────────────────────────────────────────────────────────

def _edge_present(line_mask, fixed_pos, span_lo, span_hi, axis, principal,
                  band, threshold):
    """
    Teste la présence d'un mur sur une arête de cellule.

    On prend, sur une bande autour de la ligne théorique, la meilleure couverture
    le long de l'arête. Prendre le maximum plutôt que la moyenne rend le test
    insensible à la largeur exacte de la bande : là où le mur est réellement
    projeté, la couverture vaut ~1, ailleurs ~0.

    La bande est étendue *vers l'extérieur* de l'image pour absorber la
    parallaxe, qui ne déplace jamais un mur vers le centre.
    """
    h, w = line_mask.shape
    outward = 1 if fixed_pos >= principal else -1
    p = int(round(fixed_pos))
    lo = p + min(0, outward * band) - 2
    hi = p + max(0, outward * band) + 2

    # On n'échantillonne que la moitié centrale de l'arête : ses extrémités
    # touchent les murs perpendiculaires et les coins, qui fausseraient le test.
    inset = 0.25 * (span_hi - span_lo)
    s0 = int(round(span_lo + inset))
    s1 = int(round(span_hi - inset))
    if s1 - s0 < 2:
        return False

    best = 0.0
    for q in range(lo, hi + 1):
        if axis == 0:                       # ligne à v fixé, s'étendant selon u
            if not (0 <= q < h):
                continue
            strip = line_mask[q, max(0, s0):min(w, s1)]
        else:                               # ligne à u fixé, s'étendant selon v
            if not (0 <= q < w):
                continue
            strip = line_mask[max(0, s0):min(h, s1), q]
        if strip.size:
            best = max(best, float(np.count_nonzero(strip)) / strip.size)
    return best >= threshold


def _read_walls(mask_h, mask_v, lines_v, lines_u, rows, cols,
                principal_v, principal_u, band, threshold):
    """
    Remplit la grille bitmask à partir des deux masques de traits.

    Rappel des correspondances : X croît avec v, Y croît avec u, et la ligne
    labyrinthe r=0 est celle de plus grand Y donc de plus grand u.
    """
    walls = [[0] * cols for _ in range(rows)]
    for r in range(rows):
        yi = rows - 1 - r
        u_s, u_n = lines_u[yi], lines_u[yi + 1]
        for c in range(cols):
            v_w, v_e = lines_v[c], lines_v[c + 1]
            m = 0
            # Murs N/S : Y constant → traits verticaux dans l'image (mask_v),
            # repérés par leur position en u, étendus selon v.
            if _edge_present(mask_v, u_n, v_w, v_e, 1, principal_u, band, threshold):
                m |= maze_map.N
            if _edge_present(mask_v, u_s, v_w, v_e, 1, principal_u, band, threshold):
                m |= maze_map.S
            # Murs E/W : X constant → traits horizontaux (mask_h), repérés en v.
            if _edge_present(mask_h, v_e, u_s, u_n, 0, principal_v, band, threshold):
                m |= maze_map.E
            if _edge_present(mask_h, v_w, u_s, u_n, 0, principal_v, band, threshold):
                m |= maze_map.W
            walls[r][c] = m
    return walls


# ──────────────────────────────────────────────────────────────────────────
#  Pipeline complète
# ──────────────────────────────────────────────────────────────────────────

def image_to_world(u, v, cam_x, cam_y, alt, focal_px, z=0.0):
    """
    Projette un pixel sur le plan horizontal d'altitude `z`.

    `cam_x`/`cam_y` désignent le point du sol visé par l'axe optique — le nadir
    apparent — et non la position du drone : voir `nadir_ground_point`.
    """
    s = (alt - z) / focal_px
    return cam_x + v * s, cam_y + u * s


def nadir_ground_point(x, y, alt, roll, pitch):
    """
    Point du sol visé par l'axe optique, compte tenu de l'assiette du drone.

    Un multirotor en vol stationnaire n'est jamais parfaitement à plat : un degré
    ou deux d'assiette résiduelle déplacent déjà le point visé de plusieurs
    centimètres au sol à trois mètres d'altitude. Le modèle affine image ↔ monde
    reste valable — l'échelle et la forme de la grille sont inchangées au premier
    ordre — mais la carte se retrouve translatée dans le repère monde.

    La caméra regarde selon -Z du drone ; un tangage positif fait glisser le point
    visé vers -X, un roulis positif vers +Y.
    """
    return x - alt * math.tan(pitch), y + alt * math.tan(roll)


def maze_bounds_px(frame):
    """
    Boîte englobante du labyrinthe en pixels, ou None s'il n'est pas visible.

    Sert au cadrage : tant que la boîte touche un bord de l'image, le drone ne
    voit pas tout le labyrinthe et doit monter ou se recentrer.
    """
    mask = red_mask(frame)
    ys, xs = np.nonzero(mask)
    if xs.size < 50:
        return None
    return (float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max()))


def _line_kernel_length(alt, focal_px, radius_px, wall_height, cell_px=None):
    """
    Longueur du noyau d'ouverture qui sépare les traits horizontaux des verticaux.

    Elle doit dépasser la largeur *apparente* d'un mur — son épaisseur réelle
    plus son étalement par parallaxe, qui croît avec l'excentricité — sinon un
    mur perpendiculaire assez épais survit à l'ouverture et pollue l'autre axe.
    Elle doit rester en deçà de la longueur d'un mur, c'est-à-dire d'une cellule.
    """
    thickness_px = 0.04 * focal_px / alt          # majorant large de l'épaisseur
    parallax_px = radius_px * wall_height / max(alt - wall_height, 1e-3)
    L = 1.5 * (thickness_px + parallax_px)
    if cell_px is not None:
        L = min(L, 0.5 * cell_px)
    return int(max(9, round(L)))


def extract_maze(frame, cam_x, cam_y, alt, focal_px,
                 wall_height=DEFAULT_WALL_HEIGHT,
                 marker_height=DEFAULT_MARKER_HEIGHT,
                 floor_height=DEFAULT_FLOOR_HEIGHT,
                 wall_threshold=0.45,
                 start_world=None):
    """
    Extrait le labyrinthe complet d'une image nadir.

    L'analyse se fait en deux passes : la première découvre la taille des
    cellules, la seconde rejoue la segmentation avec des noyaux morphologiques
    calibrés sur cette taille. Sans cela, la longueur des noyaux devrait être
    devinée, et une erreur de réglage se traduit directement par un mur manqué.

    Lève `MazeVisionError` si l'image ne permet pas une extraction fiable ; c'est
    un cas normal (drone encore en mouvement, cadrage incomplet) que l'appelant
    traite en réessayant sur l'image suivante.

    `start_world` permet de fournir la position monde du robot relevée
    auparavant. C'est utile parce que le marqueur ArUco, large de 12 cm, devient
    illisible bien avant les murs quand le drone monte : sur un grand labyrinthe
    l'altitude qui cadre toute la scène est déjà trop haute pour le lire. Comme
    le drone connaît sa propre pose, une lecture faite pendant la montée reste
    valide tant que le robot n'a pas bougé — ce qui est le cas, il attend la carte.
    """
    if frame is None or frame.ndim != 3:
        raise MazeVisionError("image absente ou non couleur")
    if alt <= wall_height + 0.2:
        raise MazeVisionError(f"altitude trop basse ({alt:.2f} m)")

    h, w = frame.shape[:2]
    principal_u = (w - 1) / 2.0
    principal_v = (h - 1) / 2.0
    scale_floor = alt / focal_px                      # m/px au niveau du sol

    mask = red_mask(frame)
    if np.count_nonzero(mask) < 200:
        raise MazeVisionError("aucun mur rouge détecté")

    ys, xs = np.nonzero(mask)
    radius_px = max(abs(xs.min() - principal_u), abs(xs.max() - principal_u),
                    abs(ys.min() - principal_v), abs(ys.max() - principal_v))

    cell_px_hint = None
    result = None
    for _ in range(2):
        goal_px, goal_blob = find_disc(mask, cell_px_hint)
        wall_mask = cv2.subtract(mask, cv2.dilate(
            goal_blob, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))))

        line_len = _line_kernel_length(alt, focal_px, radius_px, wall_height, cell_px_hint)
        mask_h, mask_v = _line_masks(wall_mask, line_len)

        lines_v = _detect_lines(mask_h, 0, principal_v, alt, wall_height)
        lines_u = _detect_lines(mask_v, 1, principal_u, alt, wall_height)
        if len(lines_v) < 2 or len(lines_u) < 2:
            raise MazeVisionError(
                f"lignes de grille insuffisantes (v={len(lines_v)}, u={len(lines_u)})")

        rows, cols, _, _ = infer_grid_shape(lines_v, lines_u)
        grid_v, step_v = _regular_lines(lines_v, cols)
        grid_u, step_u = _regular_lines(lines_u, rows)

        result = (rows, cols, grid_v, grid_u, step_v, step_u,
                  mask_h, mask_v, goal_px, line_len)
        new_hint = 0.5 * (step_v + step_u)
        if cell_px_hint is not None and abs(new_hint - cell_px_hint) < 0.02 * new_hint:
            break
        cell_px_hint = new_hint

    (rows, cols, grid_v, grid_u, step_v, step_u,
     mask_h, mask_v, goal_px, line_len) = result

    # Étalement maximal d'un mur par parallaxe, sur le pixel le plus excentré.
    max_radius = max(abs(grid_v[0] - principal_v), abs(grid_v[-1] - principal_v),
                     abs(grid_u[0] - principal_u), abs(grid_u[-1] - principal_u))
    parallax = max_radius * wall_height / max(alt - wall_height, 1e-3)
    band = int(min(max(3.0, math.ceil(parallax)), 0.25 * min(step_v, step_u)))

    walls = _read_walls(mask_h, mask_v, grid_v, grid_u, rows, cols,
                        principal_v, principal_u, band, wall_threshold)

    cell_size = 0.5 * (step_v + step_u) * scale_floor

    # Origine = centre monde de la cellule (rows-1, 0), la plus proche de
    # l'origine du repère grille (x et y minimaux).
    origin_v = 0.5 * (grid_v[0] + grid_v[1])
    origin_u = 0.5 * (grid_u[0] + grid_u[1])
    origin_x, origin_y = image_to_world(origin_u - principal_u, origin_v - principal_v,
                                        cam_x, cam_y, alt, focal_px, z=0.0)

    def px_to_cell(px, z):
        u_c, v_c = px[0] - principal_u, px[1] - principal_v
        wx, wy = image_to_world(u_c, v_c, cam_x, cam_y, alt, focal_px, z=z)
        return maze_map.world_to_cell(wx, wy, rows, cols, cell_size, origin_x, origin_y)

    goal_cell = px_to_cell(goal_px, floor_height) if goal_px else None

    if start_world is not None:
        start_cell = maze_map.world_to_cell(start_world[0], start_world[1],
                                            rows, cols, cell_size, origin_x, origin_y)
        start_px = None
    else:
        start_px = _detect_aruco_center(frame)
        start_z = marker_height
        if start_px is None:
            # Repli : la pastille de départ verte, visible tant que le robot ne
            # la masque pas complètement.
            start_px, _ = find_disc(green_mask(frame), 0.5 * (step_v + step_u))
            start_z = floor_height
        start_cell = px_to_cell(start_px, start_z) if start_px else None

    diagnostics = {
        "step_v_px": step_v, "step_u_px": step_u,
        "scale_m_per_px": scale_floor,
        "parallax_px": parallax, "band_px": band,
        "line_len_px": line_len,
        "goal_px": goal_px, "start_px": start_px,
    }
    if start_cell is None:
        raise MazeVisionError("cellule de départ introuvable (ni ArUco ni pastille verte)")
    if goal_cell is None:
        raise MazeVisionError("pastille d'arrivée introuvable")

    return MazeObservation(walls, rows, cols, cell_size, origin_x, origin_y,
                           start_cell, goal_cell, diagnostics)


def locate_robot_world(frame, cam_x, cam_y, alt, focal_px,
                       marker_height=DEFAULT_MARKER_HEIGHT):
    """
    Position monde (x, y) du marqueur ArUco du robot, ou None s'il n'est pas lu.

    À appeler dès que possible pendant la montée : c'est à basse altitude que le
    marqueur est le plus lisible, et le robot ne bouge pas avant d'avoir la carte.
    """
    px = _detect_aruco_center(frame)
    if px is None:
        return None
    h, w = frame.shape[:2]
    return image_to_world(px[0] - (w - 1) / 2.0, px[1] - (h - 1) / 2.0,
                          cam_x, cam_y, alt, focal_px, z=marker_height)


def _detect_aruco_center(frame):
    """Centre image du marqueur ArUco ID 0, ou None."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = _ARUCO_DETECTOR.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return None
    for i, marker_id in enumerate(ids.flatten()):
        if marker_id == 0:
            pts = corners[i][0]
            return float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1]))
    return None


def vote_observations(observations, min_agreement=0.6):
    """
    Fusionne plusieurs extractions en une carte consensuelle.

    Un mur n'est retenu que s'il a été vu sur une fraction suffisante des images
    qui s'accordent sur les dimensions. Une extraction isolée peut rater un mur
    (reflet, occultation par le robot) ; le vote élimine ces accidents, et rater
    un mur est bien plus dangereux qu'en inventer un.
    """
    if not observations:
        raise MazeVisionError("aucune observation à fusionner")

    shapes = {}
    for obs in observations:
        shapes.setdefault((obs.rows, obs.cols), []).append(obs)
    (rows, cols), group = max(shapes.items(), key=lambda kv: len(kv[1]))
    if len(group) < max(1, len(observations) // 2):
        raise MazeVisionError(
            f"dimensions instables entre images : {sorted(shapes)}")

    n = len(group)
    walls = [[0] * cols for _ in range(rows)]
    for r in range(rows):
        for c in range(cols):
            for d in (maze_map.N, maze_map.E, maze_map.S, maze_map.W):
                votes = sum(1 for o in group if o.walls[r][c] & d)
                if votes / n >= min_agreement:
                    walls[r][c] |= d

    def median(vals):
        return float(np.median(np.asarray(vals, dtype=float)))

    def majority(vals):
        vals = [v for v in vals if v is not None]
        if not vals:
            return None
        return max(set(vals), key=vals.count)

    merged = MazeObservation(
        walls, rows, cols,
        median([o.cell_size for o in group]),
        median([o.origin_x for o in group]),
        median([o.origin_y for o in group]),
        majority([o.start_cell for o in group]),
        majority([o.goal_cell for o in group]),
        {"votes": n, "total": len(observations)},
    )
    if merged.start_cell is None or merged.goal_cell is None:
        raise MazeVisionError("start ou goal absent du consensus")
    return merged
