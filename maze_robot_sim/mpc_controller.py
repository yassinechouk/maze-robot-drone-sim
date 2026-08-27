#!/usr/bin/env python3
"""
mpc_controller.py — Commande prédictive (MPC) du robot différentiel dans le
labyrinthe, formulée avec CasADi et résolue par IPOPT.

Le problème résolu à chaque pas :

    min  Σ_k  ‖x_k − x_ref,k‖²_Q + ‖u_k‖²_R + ‖u_k − u_{k−1}‖²_R∆
              + w_align · (v_k · sin(θ_k − θ_ref,k))²
              + w_mur   · Σ_murs  max(0, marge − d(p_k, mur))²
    s.c. x_{k+1} = f(x_k, u_k)         modèle unicycle
         0 ≤ v ≤ v_max,  |ω| ≤ ω_max
         |∆v| ≤ a_max·dt,  |∆ω| ≤ α_max·dt

Deux choix méritent d'être explicités.

**Le terme d'alignement** `(v · sin(θ − θ_ref))²` pénalise le fait d'avancer vite
en étant mal orienté. C'est la version continue et optimisable du facteur
`cos(erreur)^8` de l'ancien contrôleur : là où celui-ci réagissait à l'erreur
courante, celui-ci la voit venir sur tout l'horizon et freine *avant* le virage.

**Les murs sont des pénalités douces, pas des contraintes dures.** Une contrainte
dure `d ≥ marge` rend le problème infaisable dès que le robot dérive à l'intérieur
de la marge — et IPOPT renvoie alors une erreur au lieu d'une commande, exactement
au moment où l'on en a le plus besoin. La pénalité douce reste toujours faisable
et se contente de rendre la violation très coûteuse. Aucune formulation ne
garantit zéro collision si l'odométrie dérive ; celle-ci dégrade proprement.

Géométrie : le robot est modélisé par son disque circonscrit (rayon 0,177 m pour
un châssis de 0,25 m), seul modèle correct pour un différentiel qui pivote sur
place. Dans un couloir de 0,40 m il ne reste que 2,3 cm de jeu de chaque côté,
d'où des pénalités de mur volontairement raides.
"""
import math

import casadi as ca
import numpy as np

# Segment factice, placé assez loin pour que sa pénalité soit nulle, utilisé
# pour compléter la liste de murs quand le robot en voit moins que prévu.
_FAR_AWAY = 1.0e4


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class PathTracker:
    """
    Génère la trajectoire de référence à suivre sur l'horizon du MPC.

    Le chemin A* est une polyligne passant par les centres de cellules. À chaque
    cycle on projette le robot dessus, puis on échantillonne des points espacés
    de `v_nom · dt` le long de l'abscisse curviligne. Le cap de référence est la
    tangente locale de la polyligne ; elle est discontinue dans les virages, ce
    qui est justement le signal que le MPC utilise pour freiner en amont.
    """

    def __init__(self, waypoints):
        self.pts = [np.array(p, dtype=float) for p in waypoints]
        self.seg_len = [float(np.linalg.norm(self.pts[i + 1] - self.pts[i]))
                        for i in range(len(self.pts) - 1)]
        self.cum = [0.0]
        for L in self.seg_len:
            self.cum.append(self.cum[-1] + L)
        self.total = self.cum[-1]
        self._s = 0.0

    def _point_at(self, s):
        """Point et tangente de la polyligne à l'abscisse curviligne `s`."""
        s = max(0.0, min(self.total, s))
        i = 0
        while i < len(self.seg_len) - 1 and s > self.cum[i + 1]:
            i += 1
        if self.seg_len[i] < 1e-9:
            return self.pts[i], 0.0
        t = (s - self.cum[i]) / self.seg_len[i]
        p = self.pts[i] + t * (self.pts[i + 1] - self.pts[i])
        d = self.pts[i + 1] - self.pts[i]
        return p, math.atan2(d[1], d[0])

    def project(self, x, y, search_ahead=1.0):
        """
        Abscisse curviligne du robot, cherchée uniquement *en avant* de la
        position précédente. Un chemin de labyrinthe repasse près de lui-même
        (couloirs parallèles séparés de 40 cm) : une projection globale pourrait
        accrocher un segment situé beaucoup plus loin et faire sauter le suivi.
        """
        best_s, best_d = self._s, float("inf")
        s = self._s
        step = 0.02
        while s <= min(self.total, self._s + search_ahead):
            p, _ = self._point_at(s)
            d = (p[0] - x) ** 2 + (p[1] - y) ** 2
            if d < best_d:
                best_d, best_s = d, s
            s += step
        self._s = best_s
        return best_s

    def reference(self, x, y, horizon, dt, v_nom):
        """
        Référence [3 x (horizon+1)] : positions, puis cap tangent.

        Le dernier tronçon est saturé sur le goal : une fois l'abscisse arrivée
        au bout, tous les points de l'horizon s'y accumulent, ce qui transforme
        naturellement le suivi en régulation de position finale.
        """
        s0 = self.project(x, y)
        ref = np.zeros((3, horizon + 1))
        for k in range(horizon + 1):
            p, th = self._point_at(s0 + v_nom * dt * k)
            ref[0, k], ref[1, k], ref[2, k] = p[0], p[1], th
        # Continuité de phase : le MPC voit θ comme une variable non bornée, il
        # faut donc que la référence ne saute pas de +π à −π au fil de l'horizon.
        for k in range(1, horizon + 1):
            ref[2, k] = ref[2, k - 1] + wrap_angle(ref[2, k] - ref[2, k - 1])
        return ref, s0

    def remaining(self, s):
        return self.total - s


class MPCController:
    """
    Solveur MPC réutilisable. La structure du problème est figée à la
    construction ; à chaque appel seuls les *paramètres* changent (état courant,
    référence, murs voisins, commande précédente), ce qui évite de reconstruire
    le graphe CasADi 20 fois par seconde.
    """

    def __init__(self, horizon=18, dt=0.10,
                 v_max=0.30, w_max=1.0,
                 a_max=0.6, alpha_max=1.5,
                 robot_radius=0.177, safety_margin=0.023,
                 n_wall_slots=10,
                 q_pos=60.0, q_yaw=6.0, r_v=0.6, r_w=0.3,
                 rd_v=12.0, rd_w=4.0, w_align=45.0, w_wall=25000.0,
                 terminal_scale=12.0, max_iter=60, max_solve_time=0.15):
        self.N = horizon
        self.dt = dt
        self.v_max = v_max
        self.w_max = w_max
        self.a_max = a_max
        self.alpha_max = alpha_max
        self.margin = robot_radius + safety_margin
        self.n_wall_slots = n_wall_slots
        self.max_iter = max_iter
        self.max_solve_time = max_solve_time

        N = self.N
        opti = ca.Opti()
        X = opti.variable(3, N + 1)
        U = opti.variable(2, N)

        p_x0     = opti.parameter(3)
        p_ref    = opti.parameter(3, N + 1)
        p_uprev  = opti.parameter(2)
        p_walls  = opti.parameter(4, n_wall_slots)   # ax, ay, bx, by

        Q  = ca.diag(ca.vertcat(q_pos, q_pos, q_yaw))
        R  = ca.diag(ca.vertcat(r_v, r_w))
        Rd = ca.diag(ca.vertcat(rd_v, rd_w))

        cost = 0
        for k in range(N):
            e = X[:, k] - p_ref[:, k]
            cost += ca.mtimes([e.T, Q, e])
            cost += ca.mtimes([U[:, k].T, R, U[:, k]])
            du = U[:, k] - (p_uprev if k == 0 else U[:, k - 1])
            cost += ca.mtimes([du.T, Rd, du])
            # Avancer vite en étant désaligné est ce qui envoie le robot dans un mur.
            cost += w_align * (U[0, k] * ca.sin(X[2, k] - p_ref[2, k])) ** 2

        e_T = X[:, N] - p_ref[:, N]
        cost += terminal_scale * ca.mtimes([e_T.T, Q, e_T])

        for k in range(N + 1):
            for j in range(n_wall_slots):
                d = self._segment_distance(X[0, k], X[1, k], p_walls[:, j])
                cost += w_wall * ca.fmax(0.0, self.margin - d) ** 2

        opti.minimize(cost)

        # Intégration point-milieu : à dt = 0,1 s et ω jusqu'à 1 rad/s, Euler
        # explicite sous-estime nettement l'arc parcouru pendant un virage.
        for k in range(N):
            th_mid = X[2, k] + 0.5 * U[1, k] * dt
            opti.subject_to(X[0, k + 1] == X[0, k] + U[0, k] * ca.cos(th_mid) * dt)
            opti.subject_to(X[1, k + 1] == X[1, k] + U[0, k] * ca.sin(th_mid) * dt)
            opti.subject_to(X[2, k + 1] == X[2, k] + U[1, k] * dt)

        opti.subject_to(X[:, 0] == p_x0)
        opti.subject_to(opti.bounded(0.0, ca.vec(U[0, :]), v_max))
        opti.subject_to(opti.bounded(-w_max, ca.vec(U[1, :]), w_max))
        for k in range(N):
            prev = p_uprev if k == 0 else U[:, k - 1]
            opti.subject_to(ca.fabs(U[0, k] - prev[0]) <= a_max * dt)
            opti.subject_to(ca.fabs(U[1, k] - prev[1]) <= alpha_max * dt)

        opti.solver("ipopt", {
            "print_time": False,
            "ipopt.print_level": 0,
            "ipopt.sb": "yes",
            "ipopt.max_iter": max_iter,
            # Plafond dur sur le temps de résolution. Sans lui, une itération
            # difficile bloque la boucle de contrôle : le nœud est mono-thread,
            # donc pendant que le solveur cherche, ni l'odométrie n'est lue ni
            # la commande n'est rafraîchie, et le robot continue à l'aveugle
            # sur son dernier ordre. Mieux vaut abandonner et basculer sur le
            # repli que de laisser le robot rouler sans supervision.
            "ipopt.max_wall_time": max_solve_time,
            "ipopt.tol": 1e-4,
            "ipopt.acceptable_tol": 1e-3,
            "ipopt.acceptable_iter": 5,
            "ipopt.warm_start_init_point": "yes",
            "ipopt.mu_strategy": "adaptive",
        })

        self.opti, self.X, self.U = opti, X, U
        self.p_x0, self.p_ref, self.p_uprev, self.p_walls = p_x0, p_ref, p_uprev, p_walls
        self._X_warm = None
        self._U_warm = None
        self.last_solve_time = 0.0
        self.fail_count = 0

    def reset(self):
        """
        Oublie le réamorçage à chaud.

        À appeler dès que le problème change de nature — nouveau chemin, robot
        replacé. Repartir de la solution d'un problème sans rapport est
        dangereux avec des pénalités de murs douces : l'optimiseur peut rester
        accroché à un minimum local situé *de l'autre côté* d'un mur, où la
        pénalité est de nouveau nulle, et sortir une trajectoire qui traverse.
        """
        self._X_warm = None
        self._U_warm = None
        self.fail_count = 0

    @staticmethod
    def _segment_distance(px, py, seg):
        """Distance symbolique d'un point à un segment, dérivable partout."""
        ax, ay, bx, by = seg[0], seg[1], seg[2], seg[3]
        vx, vy = bx - ax, by - ay
        wx, wy = px - ax, py - ay
        denom = vx * vx + vy * vy + 1e-9
        t = ca.fmin(1.0, ca.fmax(0.0, (wx * vx + wy * vy) / denom))
        dx = px - (ax + t * vx)
        dy = py - (ay + t * vy)
        return ca.sqrt(dx * dx + dy * dy + 1e-9)

    def select_walls(self, x, y, segments):
        """
        Les `n_wall_slots` murs les plus proches, complétés par des segments
        lointains. Le nombre de murs doit rester constant d'un appel à l'autre
        pour que la structure du NLP ne change pas ; ne garder que les plus
        proches suffit puisque l'horizon ne couvre qu'une poignée de cellules.
        """
        from .maze_map import point_segment_distance
        ranked = sorted(segments, key=lambda s: point_segment_distance(x, y, *s))
        chosen = ranked[:self.n_wall_slots]
        out = np.full((4, self.n_wall_slots), _FAR_AWAY, dtype=float)
        for j, s in enumerate(chosen):
            out[:, j] = s
        return out

    def solve(self, pose, ref, walls_param, u_prev):
        """
        Résout un pas de MPC.

        Retourne `(v, ω, info)`. En cas d'échec du solveur, `info["ok"]` est faux
        et l'appelant doit basculer sur son repli — le MPC ne renvoie jamais une
        commande issue d'une itération non convergée.
        """
        import time

        opti = self.opti
        # Le cap est manipulé sans repliement dans le NLP : on rapproche l'état
        # courant de la première référence pour éviter un écart artificiel de 2π.
        yaw = ref[2, 0] + wrap_angle(pose[2] - ref[2, 0])

        opti.set_value(self.p_x0, [pose[0], pose[1], yaw])
        opti.set_value(self.p_ref, ref)
        opti.set_value(self.p_uprev, u_prev)
        opti.set_value(self.p_walls, walls_param)

        if self._X_warm is not None:
            # Réamorçage décalé d'un pas : la solution précédente est déjà
            # presque optimale pour le problème courant.
            Xw = np.hstack([self._X_warm[:, 1:], self._X_warm[:, -1:]])
            Uw = np.hstack([self._U_warm[:, 1:], self._U_warm[:, -1:]])
            Xw[2, :] = yaw + np.unwrap(Xw[2, :] - Xw[2, 0])
            opti.set_initial(self.X, Xw)
            opti.set_initial(self.U, Uw)
        else:
            opti.set_initial(self.X, np.tile(np.array([[pose[0]], [pose[1]], [yaw]]),
                                             (1, self.N + 1)))
            opti.set_initial(self.U, np.zeros((2, self.N)))

        t0 = time.monotonic()
        try:
            sol = opti.solve()
            Xs, Us = sol.value(self.X), sol.value(self.U)
            ok = True
        except RuntimeError:
            # `debug.value` donne l'itéré courant : utile pour diagnostiquer,
            # mais on ne le renvoie pas comme commande.
            try:
                Xs, Us = opti.debug.value(self.X), opti.debug.value(self.U)
            except Exception:
                Xs = Us = None
            ok = False
        self.last_solve_time = time.monotonic() - t0

        if not ok or Us is None or not np.all(np.isfinite(Us)):
            self.fail_count += 1
            # Le réamorçage précédent est *conservé*. Il vient du même problème,
            # décalé d'un pas, et reste le meilleur point de départ connu ;
            # l'effacer condamnerait le cycle suivant à repartir à froid, donc à
            # être plus lent, donc à échouer lui aussi — un échec isolé se
            # transforme alors en cascade. Seul `reset()`, appelé quand le
            # problème change vraiment de nature, l'invalide.
            return 0.0, 0.0, {"ok": False, "solve_time": self.last_solve_time}

        self._X_warm, self._U_warm = np.asarray(Xs), np.asarray(Us).reshape(2, self.N)
        v = float(np.clip(self._U_warm[0, 0], 0.0, self.v_max))
        w = float(np.clip(self._U_warm[1, 0], -self.w_max, self.w_max))
        return v, w, {
            "ok": True,
            "solve_time": self.last_solve_time,
            "predicted": self._X_warm,
        }


def fallback_command(pose, ref, v_max, w_max, linear_kp=0.6, angular_kp=1.5,
                     align_tol=0.20):
    """
    Commande de secours quand le solveur échoue : viser le premier point de
    référence, pivoter sur place tant qu'on est trop désaligné.

    Ce n'est pas un contrôleur alternatif entretenu en parallèle du MPC, juste
    de quoi garder le robot sûr et lentement progressant le temps que le solveur
    reparte au cycle suivant.
    """
    dx = ref[0, 1] - pose[0]
    dy = ref[1, 1] - pose[1]
    yaw_err = wrap_angle(math.atan2(dy, dx) - pose[2])
    w = max(-w_max, min(w_max, angular_kp * yaw_err))
    if abs(yaw_err) < align_tol:
        v = min(v_max, linear_kp * math.hypot(dx, dy)) * math.cos(yaw_err) ** 8
    else:
        v = 0.0
    return max(0.0, v), w
