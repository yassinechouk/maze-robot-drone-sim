#!/usr/bin/env python3
"""
maze_map_validator.py — Compare la carte extraite par vision à la vérité terrain.

Le générateur de monde écrit, à côté du SDF, un JSON contenant la grille de murs
réellement construite dans Gazebo. Ce nœud écoute la carte publiée par le drone
et la confronte à ce fichier, cellule par cellule.

C'est un outil de diagnostic : ni le robot ni le drone ne le consultent, et rien
dans la chaîne de navigation n'en dépend. Il sert à répondre précisément à « la
vision a-t-elle vu le bon labyrinthe ? » sans avoir à lire des grilles ASCII à
l'œil, et à chiffrer l'erreur géométrique là où elle compte : la taille de
cellule et l'origine, qui se propagent directement dans les waypoints.

    ros2 run maze_robot_sim maze_map_validator --ros-args \\
        -p reference_json:=/chemin/vers/generated_maze.json
"""
import json

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from . import maze_map

_DIR_NAMES = {maze_map.N: "N", maze_map.E: "E", maze_map.S: "S", maze_map.W: "W"}


def latched_qos():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class MazeMapValidator(Node):

    def __init__(self):
        super().__init__("maze_map_validator")
        self.declare_parameter("reference_json", "")
        self.declare_parameter("map_topic", "/maze/occupancy_grid")

        path = str(self.get_parameter("reference_json").value)
        if not path:
            raise RuntimeError("Le paramètre 'reference_json' est requis.")
        with open(path) as f:
            self.ref = json.load(f)
        self.get_logger().info(
            f"Référence chargée : {self.ref['rows']}x{self.ref['cols']}, "
            f"cellule {self.ref['cell_size']} m ({path})")

        self.start_cell = None
        self.goal_cell = None
        self.reported = False

        self.create_subscription(OccupancyGrid, str(self.get_parameter("map_topic").value),
                                 self.map_cb, latched_qos())
        self.create_subscription(PoseStamped, "/maze/start_pose",
                                 lambda m: self._pose_cb(m, "start"), latched_qos())
        self.create_subscription(PoseStamped, "/maze/goal_pose",
                                 lambda m: self._pose_cb(m, "goal"), latched_qos())
        self._last_grid = None

    def _pose_cb(self, msg, which):
        setattr(self, f"{which}_cell", (msg.pose.position.x, msg.pose.position.y))
        self._maybe_report()

    def map_cb(self, msg: OccupancyGrid):
        self._last_grid = msg
        self._maybe_report()

    def _maybe_report(self):
        if self.reported or self._last_grid is None:
            return
        if self.start_cell is None or self.goal_cell is None:
            return
        self.reported = True
        self._report(self._last_grid)

    def _report(self, grid):
        ref = self.ref
        rows_ref, cols_ref = ref["rows"], ref["cols"]
        cell_ref = ref["cell_size"]

        walls, rows, cols = maze_map.grid_data_to_walls(
            list(grid.data), grid.info.width, grid.info.height)
        cell = grid.info.resolution * maze_map.SUB_RESOLUTION
        ox, oy = maze_map.grid_origin_to_cell_origin(
            grid.info.origin.position.x, grid.info.origin.position.y, cell)

        lines = ["", "═══ Validation de la carte extraite par vision ═══"]
        lines.append(f"  dimensions : vue {rows}x{cols} | réelle {rows_ref}x{cols_ref} "
                     f"→ {'OK' if (rows, cols) == (rows_ref, cols_ref) else 'ÉCART'}")
        lines.append(f"  cellule    : vue {cell*1000:.1f} mm | réelle {cell_ref*1000:.1f} mm "
                     f"→ écart {abs(cell-cell_ref)*1000:.1f} mm")
        lines.append(f"  origine    : vue ({ox*1000:+.0f}, {oy*1000:+.0f}) mm | "
                     f"réelle (0, 0) mm → écart {max(abs(ox), abs(oy))*1000:.1f} mm")

        if (rows, cols) == (rows_ref, cols_ref):
            wrong = []
            for r in range(rows):
                for c in range(cols):
                    diff = walls[r][c] ^ ref["walls"][r][c]
                    for d, name in _DIR_NAMES.items():
                        if diff & d:
                            kind = "inventé" if walls[r][c] & d else "manqué"
                            wrong.append(f"({r},{c}){name} {kind}")
            total = rows * cols * 4
            lines.append(f"  murs       : {total - len(wrong)}/{total} corrects "
                         f"→ {'PARFAIT' if not wrong else str(len(wrong)) + ' écarts'}")
            for w in wrong[:20]:
                lines.append(f"               • {w}")
            if len(wrong) > 20:
                lines.append(f"               … et {len(wrong) - 20} autres")

            # Un mur manqué est bien plus grave qu'un mur inventé : il autorise
            # un chemin qui traverse une cloison réelle.
            seen_start = maze_map.world_to_cell(*self.start_cell, rows, cols, cell, ox, oy)
            seen_goal = maze_map.world_to_cell(*self.goal_cell, rows, cols, cell, ox, oy)
            lines.append(f"  départ     : vu {seen_start} | réel {tuple(ref['start'])} "
                         f"→ {'OK' if list(seen_start) == ref['start'] else 'ÉCART'}")
            lines.append(f"  arrivée    : vu {seen_goal} | réel {tuple(ref['goal'])} "
                         f"→ {'OK' if list(seen_goal) == ref['goal'] else 'ÉCART'}")

            path_seen = maze_map.astar(walls, rows, cols, seen_start, seen_goal)
            path_ref = [tuple(p) for p in ref["path"]]
            same = path_seen == path_ref
            lines.append(f"  chemin A*  : {len(path_seen)} cellules vues | "
                         f"{len(path_ref)} réelles → "
                         f"{'IDENTIQUE' if same else 'DIFFÉRENT'}")
            if path_seen and not same:
                traverse = [(a, b) for a, b in zip(path_seen, path_seen[1:])
                            if not self._connected(ref["walls"], a, b)]
                if traverse:
                    lines.append(f"  ⚠ le chemin vu traverse {len(traverse)} mur(s) réel(s) : "
                                 f"{traverse[:5]}")
                else:
                    lines.append("  le chemin vu est différent mais reste praticable "
                                 "dans le labyrinthe réel")

        self.get_logger().info("\n".join(lines))

    @staticmethod
    def _connected(walls_ref, a, b):
        """Les deux cellules adjacentes sont-elles réellement reliées ?"""
        dr, dc = b[0] - a[0], b[1] - a[1]
        for d, (ddr, ddc) in maze_map.DELTA.items():
            if (ddr, ddc) == (dr, dc):
                return not (walls_ref[a[0]][a[1]] & d)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = MazeMapValidator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
