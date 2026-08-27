#!/usr/bin/env python3
"""
maze_navigator_node.py — Navigation autonome du robot dans le labyrinthe, à
partir de la seule carte transmise par le drone.

Le nœud ne lit plus aucun fichier : il attend la carte publiée par
`drone_mapper_node`, la décode, calcule lui-même son chemin A*, puis le suit avec
un contrôleur prédictif (`mpc_controller`). Rien de ce qui décrit le labyrinthe
n'entre par la ligne de commande — c'est tout l'intérêt de l'architecture.

    ┌───────────────┐   ┌──────────────┐   ┌────────────────┐
    │ carte reçue   │──▶│ A* interne   │──▶│ MPC + murs     │──▶ cmd_vel
    └───────────────┘   └──────────────┘   └────────────────┘
     /maze/occupancy_grid                  /diff_drive_controller/cmd_vel
     /maze/start_pose
     /maze/goal_pose

Machine à états
---------------
    WAIT_MAP → WAIT_DRONE → WAIT_ODOM → CALIBRATE → NAVIGATE → DONE

Repérage
--------
L'odométrie du robot part de zéro à l'endroit du spawn, qui est le centre de la
cellule de départ. La pose de départ publiée par le drone donne ce point dans le
repère monde : la pose monde du robot est donc simplement la somme des deux.
C'est aussi la raison pour laquelle la carte doit arriver avant tout mouvement.

Auto-calibration du sens de rotation
------------------------------------
Une brève rotation pure permet de détecter si la convention angulaire de
l'odométrie est inversée par rapport aux commandes envoyées au
diff_drive_controller ; `angular_sign` compense le cas échéant. Le paramètre
`invert_angular` (défaut « » = auto) permet de forcer le comportement.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import euler_from_quaternion

from . import maze_map
from .mpc_controller import MPCController, PathTracker, fallback_command, wrap_angle


def latched_qos(depth=1):
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class MazeNavigatorNode(Node):

    STATE_WAIT_MAP          = "wait_map"
    STATE_WAIT_DRONE        = "wait_drone"
    STATE_WAIT_ODOM         = "wait_odom"
    STATE_CALIBRATE_SPIN_UP = "calibrate_spin_up"
    STATE_CALIBRATE_MEASURE = "calibrate_measure"
    STATE_NAVIGATE          = "navigate"
    STATE_DONE              = "done"

    def __init__(self):
        super().__init__("maze_navigator_node")

        self.declare_parameter("cmd_vel_topic",             "/diff_drive_controller/cmd_vel")
        self.declare_parameter("odom_topic",                "/diff_drive_controller/odom")
        self.declare_parameter("imu_topic",                 "/imu")
        self.declare_parameter("map_topic",                 "/maze/occupancy_grid")
        self.declare_parameter("start_pose_topic",          "/maze/start_pose")
        self.declare_parameter("goal_pose_topic",           "/maze/goal_pose")
        self.declare_parameter("map_frame",                 "map")
        self.declare_parameter("odom_frame",                "odom")
        self.declare_parameter("invert_angular",            "")
        self.declare_parameter("calibration_angular_speed", 0.8)
        self.declare_parameter("calibration_duration_sec",  0.6)
        self.declare_parameter("goal_tolerance",            0.05)
        self.declare_parameter("map_timeout_warn_sec",      45.0)
        self.declare_parameter("odom_timeout_warn_sec",     10.0)
        # ── Réglages MPC ──
        self.declare_parameter("mpc_horizon",               18)
        self.declare_parameter("mpc_dt",                    0.10)
        self.declare_parameter("mpc_decimation",            2)
        self.declare_parameter("mpc_max_solve_time",        0.15)
        self.declare_parameter("max_linear_speed",          0.30)
        self.declare_parameter("max_angular_speed",         1.0)
        self.declare_parameter("max_acceleration",          0.6)
        self.declare_parameter("max_angular_acceleration",  1.5)
        self.declare_parameter("robot_radius",              0.177)
        self.declare_parameter("safety_margin",             0.023)
        self.declare_parameter("wall_weight",               25000.0)
        self.declare_parameter("align_weight",              45.0)
        self.declare_parameter("nominal_speed",             0.30)

        p = self.get_parameter
        self.map_frame  = str(p("map_frame").value)
        self.odom_frame = str(p("odom_frame").value)
        self.goal_tol   = float(p("goal_tolerance").value)
        self.mpc_decim  = max(1, int(p("mpc_decimation").value))
        self.v_nom      = float(p("nominal_speed").value)

        # ── Carte ──
        self.grid_msg   = None
        self.start_pose = None
        self.goal_pose  = None
        self.planned    = False
        self.walls = self.rows = self.cols = None
        self.cell_size = None
        self.origin_x = self.origin_y = None
        self.waypoints = []
        self.wall_segs = []
        self.tracker = None
        self.mpc = None

        # ── État ──
        self.state = self.STATE_WAIT_MAP
        self.pose_x = self.pose_y = self.pose_yaw = 0.0
        self._odom_xy = None
        self._odom_yaw = 0.0
        self.have_odom = False      # le contrôleur de roues publie
        self.pose_valid = False     # ... et la pose monde est calculable
        self.drone_ready = False
        self.u_prev = (0.0, 0.0)
        self._tick = 0
        self._mpc_fail_streak = 0
        self._min_wall_dist = float("inf")
        self._calib_start_yaw = None
        self._calib_start_time = None

        invert = str(p("invert_angular").value).strip().lower()
        if invert in ("true", "1", "yes"):
            self.angular_sign, self._skip_calibration = -1.0, True
            self.get_logger().info("invert_angular forcé à TRUE (angular_sign=-1).")
        elif invert in ("false", "0", "no"):
            self.angular_sign, self._skip_calibration = 1.0, True
            self.get_logger().info("invert_angular forcé à FALSE (angular_sign=+1).")
        else:
            self.angular_sign, self._skip_calibration = 1.0, False

        cmd_topic  = str(p("cmd_vel_topic").value)
        odom_topic = str(p("odom_topic").value)

        self.cmd_pub    = self.create_publisher(TwistStamped, cmd_topic, 10)
        self.status_pub = self.create_publisher(String, "/maze_navigator/status", 10)
        self.diag_pub   = self.create_publisher(String, "/maze_navigator/diagnostics", 10)
        self.path_pub   = self.create_publisher(Path, "/maze_navigator/path", latched_qos())
        self.tf_static  = StaticTransformBroadcaster(self)

        self.create_subscription(OccupancyGrid, str(p("map_topic").value),
                                 self.map_cb, latched_qos())
        self.create_subscription(PoseStamped, str(p("start_pose_topic").value),
                                 self.start_cb, latched_qos())
        self.create_subscription(PoseStamped, str(p("goal_pose_topic").value),
                                 self.goal_cb, latched_qos())
        self.create_subscription(String, "/drone/mapper/ready", self.drone_ready_cb,
                                 latched_qos())

        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(Odometry, odom_topic, self.odom_cb, odom_qos)
        self.create_subscription(Imu, str(p("imu_topic").value), lambda _m: None, 20)

        self.timer = self.create_timer(0.05, self.control_loop)   # 20 Hz
        self._map_watchdog = self.create_timer(
            float(p("map_timeout_warn_sec").value), self._check_map_received)
        self._odom_watchdog = self.create_timer(
            float(p("odom_timeout_warn_sec").value), self._check_odom_received)

        self.get_logger().info(
            f"En attente de la carte du drone sur '{p('map_topic').value}' "
            f"(+ poses de départ et d'arrivée). Aucun fichier n'est lu.")

    # ──────────────────────────────────────────────────────────
    #  Réception de la carte
    # ──────────────────────────────────────────────────────────

    def map_cb(self, msg: OccupancyGrid):
        self.grid_msg = msg
        self._try_plan()

    def start_cb(self, msg: PoseStamped):
        self.start_pose = (msg.pose.position.x, msg.pose.position.y)
        self._update_world_pose()
        self._try_plan()

    def goal_cb(self, msg: PoseStamped):
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
        self._try_plan()

    def drone_ready_cb(self, msg: String):
        if msg.data == "READY" and not self.drone_ready:
            self.drone_ready = True
            self.get_logger().info("Signal READY du drone reçu.")

    def _try_plan(self):
        """Planifie dès que la grille et les deux poses sont toutes arrivées."""
        if self.planned or not (self.grid_msg and self.start_pose and self.goal_pose):
            return

        g = self.grid_msg
        self.walls, self.rows, self.cols = maze_map.grid_data_to_walls(
            list(g.data), g.info.width, g.info.height)
        self.cell_size = g.info.resolution * maze_map.SUB_RESOLUTION
        self.origin_x, self.origin_y = maze_map.grid_origin_to_cell_origin(
            g.info.origin.position.x, g.info.origin.position.y, self.cell_size)

        start_cell = maze_map.world_to_cell(*self.start_pose, self.rows, self.cols,
                                            self.cell_size, self.origin_x, self.origin_y)
        goal_cell = maze_map.world_to_cell(*self.goal_pose, self.rows, self.cols,
                                           self.cell_size, self.origin_x, self.origin_y)
        path = maze_map.astar(self.walls, self.rows, self.cols, start_cell, goal_cell)
        if not path:
            self.get_logger().error(
                f"Aucun chemin de {start_cell} à {goal_cell} dans la carte reçue. "
                "En attente d'une carte corrigée.")
            self.grid_msg = None
            return

        self.waypoints = [
            maze_map.cell_center_world(r, c, self.rows, self.cols, self.cell_size,
                                       self.origin_x, self.origin_y)
            for (r, c) in path]
        self.wall_segs = maze_map.wall_segments(
            self.walls, self.rows, self.cols, self.cell_size,
            self.origin_x, self.origin_y)
        self.tracker = PathTracker(self.waypoints)

        p = self.get_parameter
        self.mpc = MPCController(
            horizon=int(p("mpc_horizon").value),
            dt=float(p("mpc_dt").value),
            v_max=float(p("max_linear_speed").value),
            w_max=float(p("max_angular_speed").value),
            a_max=float(p("max_acceleration").value),
            alpha_max=float(p("max_angular_acceleration").value),
            robot_radius=float(p("robot_radius").value),
            safety_margin=float(p("safety_margin").value),
            w_wall=float(p("wall_weight").value),
            w_align=float(p("align_weight").value),
            max_solve_time=float(p("mpc_max_solve_time").value),
        )

        # L'odométrie démarre à zéro au point de spawn, qui est la cellule de
        # départ : la pose de départ vue par le drone est donc l'origine du
        # repère odom exprimée dans le repère carte.
        self._publish_static_map_to_odom()
        self._publish_path()

        self.planned = True
        self.get_logger().info(
            f"Carte reçue : {self.rows}x{self.cols}, cellule {self.cell_size*100:.1f} cm, "
            f"origine ({self.origin_x:.3f}, {self.origin_y:.3f}).")
        self.get_logger().info(
            f"Chemin calculé depuis la carte reçue : {start_cell} → {goal_cell}, "
            f"{len(self.waypoints)} waypoints, {self.tracker.total:.2f} m, "
            f"{len(self.wall_segs)} segments de murs pris en compte par le MPC.")

    def _publish_static_map_to_odom(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = self.start_pose[0]
        t.transform.translation.y = self.start_pose[1]
        t.transform.rotation.w = 1.0
        self.tf_static.sendTransform(t)

    def _publish_path(self):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in self.waypoints:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    # ──────────────────────────────────────────────────────────
    #  Odométrie et commandes
    # ──────────────────────────────────────────────────────────

    def odom_cb(self, msg: Odometry):
        """
        L'odométrie est enregistrée dès qu'elle arrive, même avant la carte : la
        conversion en pose monde a besoin de la pose de départ, mais savoir que
        le contrôleur de roues publie bien est une information distincte, et les
        confondre ferait crier le chien de garde alors que tout va bien.
        """
        q = msg.pose.pose.orientation
        self._odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._odom_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        if not self.have_odom:
            self.have_odom = True
            self.get_logger().info("Première odométrie reçue.")
        self._update_world_pose()

    def _update_world_pose(self):
        if self.start_pose is None or self._odom_xy is None:
            return
        self.pose_x = self.start_pose[0] + self._odom_xy[0]
        self.pose_y = self.start_pose[1] + self._odom_xy[1]
        self.pose_yaw = self._odom_yaw
        if not self.pose_valid:
            self.pose_valid = True
            self.get_logger().info(
                f"Pose monde initialisée : ({self.pose_x:.3f}, {self.pose_y:.3f}).")

    def _publish_cmd(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _check_map_received(self):
        self._map_watchdog.cancel()
        if not self.planned:
            missing = [n for n, v in (("grille", self.grid_msg),
                                      ("pose de départ", self.start_pose),
                                      ("pose d'arrivée", self.goal_pose)) if v is None]
            self.get_logger().error(
                f"Carte toujours incomplète — manque : {', '.join(missing)}. "
                "Le drone est-il lancé (spawn_drone:=true) et a-t-il réussi son "
                "extraction ? Voir `ros2 topic echo /drone/mapper/status`.")

    def _check_odom_received(self):
        self._odom_watchdog.cancel()
        if not self.have_odom:
            self.get_logger().error(
                "Aucune odométrie reçue — le robot ne bougera pas. Vérifiez que "
                "le diff_drive_controller est actif.")

    # ──────────────────────────────────────────────────────────
    #  Boucle de contrôle
    # ──────────────────────────────────────────────────────────

    def control_loop(self):
        now = self.get_clock().now()

        if self.state == self.STATE_WAIT_MAP:
            if self.planned:
                self.state = self.STATE_WAIT_DRONE
            return

        if self.state == self.STATE_WAIT_DRONE:
            if self.drone_ready:
                self.state = self.STATE_WAIT_ODOM
            return

        if self.state == self.STATE_WAIT_ODOM:
            if not self.pose_valid:
                return
            if self._skip_calibration:
                self.state = self.STATE_NAVIGATE
                self.get_logger().info("Démarrage de la navigation (calibration ignorée).")
            else:
                self.state = self.STATE_CALIBRATE_SPIN_UP
                self.status_pub.publish(String(data="CALIBRATING"))
                self.get_logger().info("Calibration du sens de rotation…")
            return

        if self.state == self.STATE_CALIBRATE_SPIN_UP:
            calib_w = float(self.get_parameter("calibration_angular_speed").value)
            self._calib_start_yaw = self.pose_yaw
            self._calib_start_time = now
            self._publish_cmd(0.0, calib_w)
            self.state = self.STATE_CALIBRATE_MEASURE
            return

        if self.state == self.STATE_CALIBRATE_MEASURE:
            calib_w = float(self.get_parameter("calibration_angular_speed").value)
            duration = float(self.get_parameter("calibration_duration_sec").value)
            self._publish_cmd(0.0, calib_w)
            if (now - self._calib_start_time).nanoseconds * 1e-9 < duration:
                return
            yaw_delta = wrap_angle(self.pose_yaw - self._calib_start_yaw)
            self._publish_cmd(0.0, 0.0)
            self.angular_sign = -1.0 if yaw_delta < 0.0 else 1.0
            if yaw_delta < 0.0:
                self.get_logger().warning(
                    f"Calibration : commande positive → yaw décroissant "
                    f"(Δ={math.degrees(yaw_delta):.1f}°). angular_sign=-1.")
            else:
                self.get_logger().info(
                    f"Calibration : convention correcte "
                    f"(Δ={math.degrees(yaw_delta):.1f}°). angular_sign=+1.")
            # La rotation vient de fausser toute prédiction antérieure.
            self.mpc.reset()
            self.u_prev = (0.0, 0.0)
            self.state = self.STATE_NAVIGATE
            self.get_logger().info("Démarrage de la navigation.")
            return

        if self.state == self.STATE_DONE:
            return

        self._navigate()

    def _navigate(self):
        gx, gy = self.waypoints[-1]
        dist_goal = math.hypot(gx - self.pose_x, gy - self.pose_y)
        if dist_goal < self.goal_tol:
            self._publish_cmd(0.0, 0.0)
            self.state = self.STATE_DONE
            self.status_pub.publish(String(data="GOAL_REACHED"))
            self.get_logger().info(
                f"Goal atteint (écart {dist_goal*100:.1f} cm). "
                f"Distance minimale aux murs sur tout le parcours : "
                f"{self._min_wall_dist*100:.1f} cm "
                f"(dégagement {(self._min_wall_dist - float(self.get_parameter('robot_radius').value))*100:.1f} cm).")
            return

        pose = (self.pose_x, self.pose_y, self.pose_yaw)
        d_wall = min(maze_map.point_segment_distance(self.pose_x, self.pose_y, *s)
                     for s in self.wall_segs)
        self._min_wall_dist = min(self._min_wall_dist, d_wall)

        if self._tick % self.mpc_decim == 0:
            ref, s0 = self.tracker.reference(self.pose_x, self.pose_y,
                                             self.mpc.N, self.mpc.dt, self.v_nom)
            walls_param = self.mpc.select_walls(self.pose_x, self.pose_y, self.wall_segs)
            v, w, info = self.mpc.solve(pose, ref, walls_param, self.u_prev)
            if info["ok"]:
                self._mpc_fail_streak = 0
            else:
                self._mpc_fail_streak += 1
                v, w = fallback_command(pose, ref,
                                        self.mpc.v_max, self.mpc.w_max)
                self.get_logger().warning(
                    f"MPC non convergé ({self._mpc_fail_streak} d'affilée, "
                    f"{info['solve_time']*1000:.0f} ms) — commande de secours.",
                    throttle_duration_sec=1.0)
            self.u_prev = (v, w)
            self._publish_diagnostics(v, w, d_wall, s0, info)
        else:
            v, w = self.u_prev

        self._publish_cmd(v, self.angular_sign * w)
        self._tick += 1

    def _publish_diagnostics(self, v, w, d_wall, s, info):
        radius = float(self.get_parameter("robot_radius").value)
        self.diag_pub.publish(String(data=(
            f"v={v:.3f} w={w:.3f} "
            f"pose=({self.pose_x:.3f},{self.pose_y:.3f},{math.degrees(self.pose_yaw):.1f}) "
            f"s={s:.2f}/{self.tracker.total:.2f} "
            f"d_wall={d_wall:.4f} clearance={d_wall - radius:.4f} "
            f"mpc_ok={int(bool(info['ok']))} solve_ms={info['solve_time']*1000:.1f}")))
        progress = int(100 * s / max(self.tracker.total, 1e-6))
        self.status_pub.publish(String(data=f"NAVIGATE_{progress}%"))


def main(args=None):
    rclpy.init(args=args)
    node = MazeNavigatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            try:
                node._publish_cmd(0.0, 0.0)
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
