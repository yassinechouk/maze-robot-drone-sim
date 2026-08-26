#!/usr/bin/env python3
"""
Nœud de navigation A* : fait parcourir au robot le chemin calculé depuis le fichier
.json généré par maze_world_generator.py, du start jusqu'au goal.

Contrôleur go-to-goal séquentiel sur les centres de cellules du chemin A* :
  - Erreur d'angle → commande angulaire (P controller)
  - Si bien aligné  → avance en vitesse linéaire proportionnelle à la distance
  - Facteur cos(err)^8 pour un mouvement fluide sans toucher les murs

AUTO-CALIBRATION DU SENS DE ROTATION :
  Avant de commencer, une brève rotation pure (angular.z > 0 pendant ~0.6s) est
  effectuée pour détecter si la convention angulaire est inversée entre la commande
  envoyée au diff_drive_controller et l'odométrie publiée. Si c'est le cas,
  angular_sign=-1 est appliqué automatiquement à toutes les commandes angulaires.

  Paramètre `invert_angular` (défaut: "" = auto) :
    ros2 launch maze_robot_sim maze_sim.launch.py invert_angular:=true   # forcer inversion
    ros2 launch maze_robot_sim maze_sim.launch.py invert_angular:=false  # forcer normal

Topics :
  Souscrit  : <odom_topic>    (nav_msgs/Odometry,   défaut /diff_drive_controller/odom)
              <imu_topic>     (sensor_msgs/Imu,      défaut /imu) — non utilisé activement
  Publie    : <cmd_vel_topic> (geometry_msgs/TwistStamped, défaut /diff_drive_controller/cmd_vel)
              /maze_navigator/status (std_msgs/String) — CALIBRATING / WAYPOINT_i/N / GOAL_REACHED

Note ROS 2 Jazzy : le diff_drive_controller n'accepte que geometry_msgs/TwistStamped
sur ~/cmd_vel (le topic ~/cmd_vel_unstamped n'existe plus depuis Jazzy).
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion


def cell_center_xy(r, c, rows, cols, cell_size):
    x = c * cell_size
    y = (rows - 1 - r) * cell_size
    return x, y


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class MazeNavigatorNode(Node):
    # États internes de la machine à états
    STATE_WAIT_ODOM        = "wait_odom"
    STATE_CALIBRATE_SPIN_UP = "calibrate_spin_up"
    STATE_CALIBRATE_MEASURE = "calibrate_measure"
    STATE_NAVIGATE         = "navigate"
    STATE_DONE             = "done"

    def __init__(self):
        super().__init__("maze_navigator_node")

        self.declare_parameter("maze_json",                 "")
        self.declare_parameter("linear_kp",                 0.9)
        self.declare_parameter("angular_kp",                1.8)
        self.declare_parameter("max_linear_speed",          0.3)
        self.declare_parameter("max_angular_speed",         1.6)
        self.declare_parameter("goal_tolerance",            0.05)
        self.declare_parameter("align_tolerance_rad",       0.35)
        self.declare_parameter("cmd_vel_topic",             "/diff_drive_controller/cmd_vel")
        self.declare_parameter("odom_topic",                "/diff_drive_controller/odom")
        self.declare_parameter("imu_topic",                 "/imu")
        self.declare_parameter("odom_timeout_warn_sec",     5.0)
        self.declare_parameter("invert_angular",            "")
        self.declare_parameter("calibration_angular_speed", 0.8)
        self.declare_parameter("calibration_duration_sec",  0.6)

        maze_json_path = self.get_parameter("maze_json").get_parameter_value().string_value
        if not maze_json_path:
            raise RuntimeError("Le paramètre 'maze_json' est requis (chemin vers le .json généré).")

        with open(maze_json_path) as f:
            meta = json.load(f)

        self.rows       = meta["rows"]
        self.cols       = meta["cols"]
        self.cell_size  = meta["cell_size"]
        self.path_cells = [tuple(p) for p in meta["path"]]
        self.waypoints  = [cell_center_xy(r, c, self.rows, self.cols, self.cell_size)
                           for (r, c) in self.path_cells]

        self.wp_idx    = 0
        self.pose_x    = 0.0
        self.pose_y    = 0.0
        self.pose_yaw  = 0.0
        self.have_odom = False
        self._odom_msg_count = 0

        invert_param = self.get_parameter("invert_angular").get_parameter_value().string_value.strip().lower()
        if invert_param in ("true", "1", "yes"):
            self.angular_sign      = -1.0
            self.state             = self.STATE_WAIT_ODOM
            self._skip_calibration = True
            self.get_logger().info("invert_angular forcé à TRUE (angular_sign=-1) — calibration auto désactivée.")
        elif invert_param in ("false", "0", "no"):
            self.angular_sign      = 1.0
            self.state             = self.STATE_WAIT_ODOM
            self._skip_calibration = True
            self.get_logger().info("invert_angular forcé à FALSE (angular_sign=+1) — calibration auto désactivée.")
        else:
            self.angular_sign      = 1.0
            self.state             = self.STATE_WAIT_ODOM
            self._skip_calibration = False

        self._calib_start_yaw  = None
        self._calib_start_time = None

        cmd_topic  = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        imu_topic  = self.get_parameter("imu_topic").get_parameter_value().string_value

        self.cmd_pub    = self.create_publisher(TwistStamped, cmd_topic, 10)
        self.status_pub = self.create_publisher(String, "/maze_navigator/status", 10)

        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(Odometry, odom_topic, self.odom_cb, odom_qos)
        self.create_subscription(Imu, imu_topic, self.imu_cb, 20)

        self.get_logger().info(
            f"Labyrinthe {self.rows}x{self.cols} (cell={self.cell_size}m) — "
            f"{len(self.waypoints)} waypoints chargés depuis {maze_json_path}")
        self.get_logger().info(
            f"Écoute odométrie sur '{odom_topic}', IMU sur '{imu_topic}', "
            f"publie cmd_vel (TwistStamped) sur '{cmd_topic}'.")

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        warn_delay = self.get_parameter("odom_timeout_warn_sec").get_parameter_value().double_value
        self._odom_watchdog = self.create_timer(warn_delay, self._check_odom_received)

    def _check_odom_received(self):
        self._odom_watchdog.cancel()
        if not self.have_odom:
            odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
            self.get_logger().error(
                f"Aucun message d'odométrie reçu sur '{odom_topic}' ! "
                "Vérifiez qu'un publisher actif existe : "
                "`ros2 topic info <topic> --verbose` doit montrer "
                "'Publisher count: 1'. Le robot ne bougera pas tant que "
                "l'odométrie n'arrive pas.")

    def odom_cb(self, msg: Odometry):
        start_x, start_y = self.waypoints[0]
        self.pose_x = start_x + msg.pose.pose.position.x
        self.pose_y = start_y + msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose_yaw = yaw
        if not self.have_odom:
            self.get_logger().info(
                f"Premier message d'odométrie reçu. Pose globale estimée : "
                f"x={self.pose_x:.2f}, y={self.pose_y:.2f}")
        self.have_odom = True
        self._odom_msg_count += 1

    def imu_cb(self, msg: Imu):
        pass  # Souscrit mais non utilisé activement (log disponible si besoin)

    def _publish_cmd(self, linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x  = linear_x
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def control_loop(self):
        now = self.get_clock().now()

        if self.state == self.STATE_WAIT_ODOM:
            if not self.have_odom:
                return
            if self._skip_calibration:
                self.state = self.STATE_NAVIGATE
                self.get_logger().info("Démarrage de la navigation (calibration ignorée).")
            else:
                self.state = self.STATE_CALIBRATE_SPIN_UP
                self.status_pub.publish(String(data="CALIBRATING"))
                self.get_logger().info("Calibration du sens de rotation en cours...")
            return

        if self.state == self.STATE_CALIBRATE_SPIN_UP:
            calib_w = self.get_parameter("calibration_angular_speed").get_parameter_value().double_value
            self._calib_start_yaw  = self.pose_yaw
            self._calib_start_time = now
            self._publish_cmd(0.0, calib_w)
            self.state = self.STATE_CALIBRATE_MEASURE
            return

        if self.state == self.STATE_CALIBRATE_MEASURE:
            calib_w  = self.get_parameter("calibration_angular_speed").get_parameter_value().double_value
            duration = self.get_parameter("calibration_duration_sec").get_parameter_value().double_value
            self._publish_cmd(0.0, calib_w)
            elapsed = (now - self._calib_start_time).nanoseconds * 1e-9
            if elapsed < duration:
                return
            yaw_delta = wrap_angle(self.pose_yaw - self._calib_start_yaw)
            self._publish_cmd(0.0, 0.0)
            if yaw_delta < 0.0:
                self.angular_sign = -1.0
                self.get_logger().warning(
                    f"Calibration : commande angulaire positive a produit un "
                    f"yaw décroissant (Δyaw={math.degrees(yaw_delta):.1f}°). "
                    "Inversion automatique du signe des commandes angulaires "
                    "(angular_sign=-1) pour compenser.")
            else:
                self.angular_sign = 1.0
                self.get_logger().info(
                    f"Calibration : convention angulaire correcte confirmée "
                    f"(Δyaw={math.degrees(yaw_delta):.1f}°). angular_sign=+1.")
            self.state = self.STATE_NAVIGATE
            self.get_logger().info("Démarrage de la navigation.")
            return

        if self.state == self.STATE_DONE:
            return

        # ── STATE_NAVIGATE ──
        if self.wp_idx >= len(self.waypoints):
            self.state = self.STATE_DONE
            self._publish_cmd(0.0, 0.0)
            self.status_pub.publish(String(data="GOAL_REACHED"))
            self.get_logger().info("Goal atteint — arrêt du robot.")
            return

        tx, ty = self.waypoints[self.wp_idx]
        dx   = tx - self.pose_x
        dy   = ty - self.pose_y
        dist = math.hypot(dx, dy)

        goal_tol = self.get_parameter("goal_tolerance").get_parameter_value().double_value
        if dist < goal_tol:
            self.wp_idx += 1
            self.status_pub.publish(String(data=f"WAYPOINT_{self.wp_idx}/{len(self.waypoints)}"))
            self.get_logger().info(f"Waypoint {self.wp_idx}/{len(self.waypoints)} atteint.")
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err    = wrap_angle(target_yaw - self.pose_yaw)

        angular_kp = self.get_parameter("angular_kp").get_parameter_value().double_value
        linear_kp  = self.get_parameter("linear_kp").get_parameter_value().double_value
        max_v      = self.get_parameter("max_linear_speed").get_parameter_value().double_value
        max_w      = self.get_parameter("max_angular_speed").get_parameter_value().double_value
        align_tol  = self.get_parameter("align_tolerance_rad").get_parameter_value().double_value

        w_raw = max(-max_w, min(max_w, angular_kp * yaw_err))
        w     = self.angular_sign * w_raw

        # Mouvement fluide : cos(err)^8 réduit la vitesse linéaire en courbe
        if abs(yaw_err) < align_tol:
            align_factor = math.cos(yaw_err) ** 8
            v = max(0.0, min(max_v, linear_kp * dist)) * align_factor
        else:
            v = 0.0

        self._publish_cmd(v, w)


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
