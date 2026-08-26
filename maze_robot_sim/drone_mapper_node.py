#!/usr/bin/env python3
"""
drone_mapper_node.py — Suivi ArUco temps réel du robot au sol par le drone Crazyflie.

Machine à états : WAIT → TAKEOFF → HOVER_MAP → TRACKING → LANDING → DONE

BUG FIXES appliqués :
  #1 — cmd_vel_topic pointe désormais sur /model/crazyflie/cmd_vel (topic Gazebo scopé au
       modèle, seul topic que MulticopterVelocityControl écoute réellement).
  #2 — L'altitude n'est PLUS estimée par intégration open-loop des commandes publiées.
       Elle est lue depuis la pose Gazebo réelle via le topic pose_topic
       (/model/crazyflie/pose, bridgé par ros_gz_bridge). Cela évite le cas où la
       machine à états passe en HOVER_MAP/TRACKING alors que le drone est toujours au sol.
  #3 — QoS de /maze_navigator/status corrigée : VOLATILE au lieu de TRANSIENT_LOCAL
       pour correspondre à ce que maze_navigator_node publie réellement.
  #4 — Joints rotor passés en 'continuous' dans crazyflie.urdf.xacro (corrigé en
       amont dans le XACRO, pas ici, mais documenté pour traçabilité).

Topics :
  Souscrit :
    <image_topic>             (sensor_msgs/Image)  — caméra top-down du drone
    <pose_topic>              (geometry_msgs/Pose)  — pose Gazebo réelle du drone
    /maze_navigator/status    (std_msgs/String)      — statut navigation robot
  Publie :
    <cmd_vel_topic>           (geometry_msgs/Twist)  — vitesse drone → MulticopterVelocityControl
    /drone/mapper/status      (std_msgs/String)       — état interne du drone
"""
import math
import signal

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


# ── ArUco (DICT_4X4_50, ID 0 — correspond au robot_core.xacro) ──
_ARUCO_DICT     = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_ARUCO_PARAMS   = cv2.aruco.DetectorParameters()
_ARUCO_DETECTOR = cv2.aruco.ArucoDetector(_ARUCO_DICT, _ARUCO_PARAMS)

# ── États ──
STATE_WAIT      = "wait"
STATE_TAKEOFF   = "takeoff"
STATE_HOVER_MAP = "hover_map"
STATE_TRACKING  = "tracking"
STATE_LANDING   = "landing"
STATE_DONE      = "done"


class DroneMapperNode(Node):

    def __init__(self):
        super().__init__("drone_mapper_node")

        # ── Paramètres ──
        self.declare_parameter("takeoff_altitude",  1.0)
        self.declare_parameter("tracking_altitude", 0.8)
        self.declare_parameter("landing_altitude",  0.12)
        self.declare_parameter("mapping_hover_sec", 3.0)
        self.declare_parameter("linear_gain_xy",    0.5)
        self.declare_parameter("max_xy_speed",      0.2)
        self.declare_parameter("takeoff_speed",     0.3)
        self.declare_parameter("landing_speed",     0.05)
        self.declare_parameter("alt_gain",          1.2)
        self.declare_parameter("max_alt_speed",     0.4)
        self.declare_parameter("image_topic",       "/drone/camera/image_raw")
        self.declare_parameter("cmd_vel_topic",     "/crazyflie/cmd_vel")
        self.declare_parameter("pose_topic",        "/model/crazyflie/pose")
        self.declare_parameter("enable_topic",      "/crazyflie/enable")
        self.declare_parameter("status_topic",      "/drone/mapper/status")

        p = self.get_parameter
        self.takeoff_alt   = p("takeoff_altitude").value
        self.tracking_alt  = p("tracking_altitude").value
        self.landing_alt   = p("landing_altitude").value
        self.hover_sec     = p("mapping_hover_sec").value
        self.kp_xy         = p("linear_gain_xy").value
        self.max_xy        = p("max_xy_speed").value
        self.takeoff_spd   = p("takeoff_speed").value
        self.landing_spd   = p("landing_speed").value
        self.kp_alt        = p("alt_gain").value
        self.max_alt       = p("max_alt_speed").value

        # ── State machine ──
        self.state         = STATE_WAIT
        self._state_start  = None

        # BUG FIX #2 : altitude réelle lue depuis Gazebo (pas open-loop)
        self._real_alt     = 0.0    # m, mis à jour par _pose_cb
        self._pose_received = False

        # Altitude estimée par intégration open-loop (fallback si pas de pose)
        self._estimated_alt  = 0.0
        self._last_cmd_time  = None

        # Image courante OpenCV
        self._last_frame   = None

        # Goal du robot atteint ?
        self._goal_reached = False

        # Bridge ROS ↔ OpenCV
        self.bridge = CvBridge()

        # ── Publishers ──
        cmd_topic = p("cmd_vel_topic").value
        enable_topic = p("enable_topic").value
        self.cmd_pub    = self.create_publisher(Twist, cmd_topic, 10)
        self.enable_pub = self.create_publisher(Bool, enable_topic, 10)
        self.status_pub = self.create_publisher(String, p("status_topic").value, 10)

        # ── Subscribers ──

        # Caméra : BEST_EFFORT pour éviter la saturation réseau
        img_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Image, p("image_topic").value, self._image_cb, img_qos)

        # Pose Gazebo du drone : BEST_EFFORT (topics Gazebo natifs sont BEST_EFFORT)
        pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Pose, p("pose_topic").value, self._pose_cb, pose_qos)

        # BUG FIX #3 : QoS VOLATILE pour correspondre à maze_navigator_node
        nav_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            String, "/maze_navigator/status", self._nav_status_cb, nav_qos)

        # ── Boucle de contrôle à 20 Hz ──
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f"[Drone] DroneMapperNode démarré.\n"
            f"  cmd_vel → {cmd_topic}\n"
            f"  enable  → {enable_topic}\n"
            f"  pose    → {p('pose_topic').value}\n"
            f"  image   → {p('image_topic').value}\n"
            f"  Décollage cible : {self.takeoff_alt:.2f} m | "
            f"Suivi : {self.tracking_alt:.2f} m | "
            f"Atterrissage : {self.landing_alt:.2f} m"
        )

        # Activer le contrôleur MulticopterVelocityControl périodiquement
        # (temps que Gazebo spawne le modèle et initialise le bridge).
        self.create_timer(1.0, self._send_enable_periodic)

    # ──────────────────────────────────────────────────────────
    #  Callbacks
    # ──────────────────────────────────────────────────────────

    def _send_enable_periodic(self):
        """Envoie enable=True au MulticopterVelocityControl."""
        if self.state != STATE_DONE:
            msg = Bool()
            msg.data = True
            self.enable_pub.publish(msg)

    def _image_cb(self, msg: Image):
        try:
            self._last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"[Drone] cv_bridge error: {e}", throttle_duration_sec=5.0)

    def _pose_cb(self, msg: Pose):
        """BUG FIX #2 : altitude réelle depuis la pose Gazebo."""
        self._real_alt = msg.position.z
        if not self._pose_received:
            self._pose_received = True
            self.get_logger().info(
                f"[Drone] Première pose Gazebo reçue : z={self._real_alt:.3f} m")

    def _nav_status_cb(self, msg: String):
        if msg.data == "GOAL_REACHED" and not self._goal_reached:
            self._goal_reached = True
            self.get_logger().info(
                "[Drone] GOAL_REACHED reçu → déclenchement atterrissage.")

    def get_alt(self) -> float:
        """Retourne l'altitude réelle si reçue, sinon l'altitude estimée par intégration."""
        if self._pose_received:
            return self._real_alt
        return self._estimated_alt

    def _pub_cmd(self, vx: float, vy: float, vz: float, wz: float = 0.0):
        import time
        now = time.monotonic()
        if self._last_cmd_time is not None:
            dt = now - self._last_cmd_time
            self._estimated_alt = max(0.0, self._estimated_alt + vz * dt)
        self._last_cmd_time = now

        msg = Twist()
        msg.linear.x  = float(vx)
        msg.linear.y  = float(vy)
        msg.linear.z  = float(vz)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def _pub_status(self, s: str):
        self.status_pub.publish(String(data=s))

    def _detect_aruco_error(self, frame):
        """
        Détecte le marqueur ArUco ID 0 et retourne l'erreur (ex, ey) normalisée
        [-1, 1]². Retourne (None, None) si non détecté.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = _ARUCO_DETECTOR.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            return None, None

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == 0:
                pts = corners[i][0]
                cx_m = float(np.mean(pts[:, 0]))
                cy_m = float(np.mean(pts[:, 1]))
                h, w = frame.shape[:2]
                ex = (cx_m - w / 2.0) / (w / 2.0)
                ey = (cy_m - h / 2.0) / (h / 2.0)
                return ex, ey

        return None, None

    def _time_in_state(self) -> float:
        import time
        if self._state_start is None:
            return 0.0
        return time.monotonic() - self._state_start

    def _enter_state(self, new_state: str):
        import time
        self.state        = new_state
        self._state_start = time.monotonic()
        self.get_logger().info(
            f"[Drone] ── Transition → {new_state.upper()} "
            f"(alt={self.get_alt():.3f} m)")
        self._pub_status(new_state.upper())

    # ──────────────────────────────────────────────────────────
    #  Boucle de contrôle principale
    # ──────────────────────────────────────────────────────────

    def _control_loop(self):
        curr_alt = self.get_alt()

        # ── WAIT : démarrer décollage dès qu'on a le flux caméra ──
        if self.state == STATE_WAIT:
            if self._last_frame is not None or self._time_in_state() > 2.0:
                self.get_logger().info(
                    f"[Drone] Démarrage décollage vers {self.takeoff_alt:.2f} m.")
                self._enter_state(STATE_TAKEOFF)
            else:
                self.get_logger().info(
                    "[Drone] En attente de l'initialisation...",
                    throttle_duration_sec=3.0)
            return

        # ── TAKEOFF : monter jusqu’à takeoff_altitude ──
        if self.state == STATE_TAKEOFF:
            alt_err = self.takeoff_alt - curr_alt
            self.get_logger().info(
                f"[Drone] TAKEOFF alt={curr_alt:.3f} m / cible={self.takeoff_alt:.2f} m "
                f"/ err={alt_err:.3f} m",
                throttle_duration_sec=1.0)

            if abs(alt_err) < 0.08:
                self._pub_cmd(0.0, 0.0, 0.0)
                self.get_logger().info(
                    f"[Drone] Altitude cible atteinte ({curr_alt:.3f} m). "
                    f"Passage en HOVER_MAP pour {self.hover_sec:.1f}s.")
                self._enter_state(STATE_HOVER_MAP)
            else:
                vz = float(np.clip(0.4 * alt_err, -0.2, 0.3))
                self._pub_cmd(0.0, 0.0, vz)
                self._pub_status("TAKEOFF")
            return

        # ── HOVER_MAP : vol stationnaire + observation ──
        if self.state == STATE_HOVER_MAP:
            alt_err = self.takeoff_alt - self._real_alt
            # Deadband large : si err < 0.15m on coupe les moteurs (vitesse nulle)
            if abs(alt_err) < 0.15:
                vz = 0.0
            else:
                vz = float(np.clip(0.2 * alt_err, -0.15, 0.15))
            self._pub_cmd(0.0, 0.0, vz)
            self._pub_status("MAPPING")

            t = self._time_in_state()
            self.get_logger().info(
                f"[Drone] MAPPING ({t:.1f}/{self.hover_sec:.1f}s) "
                f"alt={self._real_alt:.3f} m vz={vz:.3f}",
                throttle_duration_sec=1.0)

            if t >= self.hover_sec:
                self.get_logger().info(
                    f"[Drone] Cartographie terminée ({self.hover_sec:.1f}s). "
                    f"Passage en TRACKING à {self.tracking_alt:.2f} m.")
                self._enter_state(STATE_TRACKING)
            return

        # ── TRACKING : suivi ArUco du robot ──
        if self.state == STATE_TRACKING:
            if self._goal_reached:
                self._enter_state(STATE_LANDING)
                return
            self._pub_status("TRACKING")
            self._do_tracking(target_alt=self.tracking_alt)
            return

        # ── LANDING : descente progressive + centrage ArUco ──
        if self.state == STATE_LANDING:
            self.get_logger().info(
                f"[Drone] LANDING alt={self._real_alt:.3f} m / cible={self.landing_alt:.3f} m",
                throttle_duration_sec=1.0)

            if self._real_alt <= self.landing_alt + 0.02:
                self._pub_cmd(0.0, 0.0, 0.0)
                self.get_logger().info(
                    f"[Drone] Posé sur le robot (alt={self._real_alt:.3f} m). DONE.")
                self._enter_state(STATE_DONE)
                return

            # Descente progressive avec maintien XY
            next_alt = max(self.landing_alt, self._real_alt - self.landing_spd)
            self._pub_status("LANDING")
            self._do_tracking(target_alt=next_alt)
            return

        # ── DONE ──
        if self.state == STATE_DONE:
            self._pub_cmd(0.0, 0.0, 0.0)
            self._pub_status("DONE")

    def _do_tracking(self, target_alt: float):
        """
        Asservissement XY par détection ArUco + contrôle P d'altitude.
        Logs clairs des erreurs et commandes publiées.
        """
        if self._last_frame is None:
            self._pub_cmd(0.0, 0.0, 0.0)
            return

        ex, ey = self._detect_aruco_error(self._last_frame)

        if ex is not None and ey is not None:
            # ex > 0 : marqueur à droite de l'image → drone va +y (à droite dans frame)
            # ey > 0 : marqueur en bas de l'image   → drone va +x (vers l'avant)
            vx = float(np.clip(self.kp_xy * ey, -self.max_xy, self.max_xy))
            vy = float(np.clip(self.kp_xy * ex, -self.max_xy, self.max_xy))
            self.get_logger().info(
                f"[Drone] ArUco détecté (ex={ex:.3f}, ey={ey:.3f}) "
                f"→ cmd vx={vx:.3f} vy={vy:.3f}",
                throttle_duration_sec=0.5)
        else:
            vx, vy = 0.0, 0.0
            self.get_logger().warn(
                f"[Drone] ArUco ID 0 non détecté (alt={self._real_alt:.2f} m) "
                f"— position XY maintenue.",
                throttle_duration_sec=2.0)

        # Contrôle altitude (feedback réel) avec deadband
        alt_err = target_alt - self._real_alt
        if abs(alt_err) < 0.1:
            vz = 0.0  # Dans la zone cible → arrêt vertical
        else:
            vz = float(np.clip(0.3 * alt_err, -0.15, 0.15))
        self.get_logger().info(
            f"[Drone] Alt réelle={self._real_alt:.3f} m / cible={target_alt:.3f} m → vz={vz:.3f}",
            throttle_duration_sec=1.0)

        self._pub_cmd(vx, vy, vz)


def main(args=None):
    rclpy.init(args=args)
    node = DroneMapperNode()

    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
