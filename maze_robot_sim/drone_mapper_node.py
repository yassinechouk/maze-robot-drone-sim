#!/usr/bin/env python3
"""
drone_mapper_node.py — Le drone Crazyflie en éclaireur : il cartographie le
labyrinthe par vision, transmet la carte au robot, puis le suit jusqu'au but.

Machine à états
---------------
    WAIT ─→ TAKEOFF ─→ FRAME_MAZE ─→ PROCESS_VISION ─→ SEND_MAP
                           ↑               │
                           └───── échec ───┘
    SEND_MAP ─→ RETURN_TO_ROBOT ─→ TRACKING ─→ LANDING ─→ DONE
                        │
                   « READY » part d'ici

`FRAME_MAZE` est l'étape qui rend la cartographie autonome. Le drone décolle
au-dessus du départ, c'est-à-dire dans un *coin* du labyrinthe, et ne sait ni où
se trouve le centre de la scène ni quelle altitude la cadre entièrement. Il
asservit donc sa position sur le barycentre des murs vus et son altitude sur le
taux de remplissage de l'image, jusqu'à ce que le labyrinthe tienne entièrement
dans le champ avec une marge. Aucune dimension n'a besoin d'être fournie.

`RETURN_TO_ROBOT` est ce qui garantit que le suivi commence bien au premier
mouvement du robot. Pour cartographier, le drone a dû s'éloigner : il s'est placé
au centre du labyrinthe et bien plus haut que son altitude de suivi. S'il donnait
le départ depuis là, le robot s'élancerait pendant que le drone est encore à
plusieurs cellules de distance, et le suivi débuterait en retard, en rattrapage.
Le drone revient donc d'abord au-dessus du robot — en visant sa position monde
relevée pendant la montée, puis en affinant à l'asservissement visuel dès que le
marqueur redevient lisible — et ne publie `READY` qu'une fois stabilisé sur lui.
Le robot ne bouge pas avant, puisqu'il attend ce signal.

Topics
------
  Souscrit :
    <image_topic>            (sensor_msgs/Image)      caméra nadir
    <pose_topic>             (geometry_msgs/Pose)     pose Gazebo du drone
    /maze_navigator/status   (std_msgs/String)        statut du robot
  Publie :
    <cmd_vel_topic>          (geometry_msgs/Twist)    vitesse drone
    <enable_topic>           (std_msgs/Bool)          activation du contrôleur de vol
    /maze/occupancy_grid     (nav_msgs/OccupancyGrid) carte, latchée
    /maze/start_pose         (geometry_msgs/PoseStamped) départ, latché
    /maze/goal_pose          (geometry_msgs/PoseStamped) arrivée, latchée
    /drone/mapper/ready      (std_msgs/String)        « READY », latché
    /drone/mapper/status     (std_msgs/String)        état interne
"""
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from tf_transformations import euler_from_quaternion

from . import maze_map, maze_vision

STATE_WAIT           = "wait"
STATE_TAKEOFF        = "takeoff"
STATE_FRAME_MAZE     = "frame_maze"
STATE_PROCESS_VISION = "process_vision"
STATE_SEND_MAP       = "send_map"
STATE_RETURN         = "return_to_robot"
STATE_TRACKING       = "tracking"
STATE_LANDING        = "landing"
STATE_DONE           = "done"


def latched_qos(depth=1):
    """QoS « dernier message conservé » : un abonné tardif reçoit quand même la carte."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class DroneMapperNode(Node):

    def __init__(self):
        super().__init__("drone_mapper_node")

        self.declare_parameter("mapping_altitude",      2.5)
        self.declare_parameter("mapping_altitude_min",  1.2)
        self.declare_parameter("mapping_altitude_max",  8.0)
        self.declare_parameter("tracking_height_above_robot", 1.0)
        self.declare_parameter("landing_altitude",      0.12)
        self.declare_parameter("frame_fill_min",        0.45)
        self.declare_parameter("frame_fill_max",        0.80)
        self.declare_parameter("frame_center_tol",      0.05)
        self.declare_parameter("frame_settle_sec",      1.5)
        self.declare_parameter("frame_timeout_sec",     60.0)
        self.declare_parameter("return_center_tol",     0.08)
        self.declare_parameter("return_settle_sec",     1.0)
        self.declare_parameter("return_timeout_sec",    40.0)
        self.declare_parameter("position_gain_xy",      0.8)
        self.declare_parameter("vision_samples",        5)
        self.declare_parameter("vision_min_agreement",  0.6)
        self.declare_parameter("vision_max_attempts",   40)
        self.declare_parameter("camera_hfov",           1.047)
        self.declare_parameter("camera_z_offset",       0.02)
        self.declare_parameter("wall_height",           0.15)
        self.declare_parameter("max_tilt_rad",          0.10)
        self.declare_parameter("linear_gain_xy",        0.5)
        self.declare_parameter("max_xy_speed",          0.2)
        self.declare_parameter("frame_gain_xy",         0.9)
        self.declare_parameter("landing_speed",         0.05)
        self.declare_parameter("image_topic",           "/drone/camera/image_raw")
        self.declare_parameter("cmd_vel_topic",         "/crazyflie/cmd_vel")
        self.declare_parameter("pose_topic",            "/model/crazyflie/pose")
        self.declare_parameter("enable_topic",          "/crazyflie/enable")
        self.declare_parameter("status_topic",          "/drone/mapper/status")
        self.declare_parameter("map_frame",             "map")

        p = self.get_parameter
        self.map_alt        = float(p("mapping_altitude").value)
        self.map_alt_min    = float(p("mapping_altitude_min").value)
        self.map_alt_max    = float(p("mapping_altitude_max").value)
        # Le suivi est spécifié par rapport au *robot*, pas au sol : c'est la
        # distance drone-robot qui détermine la taille du marqueur dans l'image,
        # donc la qualité de l'asservissement. Le pont du robot est à 8,6 cm.
        self.track_height   = float(p("tracking_height_above_robot").value)
        self.tracking_alt   = self.track_height + maze_vision.DEFAULT_MARKER_HEIGHT
        self.landing_alt    = float(p("landing_altitude").value)
        self.fill_min       = float(p("frame_fill_min").value)
        self.fill_max       = float(p("frame_fill_max").value)
        self.center_tol     = float(p("frame_center_tol").value)
        self.settle_sec     = float(p("frame_settle_sec").value)
        self.frame_timeout  = float(p("frame_timeout_sec").value)
        self.return_tol     = float(p("return_center_tol").value)
        self.return_settle  = float(p("return_settle_sec").value)
        self.return_timeout = float(p("return_timeout_sec").value)
        self.kp_pos         = float(p("position_gain_xy").value)
        self.n_samples      = int(p("vision_samples").value)
        self.min_agreement  = float(p("vision_min_agreement").value)
        self.max_attempts   = int(p("vision_max_attempts").value)
        self.hfov           = float(p("camera_hfov").value)
        self.cam_dz         = float(p("camera_z_offset").value)
        self.wall_height    = float(p("wall_height").value)
        self.max_tilt       = float(p("max_tilt_rad").value)
        self.kp_xy          = float(p("linear_gain_xy").value)
        self.max_xy         = float(p("max_xy_speed").value)
        self.kp_frame       = float(p("frame_gain_xy").value)
        self.landing_spd    = float(p("landing_speed").value)
        self.map_frame      = str(p("map_frame").value)

        self.state        = STATE_WAIT
        self._state_start = None

        self._pose = None                # (x, y, z, roulis, tangage) réels du drone
        self._pose_received = False
        self._last_frame = None
        self._last_frame_pose = None     # pose associée à l'image courante
        self._focal_px = None

        # Position monde du marqueur ArUco, relevée le plus bas possible : à
        # l'altitude de cartographie le marqueur de 12 cm est souvent illisible.
        self._robot_world = None

        self._observations = []
        self._vision_attempts = 0
        self._centered_since = None
        self._over_robot_since = None
        self._goal_reached = False
        self._map_published = False

        self.bridge = CvBridge()

        cmd_topic    = str(p("cmd_vel_topic").value)
        enable_topic = str(p("enable_topic").value)
        self.cmd_pub    = self.create_publisher(Twist,  cmd_topic,    10)
        self.enable_pub = self.create_publisher(Bool,   enable_topic, 10)
        self.status_pub = self.create_publisher(String, str(p("status_topic").value), 10)
        self.ready_pub  = self.create_publisher(String, "/drone/mapper/ready", latched_qos())
        self.map_pub    = self.create_publisher(OccupancyGrid, "/maze/occupancy_grid", latched_qos())
        self.start_pub  = self.create_publisher(PoseStamped, "/maze/start_pose", latched_qos())
        self.goal_pub   = self.create_publisher(PoseStamped, "/maze/goal_pose",  latched_qos())

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, str(p("image_topic").value), self._image_cb, sensor_qos)
        self.create_subscription(Pose, str(p("pose_topic").value), self._pose_cb, sensor_qos)
        self.create_subscription(String, "/maze_navigator/status", self._nav_status_cb, 10)

        self.create_timer(0.05, self._control_loop)
        self.create_timer(1.0, self._send_enable_periodic)

        self.get_logger().info(
            f"[Drone] Cartographe démarré. Cartographie visée à {self.map_alt:.2f} m "
            f"(ajustée automatiquement entre {self.map_alt_min:.1f} et {self.map_alt_max:.1f} m), "
            f"suivi à {self.track_height:.2f} m au-dessus du robot "
            f"(altitude {self.tracking_alt:.2f} m).")

    # ──────────────────────────────────────────────────────────
    #  Callbacks
    # ──────────────────────────────────────────────────────────

    def _send_enable_periodic(self):
        if self.state != STATE_DONE:
            self.enable_pub.publish(Bool(data=True))

    def _image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"[Drone] cv_bridge : {e}", throttle_duration_sec=5.0)
            return
        self._last_frame = frame
        # La pose est figée avec l'image : toute la géométrie de la vision
        # suppose que les deux décrivent le même instant.
        self._last_frame_pose = self._pose
        if self._focal_px is None:
            self._focal_px = maze_vision.focal_px_from_hfov(frame.shape[1], self.hfov)
            self.get_logger().info(
                f"[Drone] Caméra {frame.shape[1]}x{frame.shape[0]}, "
                f"HFOV {math.degrees(self.hfov):.1f}° → focale {self._focal_px:.1f} px")
        self._track_robot(frame)

    def _pose_cb(self, msg: Pose):
        q = msg.orientation
        roll, pitch, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._pose = (msg.position.x, msg.position.y, msg.position.z, roll, pitch)
        if not self._pose_received:
            self._pose_received = True
            self.get_logger().info(
                f"[Drone] Première pose Gazebo : "
                f"({self._pose[0]:.2f}, {self._pose[1]:.2f}, {self._pose[2]:.2f})")

    def _nav_status_cb(self, msg: String):
        if msg.data == "GOAL_REACHED" and not self._goal_reached:
            self._goal_reached = True
            self.get_logger().info("[Drone] GOAL_REACHED reçu → atterrissage.")

    def _track_robot(self, frame):
        """Mémorise la position monde du robot dès qu'elle est lisible."""
        if self._last_frame_pose is None or self._focal_px is None:
            return
        if self.state not in (STATE_WAIT, STATE_TAKEOFF, STATE_FRAME_MAZE):
            return
        nx, ny, alt, tilt = self._camera_ground_frame(self._last_frame_pose)
        if alt <= 0.2 or tilt > self.max_tilt:
            return
        w = maze_vision.locate_robot_world(frame, nx, ny, alt, self._focal_px)
        if w is not None:
            first = self._robot_world is None
            self._robot_world = w
            if first:
                self.get_logger().info(
                    f"[Drone] Robot localisé à ({w[0]:.3f}, {w[1]:.3f}) "
                    f"depuis {alt:.2f} m.")

    # ──────────────────────────────────────────────────────────
    #  Utilitaires
    # ──────────────────────────────────────────────────────────

    def _alt(self):
        return (self._pose[2] - self.cam_dz) if self._pose else 0.0

    def _camera_ground_frame(self, pose):
        """
        (nadir_x, nadir_y, altitude, assiette) associés à une pose du drone.

        Le nadir apparent tient compte de l'assiette : sans cette correction la
        carte publiée est translatée de plusieurs centimètres dans le repère
        monde, du simple fait des un ou deux degrés de tangage résiduel d'un
        stationnaire.
        """
        x, y, z, roll, pitch = pose
        alt = z - self.cam_dz
        nx, ny = maze_vision.nadir_ground_point(x, y, alt, roll, pitch)
        return nx, ny, alt, math.hypot(roll, pitch)

    def _pub_cmd(self, vx, vy, vz, wz=0.0):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z = (
            float(vx), float(vy), float(vz), float(wz))
        self.cmd_pub.publish(msg)

    def _pub_status(self, s):
        self.status_pub.publish(String(data=s))

    def _time_in_state(self):
        return 0.0 if self._state_start is None else time.monotonic() - self._state_start

    def _enter_state(self, new_state):
        self.state = new_state
        self._state_start = time.monotonic()
        self.get_logger().info(
            f"[Drone] ── {new_state.upper()} (alt={self._alt():.2f} m)")
        self._pub_status(new_state.upper())

    def _altitude_cmd(self, target, deadband=0.06, gain=0.6, limit=0.35):
        err = target - self._alt()
        if abs(err) < deadband:
            return 0.0, True
        return float(np.clip(gain * err, -limit, limit)), False

    # ──────────────────────────────────────────────────────────
    #  Boucle de contrôle
    # ──────────────────────────────────────────────────────────

    def _control_loop(self):
        if self.state == STATE_WAIT:
            if self._last_frame is not None and self._pose_received:
                self._enter_state(STATE_TAKEOFF)
            elif self._time_in_state() > 15.0:
                self.get_logger().error(
                    "[Drone] Ni image ni pose après 15 s — vérifiez le pont ros_gz.")
                self._state_start = time.monotonic()
            else:
                self.get_logger().info("[Drone] Initialisation…", throttle_duration_sec=3.0)
            return

        if self.state == STATE_TAKEOFF:
            vz, reached = self._altitude_cmd(self.map_alt, deadband=0.08)
            self._pub_cmd(0.0, 0.0, vz)
            self._pub_status("TAKEOFF")
            self.get_logger().info(
                f"[Drone] Montée {self._alt():.2f} → {self.map_alt:.2f} m",
                throttle_duration_sec=1.0)
            if reached:
                self._centered_since = None
                self._enter_state(STATE_FRAME_MAZE)
            return

        if self.state == STATE_FRAME_MAZE:
            self._do_framing()
            return

        if self.state == STATE_PROCESS_VISION:
            self._do_vision()
            return

        if self.state == STATE_SEND_MAP:
            # La carte est publiée, mais le robot ne part pas encore : le drone
            # est au centre du labyrinthe et bien trop haut pour le suivre.
            self._over_robot_since = None
            self._enter_state(STATE_RETURN)
            return

        if self.state == STATE_RETURN:
            self._do_return()
            return

        if self.state == STATE_TRACKING:
            if self._goal_reached:
                self._enter_state(STATE_LANDING)
                return
            self._pub_status("TRACKING")
            vz, _ = self._altitude_cmd(self.tracking_alt, deadband=0.10,
                                       gain=0.3, limit=0.15)
            vx, vy = self._aruco_velocity()
            self._pub_cmd(vx, vy, vz)
            return

        if self.state == STATE_LANDING:
            self._pub_status("LANDING")
            if self._alt() <= self.landing_alt + 0.02:
                self._pub_cmd(0.0, 0.0, 0.0)
                self.get_logger().info(f"[Drone] Posé (alt={self._alt():.3f} m).")
                self._enter_state(STATE_DONE)
                return
            vx, vy = self._aruco_velocity()
            # Descente à vitesse constante : une bande morte sur l'altitude
            # absorberait les derniers centimètres et le drone resterait suspendu.
            self._pub_cmd(vx, vy, -self.landing_spd)
            return

        if self.state == STATE_DONE:
            self.enable_pub.publish(Bool(data=False))
            self._pub_cmd(0.0, 0.0, 0.0)
            self._pub_status("DONE")

    def _aruco_error(self):
        """
        Écart normalisé [-1, 1]² entre le centre de l'image et le marqueur du
        robot, ou None si le marqueur n'est pas lu.
        """
        if self._last_frame is None:
            return None
        px = maze_vision._detect_aruco_center(self._last_frame)
        if px is None:
            return None
        h, w = self._last_frame.shape[:2]
        return ((px[0] - w / 2.0) / (w / 2.0), (px[1] - h / 2.0) / (h / 2.0))

    def _aruco_velocity(self):
        """Asservissement visuel XY sur le marqueur du robot."""
        err = self._aruco_error()
        if err is None:
            self.get_logger().warn("[Drone] ArUco perdu — position maintenue.",
                                   throttle_duration_sec=2.0)
            return 0.0, 0.0
        ex, ey = err
        # u croît vers +y monde, v croît vers +x monde (cf. maze_vision).
        return (float(np.clip(self.kp_xy * ey, -self.max_xy, self.max_xy)),
                float(np.clip(self.kp_xy * ex, -self.max_xy, self.max_xy)))

    # ──────────────────────────────────────────────────────────
    #  Retour au-dessus du robot
    # ──────────────────────────────────────────────────────────

    def _do_return(self):
        """
        Ramène le drone à `tracking_height_above_robot` au-dessus du robot, puis
        donne le départ.

        Deux repères se relaient. De haut, le marqueur de 12 cm est trop petit
        pour être lu : le drone vise alors la position monde du robot, relevée
        pendant la montée et toujours valable puisque le robot attend le signal
        pour bouger. En descendant, le marqueur redevient lisible et
        l'asservissement visuel prend le relais — plus précis, et surtout
        insensible à une éventuelle erreur sur la position mémorisée.

        `READY` n'est publié qu'une fois la hauteur atteinte *et* le drone
        stabilisé au-dessus du marqueur. Le donner plus tôt ferait démarrer le
        robot alors que le drone est encore à plusieurs cellules de là : le suivi
        commencerait en rattrapage, ce qui est exactement ce qu'on veut éviter.
        """
        self._pub_status("RETURN")
        vz, alt_ok = self._altitude_cmd(self.tracking_alt, deadband=0.08)

        err = self._aruco_error()
        if err is not None:
            ex, ey = err
            vx = float(np.clip(self.kp_xy * ey, -self.max_xy, self.max_xy))
            vy = float(np.clip(self.kp_xy * ex, -self.max_xy, self.max_xy))
            over_robot = math.hypot(ex, ey) < self.return_tol
            source = "ArUco"
        elif self._robot_world is not None and self._pose is not None:
            dx = self._robot_world[0] - self._pose[0]
            dy = self._robot_world[1] - self._pose[1]
            vx = float(np.clip(self.kp_pos * dx, -self.max_xy, self.max_xy))
            vy = float(np.clip(self.kp_pos * dy, -self.max_xy, self.max_xy))
            # Sans le marqueur on ne se déclare jamais arrivé : la position
            # mémorisée sert à s'approcher, pas à conclure.
            over_robot = False
            source = f"position monde (reste {math.hypot(dx, dy):.2f} m)"
        else:
            vx = vy = 0.0
            over_robot = False
            source = "aucun repère"

        self._pub_cmd(vx, vy, vz)
        self.get_logger().info(
            f"[Drone] Retour au robot via {source} — alt={self._alt():.2f}/"
            f"{self.tracking_alt:.2f} m, au-dessus={over_robot}",
            throttle_duration_sec=1.0)

        # Une image sans marqueur est une absence d'information, pas une preuve
        # que le drone a quitté sa position : elle ne remet donc pas le compteur
        # à zéro. Seule une lecture franchement décentrée le fait. Sans cette
        # nuance, les quelques pour cent d'images manquées suffiraient à empêcher
        # le critère d'aboutir, et le départ ne serait jamais donné qu'au délai.
        seen = err is not None
        if not alt_ok or (seen and not over_robot):
            self._over_robot_since = None
        elif seen and self._over_robot_since is None:
            self._over_robot_since = time.monotonic()

        if (self._over_robot_since is not None
                and time.monotonic() - self._over_robot_since >= self.return_settle):
            self._give_go("en position")
            return

        if self._time_in_state() > self.return_timeout:
            # Le robot ne doit jamais rester bloqué à cause du drone : si le
            # retour n'aboutit pas, on donne quand même le départ et le suivi
            # se fera en rattrapage plutôt que pas du tout.
            self._give_go("délai dépassé, départ donné malgré tout")

    def _give_go(self, reason):
        self.ready_pub.publish(String(data="READY"))
        self.get_logger().info(
            f"[Drone] À {self.track_height:.2f} m au-dessus du robot "
            f"({reason}) — READY envoyé, passage en suivi.")
        self._enter_state(STATE_TRACKING)

    # ──────────────────────────────────────────────────────────
    #  Cadrage autonome
    # ──────────────────────────────────────────────────────────

    def _do_framing(self):
        """
        Centre le labyrinthe dans l'image et ajuste l'altitude jusqu'à ce qu'il
        y tienne entièrement avec de la marge.

        Le remplissage est mesuré sur la boîte englobante des murs : trop grand,
        il manque des bords hors champ ; trop petit, la résolution par cellule
        s'effondre et les murs fins deviennent illisibles.
        """
        if self._last_frame is None:
            self._pub_cmd(0.0, 0.0, 0.0)
            return
        self._pub_status("FRAMING")

        frame = self._last_frame
        h, w = frame.shape[:2]
        bbox = maze_vision.maze_bounds_px(frame)

        if bbox is None:
            # Rien de rouge en vue : prendre de la hauteur élargit le champ.
            vz, _ = self._altitude_cmd(min(self.map_alt + 0.5, self.map_alt_max))
            self.map_alt = min(self.map_alt + 0.02, self.map_alt_max)
            self._pub_cmd(0.0, 0.0, max(vz, 0.15))
            self.get_logger().warn("[Drone] Labyrinthe hors champ — montée.",
                                   throttle_duration_sec=2.0)
            self._centered_since = None
            return

        x0, y0, x1, y1 = bbox
        cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
        ex = (cx - w / 2.0) / (w / 2.0)
        ey = (cy - h / 2.0) / (h / 2.0)
        fill = max((x1 - x0) / w, (y1 - y0) / h)
        touches_border = x0 < 2 or y0 < 2 or x1 > w - 3 or y1 > h - 3

        # Le remplissage varie comme l'inverse de l'altitude : une seule règle
        # de trois donne directement la bonne altitude, là où un pas fixe
        # oscille longuement autour de la cible parce que la montée traîne
        # derrière la consigne.
        fill_target = 0.5 * (self.fill_min + self.fill_max)
        if touches_border:
            # Boîte tronquée par le bord : le remplissage mesuré est sous-estimé
            # et ne peut pas servir de mesure — il faut simplement monter.
            self.map_alt = min(max(self.map_alt, self._alt()) * 1.15 + 0.05,
                               self.map_alt_max)
        elif not (self.fill_min <= fill <= self.fill_max):
            self.map_alt = float(np.clip(self._alt() * fill / fill_target,
                                         self.map_alt_min, self.map_alt_max))

        vz, alt_ok = self._altitude_cmd(self.map_alt)
        vx = float(np.clip(self.kp_frame * ey, -self.max_xy, self.max_xy))
        vy = float(np.clip(self.kp_frame * ex, -self.max_xy, self.max_xy))
        self._pub_cmd(vx, vy, vz)

        centered = (abs(ex) < self.center_tol and abs(ey) < self.center_tol
                    and not touches_border
                    and self.fill_min <= fill <= self.fill_max and alt_ok)
        self.get_logger().info(
            f"[Drone] Cadrage : err=({ex:+.3f},{ey:+.3f}) remplissage={fill:.2f} "
            f"alt={self._alt():.2f}/{self.map_alt:.2f} m centré={centered}",
            throttle_duration_sec=1.0)

        if not centered:
            self._centered_since = None
        elif self._centered_since is None:
            self._centered_since = time.monotonic()
        elif time.monotonic() - self._centered_since >= self.settle_sec:
            # Immobiliser le drone avant de photographier : la moindre vitesse
            # décorrèle l'image de la pose utilisée pour la géométrie.
            self._pub_cmd(0.0, 0.0, 0.0)
            self._observations = []
            self._vision_attempts = 0
            self._enter_state(STATE_PROCESS_VISION)
            return

        if self._time_in_state() > self.frame_timeout:
            self.get_logger().warn(
                "[Drone] Cadrage non stabilisé dans le temps imparti — "
                "extraction tentée telle quelle.")
            self._observations = []
            self._vision_attempts = 0
            self._enter_state(STATE_PROCESS_VISION)

    # ──────────────────────────────────────────────────────────
    #  Extraction et publication de la carte
    # ──────────────────────────────────────────────────────────

    def _do_vision(self):
        """
        Accumule plusieurs extractions puis publie la carte consensuelle.

        Une image isolée suffit rarement : un mur peut être manqué à cause d'un
        reflet ou d'un pixel de bruit. Le vote sur plusieurs images élimine ces
        accidents, et rater un mur coûte bien plus cher qu'en inventer un.
        """
        self._pub_cmd(0.0, 0.0, 0.0)
        self._pub_status("MAPPING")

        if self._last_frame is None or self._last_frame_pose is None:
            return

        self._vision_attempts += 1
        nx, ny, alt, tilt = self._camera_ground_frame(self._last_frame_pose)
        if tilt > self.max_tilt:
            # Au-delà de quelques degrés d'assiette la projection n'est plus
            # assimilable à une homothétie : l'image est déformée, pas seulement
            # décalée. Autant attendre que le drone se stabilise.
            self.get_logger().warn(
                f"[Drone] Assiette de {math.degrees(tilt):.1f}° — image écartée.",
                throttle_duration_sec=1.0)
            return
        try:
            obs = maze_vision.extract_maze(
                self._last_frame, nx, ny, alt, self._focal_px,
                wall_height=self.wall_height,
                start_world=self._robot_world)
            self._observations.append(obs)
            self.get_logger().info(
                f"[Drone] Extraction {len(self._observations)}/{self.n_samples} : {obs}")
        except maze_vision.MazeVisionError as e:
            self.get_logger().warn(
                f"[Drone] Extraction refusée ({self._vision_attempts}) : {e}",
                throttle_duration_sec=1.0)

        if len(self._observations) >= self.n_samples:
            self._finalize_map()
            return

        if self._vision_attempts >= self.max_attempts:
            if len(self._observations) >= 2:
                self.get_logger().warn(
                    f"[Drone] Seulement {len(self._observations)} extractions valides — "
                    "publication sur cet échantillon réduit.")
                self._finalize_map()
            else:
                self.get_logger().error(
                    "[Drone] Extraction impossible — retour au cadrage.")
                self.map_alt = min(self.map_alt * 1.08, self.map_alt_max)
                self._centered_since = None
                self._enter_state(STATE_FRAME_MAZE)

    def _finalize_map(self):
        try:
            obs = maze_vision.vote_observations(self._observations, self.min_agreement)
        except maze_vision.MazeVisionError as e:
            self.get_logger().error(f"[Drone] Consensus impossible : {e} — recadrage.")
            self._centered_since = None
            self._enter_state(STATE_FRAME_MAZE)
            return

        path = maze_map.astar(obs.walls, obs.rows, obs.cols, obs.start_cell, obs.goal_cell)
        if not path:
            # Une carte sans chemin est forcément fausse : le labyrinthe réel est
            # résoluble par construction. Mieux vaut réobserver que publier ça.
            self.get_logger().error(
                f"[Drone] Carte extraite non résoluble de {obs.start_cell} à "
                f"{obs.goal_cell} — nouvelle observation.")
            self._observations = []
            self._vision_attempts = 0
            self._centered_since = None
            self._enter_state(STATE_FRAME_MAZE)
            return

        self._publish_map(obs)
        self.get_logger().info(
            f"[Drone] Carte publiée : {obs.rows}x{obs.cols}, cellule "
            f"{obs.cell_size*100:.1f} cm, départ {obs.start_cell}, arrivée "
            f"{obs.goal_cell}, chemin de {len(path)} cellules "
            f"(consensus sur {obs.diagnostics['votes']}/{obs.diagnostics['total']} images).")
        self._log_grid(obs)
        self._enter_state(STATE_SEND_MAP)

    def _publish_map(self, obs):
        data, width, height = maze_map.walls_to_grid_data(obs.walls, obs.rows, obs.cols)
        ox, oy = maze_map.grid_origin(obs.cell_size, obs.origin_x, obs.origin_y)

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame
        grid.info.resolution = obs.cell_size / maze_map.SUB_RESOLUTION
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = ox
        grid.info.origin.position.y = oy
        grid.info.origin.orientation.w = 1.0
        grid.data = [int(v) for v in data]
        self.map_pub.publish(grid)

        def pose_of(cell):
            wx, wy = maze_map.cell_center_world(cell[0], cell[1], obs.rows, obs.cols,
                                                obs.cell_size, obs.origin_x, obs.origin_y)
            msg = PoseStamped()
            msg.header = grid.header
            msg.pose.position.x = wx
            msg.pose.position.y = wy
            msg.pose.orientation.w = 1.0
            return msg

        self.start_pub.publish(pose_of(obs.start_cell))
        self.goal_pub.publish(pose_of(obs.goal_cell))
        self._map_published = True

    def _log_grid(self, obs):
        """Trace la grille extraite en ASCII, pour comparer d'un coup d'œil."""
        lines = []
        for r in range(obs.rows):
            top = "".join("+---" if obs.walls[r][c] & maze_map.N else "+   "
                          for c in range(obs.cols)) + "+"
            mid = ""
            for c in range(obs.cols):
                mid += "|" if obs.walls[r][c] & maze_map.W else " "
                cell = (r, c)
                mid += " S " if cell == obs.start_cell else (
                       " G " if cell == obs.goal_cell else "   ")
            mid += "|" if obs.walls[r][obs.cols - 1] & maze_map.E else " "
            lines += [top, mid]
        lines.append("".join("+---" if obs.walls[obs.rows - 1][c] & maze_map.S else "+   "
                             for c in range(obs.cols)) + "+")
        self.get_logger().info("[Drone] Grille extraite :\n" + "\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = DroneMapperNode()
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
