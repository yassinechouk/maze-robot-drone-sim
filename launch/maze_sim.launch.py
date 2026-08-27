#!/usr/bin/env python3
"""
Launch principal : génère le monde labyrinthe SDF, démarre Gazebo Harmonic,
spawn le robot différentiel avec ros2_control et le drone Crazyflie, ponte
IMU/horloge/caméra via ros_gz_bridge, démarre les contrôleurs
(joint_state_broadcaster + diff_drive_controller), puis les deux nœuds
autonomes.

Le drone est indispensable : c'est lui qui cartographie le labyrinthe et
transmet la carte. Le robot ne lit aucun fichier et reste à l'arrêt tant qu'il
n'a pas reçu de carte — lancer avec spawn_drone:=false ne fait donc rien avancer.

Arguments principaux :
  rows / cols / cell_size  Dimensions du labyrinthe *généré* (le drone, lui, les
                           redécouvre par vision : elles ne lui sont pas fournies)
  seed:=<N>                Fixe le labyrinthe (aléatoire par défaut)
  validate_map:=true       Lance maze_map_validator, qui compare la carte vue par
                           le drone à la grille réellement construite
  mapping_altitude:=<m>    Altitude initiale de cartographie ; le drone l'ajuste
                           ensuite tout seul jusqu'à cadrer tout le labyrinthe

Topics ajoutés par la chaîne de cartographie :
  /maze/occupancy_grid      (nav_msgs/OccupancyGrid)  carte extraite, latchée
  /maze/start_pose          (geometry_msgs/PoseStamped)
  /maze/goal_pose           (geometry_msgs/PoseStamped)
  /maze_navigator/path      (nav_msgs/Path)           chemin A* du robot
  /maze_navigator/diagnostics (std_msgs/String)       vitesse, jeu aux murs, MPC
"""
import os
import random
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("maze_robot_sim")

    # ── Configurations ──
    rows           = LaunchConfiguration("rows")
    cols           = LaunchConfiguration("cols")
    cell_size      = LaunchConfiguration("cell_size")
    seed           = LaunchConfiguration("seed")
    world_file     = LaunchConfiguration("world_file")
    generate_world = LaunchConfiguration("generate_world")
    y_spawn        = LaunchConfiguration("y_spawn")
    x_spawn        = LaunchConfiguration("x_spawn")
    invert_angular = LaunchConfiguration("invert_angular")
    spawn_drone    = LaunchConfiguration("spawn_drone")
    drone_x        = LaunchConfiguration("drone_x")
    drone_y        = LaunchConfiguration("drone_y")
    drone_z        = LaunchConfiguration("drone_z")
    validate_map      = LaunchConfiguration("validate_map")
    mapping_altitude  = LaunchConfiguration("mapping_altitude")

    # ── Arguments ──
    declare_rows = DeclareLaunchArgument("rows", default_value="4")
    declare_cols = DeclareLaunchArgument("cols", default_value="4")
    declare_cell_size = DeclareLaunchArgument("cell_size", default_value="0.4")
    # Seed aléatoire à chaque lancement (peut être fixé via seed:=<N>)
    declare_seed = DeclareLaunchArgument("seed", default_value=str(random.randint(1, 100000)))
    declare_generate_world = DeclareLaunchArgument("generate_world", default_value="true")
    declare_world_file = DeclareLaunchArgument(
        "world_file",
        default_value=os.path.join(pkg_share, "worlds", "generated_maze.sdf"),
    )
    declare_x_spawn = DeclareLaunchArgument("x_spawn", default_value="0.0")
    declare_y_spawn = DeclareLaunchArgument(
        "y_spawn",
        default_value=PythonExpression(['str((int(', rows, ') - 1) * float(', cell_size, '))'])
    )
    declare_invert_angular = DeclareLaunchArgument("invert_angular", default_value="")

    # Arguments drone
    declare_spawn_drone = DeclareLaunchArgument(
        "spawn_drone", default_value="true",
        description="Spawn the Crazyflie drone above the start (true/false)"
    )
    declare_drone_x = DeclareLaunchArgument("drone_x", default_value="0.0")
    declare_drone_y = DeclareLaunchArgument(
        "drone_y",
        default_value=PythonExpression(['str((int(', rows, ') - 1) * float(', cell_size, '))'])
    )
    declare_drone_z = DeclareLaunchArgument("drone_z", default_value="0.1")
    declare_validate_map = DeclareLaunchArgument(
        "validate_map", default_value="false",
        description="Compare la carte vue par le drone à la grille réellement générée"
    )
    declare_mapping_altitude = DeclareLaunchArgument(
        "mapping_altitude", default_value="2.5",
        description="Altitude initiale de cartographie ; ajustée ensuite par le drone"
    )

    # ── Chemins ──
    generated_world_path    = os.path.join(pkg_share, "worlds", "generated_maze.sdf")
    generated_meta_path     = os.path.join(pkg_share, "worlds", "generated_maze.json")
    controller_params_path  = os.path.join(pkg_share, "config", "diff_drive_controller.yaml")

    # ── 1. Génération du monde labyrinthe ──
    generate_world_cmd = ExecuteProcess(
        cmd=[
            sys.executable, "-m", "maze_robot_sim.maze_world_generator",
            "--rows", rows,
            "--cols", cols,
            "--cell-size", cell_size,
            "--seed", seed,
            "--out", generated_world_path,
        ],
        output="screen",
        condition=IfCondition(generate_world),
    )

    # ── 2. Robot state publisher ──
    robot_xacro_file = os.path.join(pkg_share, "description", "maze_robot.urdf.xacro")
    robot_description_content = ParameterValue(
        Command(["xacro ", robot_xacro_file]), value_type=str
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": True,
        }],
    )

    # Robot state publisher du drone (séparé, namespace /drone)
    drone_xacro_file = os.path.join(pkg_share, "description", "crazyflie.urdf.xacro")
    drone_sdf_file = os.path.join(pkg_share, "description", "crazyflie_model.sdf")
    drone_description_content = ParameterValue(
        Command(["xacro ", drone_xacro_file]), value_type=str
    )
    drone_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="drone",
        output="screen",
        parameters=[{
            "robot_description": drone_description_content,
            "use_sim_time": True,
        }],
        condition=IfCondition(spawn_drone),
    )

    # ── 3. Gazebo Harmonic ──
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [world_file, " -r -v 3"]}.items(),
    )
    start_gz_after_world_gen = RegisterEventHandler(
        OnProcessExit(target_action=generate_world_cmd, on_exit=[gz_sim_launch]),
        condition=IfCondition(generate_world),
    )
    gz_sim_launch_direct = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [world_file, " -r -v 3"]}.items(),
        condition=UnlessCondition(generate_world),
    )

    # ── 4. Spawn robot au sol ──
    log_spawn_coords = LogInfo(
        msg=["[DIAGNOSTIC] Spawn robot : x=", x_spawn, " y=", y_spawn]
    )
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "maze_robot",
            "-topic", "robot_description",
            "-x", x_spawn,
            "-y", y_spawn,
            "-z", "0.05",
            "-allow_renaming", "false",
        ],
        output="screen",
    )
    diagnostic_pose_check = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=["gz", "model", "-m", "maze_robot", "-p"],
                output="screen",
                name="diagnostic_pose_check",
            ),
        ],
    )

    # ── 5. Spawn du drone Crazyflie (conditionnel) ──
    spawn_drone_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "crazyflie",
            "-file", drone_sdf_file,
            "-x", drone_x,
            "-y", drone_y,
            "-z", drone_z,
            "-allow_renaming", "false",
        ],
        output="screen",
        condition=IfCondition(spawn_drone),
    )

    # ── 6. Bridge ROS 2 ↔ Gazebo ──
    # Bridge de base (IMU + horloge pour le robot)
    gz_bridge_base = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge_base",
        arguments=[
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Bridge drone : caméra (Gazebo → ROS) + cmd_vel (ROS → Gazebo)
    # BUG FIX #1 : MulticopterVelocityControl écoute sur le topic Gazebo
    # interne /model/crazyflie/cmd_vel (nom scopé au modèle). Le bridge doit
    # faire : ROS /drone/cmd_vel  →  Gazebo /model/crazyflie/cmd_vel
    # La syntaxe ros_gz_bridge est : ros_topic@ros_type@gz_topic]gz_type
    # mais parameter_bridge ne supporte pas le remapping de topic dans le même
    # argument. On utilise donc le topic Gazebo exact directement en ROS côté drone :
    # côté ROS on publie sur /model/crazyflie/cmd_vel pour simplifier.
    # drone_mapper_node publie sur cmd_vel_topic param = /model/crazyflie/cmd_vel.
    gz_bridge_drone = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge_drone",
        arguments=[
            # Caméra top-down drone : Gazebo → ROS 2
            "/drone/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            # Commandes vitesse drone : ROS 2 → Gazebo
            "/crazyflie/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            # Pose du drone depuis Gazebo → ROS 2 (pour feedback altitude réel)
            "/model/crazyflie/pose@geometry_msgs/msg/Pose[gz.msgs.Pose",
            # Enable/Disable du plugin MulticopterVelocityControl : ROS 2 → Gazebo
            "/crazyflie/enable@std_msgs/msg/Bool]gz.msgs.Boolean",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(spawn_drone),
    )

    # ── 7. Contrôleurs ros2_control ──
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--param-file", controller_params_path,
        ],
        output="screen",
    )

    # ── 8. Nœud de navigation (carte reçue du drone + MPC) ──
    navigator_node = Node(
        package="maze_robot_sim",
        executable="maze_navigator_node",
        output="screen",
        parameters=[{
            "odom_topic":                 "/diff_drive_controller/odom",
            "cmd_vel_topic":              "/diff_drive_controller/cmd_vel",
            "imu_topic":                  "/imu",
            "map_topic":                  "/maze/occupancy_grid",
            "invert_angular":             invert_angular,
            "use_sim_time":               True,
            # ── Commande prédictive ──
            # Horizon de 1,8 s (18 pas x 0,1 s) : à 0,30 m/s le robot anticipe
            # environ 1,5 cellule, de quoi voir venir un virage.
            "mpc_horizon":                18,
            "mpc_dt":                     0.10,
            "mpc_decimation":             2,      # résolution à 10 Hz, commande à 20 Hz
            # Plafond de temps de résolution : le nœud est mono-thread, une
            # résolution qui s'éternise fige aussi la lecture de l'odométrie.
            "mpc_max_solve_time":         0.15,
            "max_linear_speed":           0.30,
            # Dynamique angulaire volontairement douce : les pivots rapides
            # font patiner les roues, et le patinage se paie en dérive
            # d'odométrie, donc en écart entre là où le robot se croit et où il
            # est réellement. Passer de 1,4 à 1,0 rad/s a divisé cette dérive
            # par 1,5 et amélioré le jeu résiduel aux murs.
            "max_angular_speed":          1.0,
            "max_acceleration":           0.6,
            "max_angular_acceleration":   1.5,
            # Marge de pénalité = demi-largeur de couloir : toute sortie de l'axe
            # devient coûteuse, ce qui est nécessaire avec 2,3 cm de jeu réel.
            "robot_radius":               0.177,
            "safety_margin":              0.023,
            "wall_weight":                25000.0,
            "align_weight":               45.0,
            "nominal_speed":              0.30,
            "goal_tolerance":             0.05,
        }],
    )

    # ── 9. Drone cartographe + tracker ──
    drone_mapper_node = Node(
        package="maze_robot_sim",
        executable="drone_mapper_node",
        output="screen",
        parameters=[{
            "use_sim_time":          True,
            "image_topic":           "/drone/camera/image_raw",
            "cmd_vel_topic":         "/crazyflie/cmd_vel",
            "enable_topic":          "/crazyflie/enable",
            "pose_topic":            "/model/crazyflie/pose",
            # Altitude de départ seulement : le cadrage la corrige tout seul
            # jusqu'à ce que le labyrinthe tienne entièrement dans l'image.
            "mapping_altitude":      mapping_altitude,
            "mapping_altitude_min":  1.2,
            "mapping_altitude_max":  8.0,
            # Hauteur de suivi exprimée par rapport au robot : c'est la
            # distance drone-robot qui fixe la taille du marqueur dans l'image.
            # Le drone ne donne le départ qu'une fois revenu à cette hauteur
            # au-dessus du robot, sans quoi le suivi démarrerait en rattrapage.
            "tracking_height_above_robot": 1.0,
            "return_center_tol":     0.08,
            "return_settle_sec":     1.0,
            "return_timeout_sec":    40.0,
            "landing_altitude":      0.12,
            "frame_fill_min":        0.45,
            "frame_fill_max":        0.80,
            "frame_center_tol":      0.05,
            "frame_settle_sec":      1.5,
            # Vote sur plusieurs images : un mur manqué sur une image isolée
            # ferait planifier un chemin qui traverse une cloison.
            "vision_samples":        5,
            "vision_min_agreement":  0.6,
            "camera_hfov":           1.047,
            "wall_height":           0.15,
            "linear_gain_xy":        0.5,
            "max_xy_speed":          0.2,
            "landing_speed":         0.05,
        }],
        condition=IfCondition(spawn_drone),
    )

    warn_no_drone = LogInfo(
        msg=("[ATTENTION] spawn_drone:=false — le robot attend une carte que "
             "personne ne publiera et ne bougera pas."),
        condition=UnlessCondition(spawn_drone),
    )

    # ── 10. Validation de la carte (diagnostic, hors chaîne de navigation) ──
    map_validator_node = Node(
        package="maze_robot_sim",
        executable="maze_map_validator",
        output="screen",
        parameters=[{
            "use_sim_time":   True,
            "reference_json": generated_meta_path,
        }],
        condition=IfCondition(validate_map),
    )

    # ── Séquencement ──
    # Robot : spawn → JSB → DDC → Navigator
    start_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot_node, on_exit=[joint_state_broadcaster_spawner])
    )
    start_ddc_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[diff_drive_controller_spawner])
    )
    start_navigator_after_ddc = RegisterEventHandler(
        OnProcessExit(target_action=diff_drive_controller_spawner, on_exit=[navigator_node])
    )

    return LaunchDescription([
        # Arguments
        declare_rows,
        declare_cols,
        declare_cell_size,
        declare_seed,
        declare_generate_world,
        declare_world_file,
        declare_x_spawn,
        declare_y_spawn,
        declare_invert_angular,
        declare_spawn_drone,
        declare_drone_x,
        declare_drone_y,
        declare_drone_z,
        declare_validate_map,
        declare_mapping_altitude,
        # Génération monde
        generate_world_cmd,
        # Publishers description
        robot_state_publisher_node,
        drone_state_publisher_node,
        # Gazebo
        start_gz_after_world_gen,
        gz_sim_launch_direct,
        # Spawn
        log_spawn_coords,
        spawn_robot_node,
        spawn_drone_node,
        diagnostic_pose_check,
        # Bridges
        gz_bridge_base,
        gz_bridge_drone,
        # Contrôleurs + navigation (séquencés)
        start_jsb_after_spawn,
        start_ddc_after_jsb,
        start_navigator_after_ddc,
        # Drone cartographe (démarre indépendamment dès le départ)
        drone_mapper_node,
        warn_no_drone,
        map_validator_node,
    ])
