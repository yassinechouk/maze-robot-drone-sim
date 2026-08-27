#!/usr/bin/env python3
"""
Launch principal : génère le monde labyrinthe SDF, démarre Gazebo Harmonic,
spawn le robot différentiel avec ros2_control, ponte IMU/horloge/caméra via
ros_gz_bridge, démarre les contrôleurs (joint_state_broadcaster +
diff_drive_controller), et démarre le nœud de navigation A*.

Arguments drone (tous optionnels, drone désactivé par défaut) :
  spawn_drone:=true      Active le spawn du drone Crazyflie
  drone_x:=0.0          Position X de spawn du drone
  drone_y:=<start_y>    Position Y de spawn du drone (défaut = position Y du robot)
  drone_z:=1.0          Altitude de spawn du drone

Topics ajoutés avec spawn_drone:=true :
  /drone/camera/image_raw  (sensor_msgs/Image)   — caméra top-down du drone
  /drone/cmd_vel           (geometry_msgs/Twist)  — commandes au drone
  /drone/mapper/status     (std_msgs/String)       — état du drone_mapper_node
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

    # ── 8. Nœud de navigation A* ──
    navigator_node = Node(
        package="maze_robot_sim",
        executable="maze_navigator_node",
        output="screen",
        parameters=[{
            "maze_json":               generated_meta_path,
            "odom_topic":              "/diff_drive_controller/odom",
            "cmd_vel_topic":           "/diff_drive_controller/cmd_vel",
            "imu_topic":               "/imu",
            "invert_angular":          invert_angular,
            "use_sim_time":            True,
            # ── Gains de navigation ajustés pour un parcours smooth ──
            "linear_kp":              0.5,    # était 0.9 — décélère plus tôt
            "angular_kp":             1.2,    # était 1.8 — virages plus doux
            "max_linear_speed":       0.18,   # était 0.3 — vitesse max réduite
            "max_angular_speed":      1.4,    # était 2.0 — rotation plus lente
            "goal_tolerance":         0.10,   # 10cm : agit comme un "lookahead" pur pursuit pour enchaîner les cellules de façon fluide
            "align_tolerance_rad":    0.20,   # était 0.35 — commence à avancer plus tôt/précisément
        }],
    )

    # ── 9. Drone mapper + tracker node (conditionnel) ──
    drone_mapper_node = Node(
        package="maze_robot_sim",
        executable="drone_mapper_node",
        output="screen",
        parameters=[{
            "use_sim_time":       True,
            "image_topic":        "/drone/camera/image_raw",
            "cmd_vel_topic":      "/crazyflie/cmd_vel",
            "enable_topic":       "/crazyflie/enable",
            "pose_topic":         "/model/crazyflie/pose",
            "takeoff_altitude":   1.0,
            "tracking_altitude":  0.8,
            "landing_altitude":   0.12,
            "mapping_hover_sec":  3.0,
            "linear_gain_xy":     0.5,
            "max_xy_speed":       0.2,
            "takeoff_speed":      0.3,
            "landing_speed":      0.05,
            "alt_gain":           1.2,
            "max_alt_speed":      0.4,
        }],
        condition=IfCondition(spawn_drone),
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
        # Drone mapper (lance indépendamment dès le départ)
        drone_mapper_node,
    ])
