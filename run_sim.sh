#!/usr/bin/env bash
# Script d'exécution directe pour le projet maze_robot_sim

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Nettoyage des anciennes instances Gazebo fantômes en arrière-plan
echo "Nettoyage des anciennes sessions Gazebo..."
pkill -9 -f "gz sim" >/dev/null 2>&1 || true
sleep 1

# Sourcing de ROS 2 et du workspace local
source /opt/ros/jazzy/setup.bash
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
else
    echo "Compilation du package..."
    colcon build --symlink-install
    source "$SCRIPT_DIR/install/setup.bash"
fi

echo "Lancement de la simulation maze_robot_sim..."
ros2 launch maze_robot_sim maze_sim.launch.py "$@"
