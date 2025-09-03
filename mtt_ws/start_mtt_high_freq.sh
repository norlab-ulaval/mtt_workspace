#!/bin/bash

# Script de démarrage MTT-154 haute performance
# Usage: ./start_mtt_high_freq.sh [frequency_hz]

FREQUENCY=${1:-400.0}

echo "🚀 Démarrage MTT-154 à ${FREQUENCY}Hz..."
echo "   - Interface CAN: vcan0 (mode test)"
echo "   - Fréquence de contrôle: ${FREQUENCY}Hz"
echo ""

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash

# Lancer le système MTT avec la fréquence spécifiée
ros2 launch mtt_driver mtt_composable_system.launch.py \
    test_mode:=true \
    control_frequency_hz:=${FREQUENCY} \
    driver_log_level:=INFO
