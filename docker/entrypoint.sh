#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

# CycloneDDS configuration
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONE_IFACE="${CYCLONE_IFACE:-enP8p1s0}"
export GO2_IP="${GO2_IP:-192.168.123.161}"

# Generate CycloneDDS config from template
if [ -f /ros2_ws/cyclonedds.xml.template ]; then
    envsubst '${CYCLONE_IFACE} ${GO2_IP}' < /ros2_ws/cyclonedds.xml.template > /ros2_ws/cyclonedds.xml
fi
if [ -f /ros2_ws/cyclonedds.xml ]; then
    export CYCLONEDDS_URI=file:///ros2_ws/cyclonedds.xml
fi

# Auto-enable cmd_vel when Nav2 is active (user can still override with explicit ENABLE_CMD_VEL=false)
if [ "${NAV2:-false}" = "true" ] && [ -z "${ENABLE_CMD_VEL+x}" ]; then
    export ENABLE_CMD_VEL=true
    echo "[entrypoint] Auto-enabled ENABLE_CMD_VEL because NAV2=true"
fi

echo "=== Go2 CycloneDDS Driver ==="
echo "  RMW: ${RMW_IMPLEMENTATION}"
echo "  CYCLONEDDS_URI: ${CYCLONEDDS_URI:-not set}"
echo "  CYCLONE_IFACE: ${CYCLONE_IFACE}"
echo "  GO2_IP: ${GO2_IP}"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
echo "  NAV2: ${NAV2:-false}"
echo "  ENABLE_CMD_VEL: ${ENABLE_CMD_VEL:-false}"
echo "==========================="

exec "$@"
