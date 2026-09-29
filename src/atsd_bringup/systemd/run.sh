#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source /opt/atsd/ws/install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GPIOZERO_PIN_FACTORY=lgpio
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

exec "$@"
