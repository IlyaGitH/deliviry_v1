#!/usr/bin/env bash
# ============================================================
#  Обёртка запуска узлов ATSD. Ставится в /opt/atsd/run.sh
#
#  Три неочевидных момента, все выявлены при развёртывании:
#
#  1. set -u ломает setup.bash от ROS: скрипты окружения
#     обращаются к незаданным переменным вроде
#     AMENT_TRACE_SETUP_FILES. Отключаем на время source.
#
#  2. gpiozero на Pi 5 не находит фабрику автоматически —
#     задаём lgpio явно.
#
#  3. Cyclone DDS вместо Fast DDS. На связке Pi 5 + wlan0
#     Fast DDS вёл себя нестабильно: узлы то появлялись
#     в ros2 node list, то пропадали, а данные по топикам
#     не доставлялись вовсе. Cyclone работает ровно.
#
#  4. Обнаружение ограничено LOCALHOST. Все узлы живут
#     на одной машине, а мультикаст по вайфаю режется
#     точкой доступа.
# ============================================================
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
