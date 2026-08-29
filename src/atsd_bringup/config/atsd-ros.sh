# ============================================================
#  АТСД-1М · окружение ROS 2 для всех пользователей
#  Ставится в /etc/profile.d/atsd-ros.sh
#
#  Подключается из ~/.bashrc строкой:
#     [ -f /etc/profile.d/atsd-ros.sh ] && . /etc/profile.d/atsd-ros.sh
#
#  Через сам profile.d не срабатывает: ssh запускает
#  интерактивную оболочку, а не login-shell.
# ============================================================

if [ -f /opt/ros/jazzy/setup.bash ]; then
  set +u
  . /opt/ros/jazzy/setup.bash
  [ -f /opt/atsd/ws/install/setup.bash ] && . /opt/atsd/ws/install/setup.bash
fi

export ROS_DOMAIN_ID=17
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GPIOZERO_PIN_FACTORY=lgpio
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
