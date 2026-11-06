#!/usr/bin/env bash
# Обновление уже установленного робота из текущей копии репозитория (без git pull).
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo 'Запустите через sudo bash update.sh'; exit 1; }
SRC_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
WS_DIR=/opt/atsd/ws
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo 'Нет ROS 2 Jazzy'; exit 1; }
[[ "${SRC_DIR}" != "${WS_DIR}/src" ]] || { echo 'Запускайте из отдельной копии репозитория, например ~/atsd_ws'; exit 1; }
id atsd >/dev/null
for package in atsd_drive atsd_sensors atsd_perception atsd_actuators atsd_web atsd_bringup; do
  [[ -f "${SRC_DIR}/${package}/package.xml" ]] || { echo "Нет пакета ${package}"; exit 1; }
done
BACKUP_DIR=$(mktemp -d /opt/atsd/update-backup-XXXXXXXX)
cp -a "${WS_DIR}/src" "${BACKUP_DIR}/src"
systemctl stop atsd.target
rsync -a --delete --exclude __pycache__/ --exclude '*.pyc' "${SRC_DIR}/" "${WS_DIR}/src/"
chown -R atsd:atsd "${WS_DIR}"
sudo -u atsd bash -c 'set -e; source /opt/ros/jazzy/setup.bash; cd /opt/atsd/ws; colcon build --symlink-install'
install -m 755 "${WS_DIR}/src/atsd_bringup/systemd/run.sh" /opt/atsd/run.sh
install -m 755 "${WS_DIR}/src/atsd_bringup/systemd/wifi.sh" /opt/atsd/wifi.sh
install -m 644 "${WS_DIR}"/src/atsd_bringup/systemd/*.service /etc/systemd/system/
install -m 644 "${WS_DIR}/src/atsd_bringup/systemd/atsd.target" /etc/systemd/system/
install -m 644 "${WS_DIR}/src/atsd_bringup/udev/99-atsd.rules" /etc/udev/rules.d/
udevadm control --reload-rules
systemctl daemon-reload
systemctl enable NetworkManager atsd-wifi
systemctl start atsd.target
if ! systemctl is-active --quiet atsd-wifi; then
  systemctl start atsd-wifi
fi
echo "Обновление завершено. Предыдущие исходники: ${BACKUP_DIR}/src"
echo 'Уже работающий сервис Wi-Fi использует старый скрипт до перезагрузки Pi.'
echo 'Для исправлений привода загрузите на VEX актуальный firmware/vex_v5/src/main.cpp из этой же версии репозитория.'
