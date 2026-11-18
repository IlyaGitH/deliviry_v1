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
# Fetch dependencies before stopping a working robot; network failure leaves it running.
bash "${SRC_DIR}/atsd_bringup/security/install.sh" --dependencies-only
BACKUP_DIR=$(mktemp -d /opt/atsd/update-backup-XXXXXXXX)
cp -a "${WS_DIR}/src" "${BACKUP_DIR}/src"
systemctl stop atsd.target
bash "${SRC_DIR}/atsd_bringup/scripts/remove_lock_cpu.sh"
rsync -a --delete --exclude __pycache__/ --exclude '*.pyc' "${SRC_DIR}/" "${WS_DIR}/src/"
bash "${WS_DIR}/src/atsd_bringup/scripts/install_vision_model.sh"
chown -R atsd:atsd "${WS_DIR}"
sudo -u atsd bash -c 'set -e; source /opt/ros/jazzy/setup.bash; cd /opt/atsd/ws; colcon build --symlink-install'
install -m 755 "${WS_DIR}/src/atsd_bringup/systemd/run.sh" /opt/atsd/run.sh
install -m 644 "${WS_DIR}/src/atsd_bringup/config/atsd-ros.sh" /etc/profile.d/atsd-ros.sh
install -m 755 "${WS_DIR}/src/atsd_bringup/systemd/wifi.sh" /opt/atsd/wifi.sh
install -m 644 "${WS_DIR}"/src/atsd_bringup/systemd/*.service /etc/systemd/system/
install -m 644 "${WS_DIR}/src/atsd_bringup/systemd/atsd.target" /etc/systemd/system/
install -m 644 "${WS_DIR}/src/atsd_bringup/udev/99-atsd.rules" /etc/udev/rules.d/
udevadm control --reload-rules
bash "${WS_DIR}/src/atsd_bringup/scripts/configure_wifi.sh"
bash "${WS_DIR}/src/atsd_bringup/security/install.sh"
systemctl daemon-reload
systemctl enable NetworkManager atsd-wifi
systemctl start atsd.target
if ! systemctl is-active --quiet atsd-wifi; then
  systemctl start atsd-wifi
fi
echo "Обновление завершено. Предыдущие исходники: ${BACKUP_DIR}/src"
echo 'Известные сети включены, ожидание раздачи 20 секунд. Перезагрузите Pi: текущую SSH-сессию обновление не отключает.'
echo 'Этот скрипт не прошивает VEX. Если его прошивка соответствует репозиторию, повторная загрузка не нужна.'
