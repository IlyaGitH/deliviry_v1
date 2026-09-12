#!/usr/bin/env bash
# ============================================================
#  АТСД-1М · развёртывание на Ubuntu 24.04 Server (arm64)
#  Raspberry Pi 5 · ROS 2 Jazzy Jalisco
#
#  Запуск:
#     sudo bash install.sh
#     sudo bash install.sh --skip-upgrade          без full-upgrade
#     sudo bash install.sh --skip-ros              без установки ROS
#     sudo bash install.sh --ros-key=/path/ros.key ключ из файла
#
#  Идемпотентен: можно прогонять повторно.
# ============================================================
set -eo pipefail

ROS_DISTRO_NAME=jazzy
ATSD_USER=atsd
ATSD_HOME=/opt/atsd
WS_DIR="${ATSD_HOME}/ws"
KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg
ROS_KEY_URL=https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

SKIP_ROS=0
SKIP_UPGRADE=0
ROS_KEY_FILE=""

for arg in "$@"; do
  case "$arg" in
    --skip-ros)     SKIP_ROS=1 ;;
    --skip-upgrade) SKIP_UPGRADE=1 ;;
    --ros-key=*)    ROS_KEY_FILE="${arg#*=}" ;;
    *) echo "Неизвестный аргумент: $arg"; exit 1 ;;
  esac
done

export DEBIAN_FRONTEND=noninteractive

log()  { echo -e "\n\033[1;36m== $* \033[0m"; }
warn() { echo -e "\033[1;33m!! $* \033[0m"; }
die()  { echo -e "\033[1;31mXX $* \033[0m"; exit 1; }

[[ $EUID -eq 0 ]] || die "Нужен root: sudo bash install.sh"

# ─────────────────────────────────────────── 0. проверки
. /etc/os-release
[[ "${UBUNTU_CODENAME:-}" == "noble" ]] || \
  warn "Ожидалась Ubuntu 24.04 noble, обнаружено: ${UBUNTU_CODENAME:-неизвестно}"

log "Исходники: ${SRC_DIR}"
for p in atsd_drive atsd_actuators atsd_sensors atsd_bringup; do
  [[ -d "${SRC_DIR}/${p}" ]] || \
    die "Нет пакета ${p}. Запускайте скрипт из src/atsd_bringup/scripts"
  [[ -f "${SRC_DIR}/${p}/setup.cfg" ]] || \
    die "Нет ${p}/setup.cfg — без него узлы не соберутся в исполняемые"
done

# ─────────────────────────────────────────── 1. репозитории
# Образы Ubuntu для Raspberry иногда приезжают без веток
# noble-updates и noble-backports. Тогда apt ловит
#   bzip2 : Depends: libbz2-1.0 but it is not installable
# — базовый релиз и security расходятся по версиям.
log "Проверяю список репозиториев"
SOURCES=/etc/apt/sources.list.d/ubuntu.sources
if [[ -f ${SOURCES} ]]; then
  if ! grep -q 'noble-updates' "${SOURCES}"; then
    warn "Нет ветки noble-updates — добавляю"
    cp "${SOURCES}" "${SOURCES}.bak.$(date +%s)"
    sed -i '0,/^Suites: noble$/s//Suites: noble noble-updates noble-backports/' "${SOURCES}"
  fi
  grep -q 'universe' "${SOURCES}" || \
    warn "В Components нет universe — часть пакетов будет недоступна"
else
  warn "${SOURCES} не найден, пропускаю проверку"
fi

apt-get update

if [[ ${SKIP_UPGRADE} -eq 0 ]]; then
  log "Обновляю систему"
  apt-get full-upgrade -y || {
    warn "full-upgrade споткнулся, чиню зависимости"
    apt-get --fix-broken install -y
    apt-get full-upgrade -y
  }
fi

# ─────────────────────────────────────────── 2. базовые пакеты
apt_install() {
  log "Ставлю: $*"
  if ! apt-get install -y "$@"; then
    warn "Повторяю после --fix-broken"
    apt-get --fix-broken install -y
    apt-get update
    apt-get install -y "$@" || die "Не удалось поставить: $*"
  fi
}

apt_install curl gnupg ca-certificates lsb-release rsync \
            build-essential git cmake \
            python3-pip python3-venv \
            python3-serial python3-gpiozero python3-lgpio \
            i2c-tools

log "Отключаю ModemManager"
# Перехватывает ttyACM на 10-20 секунд при подключении
# и мешает мосту с V5 стартовать.
systemctl disable --now ModemManager 2>/dev/null || true

# ─────────────────────────────────────────── 3. ROS 2 Jazzy
if [[ ${SKIP_ROS} -eq 0 ]]; then
  if [[ ! -d /opt/ros/${ROS_DISTRO_NAME} ]]; then

    if [[ -n "${ROS_KEY_FILE}" ]]; then
      log "Беру ключ ROS из файла ${ROS_KEY_FILE}"
      [[ -s "${ROS_KEY_FILE}" ]] || die "Файл ключа пуст или не найден"
      install -m 644 "${ROS_KEY_FILE}" "${KEYRING}"
    elif [[ -s "${KEYRING}" ]]; then
      log "Ключ ROS уже установлен"
    else
      log "Качаю ключ ROS"
      # Без --max-time curl висит молча, если raw.githubusercontent.com
      # недоступен. Качаем во временный файл, чтобы не затереть
      # рабочий ключ пустышкой.
      if curl -fsSL --max-time 25 --retry 2 "${ROS_KEY_URL}" -o /tmp/ros.key \
         && [[ -s /tmp/ros.key ]]; then
        install -m 644 /tmp/ros.key "${KEYRING}"
      else
        die "Не скачался ключ ROS. Скачайте на машине с доступом:
    curl -sSL ${ROS_KEY_URL} -o ros.key
    scp ros.key пользователь@адрес-пи:~
и запустите:  sudo bash install.sh --ros-key=/home/пользователь/ros.key"
      fi
    fi

    echo "deb [arch=$(dpkg --print-architecture) signed-by=${KEYRING}] \
http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
      > /etc/apt/sources.list.d/ros2.list
    apt-get update

    log "Ставлю ROS 2 ${ROS_DISTRO_NAME} — это надолго, полтора гигабайта"
    apt_install ros-${ROS_DISTRO_NAME}-ros-base \
                ros-dev-tools \
                python3-colcon-common-extensions \
                ros-${ROS_DISTRO_NAME}-tf2-ros \
                ros-${ROS_DISTRO_NAME}-v4l2-camera \
                ros-${ROS_DISTRO_NAME}-image-transport-plugins \
                ros-${ROS_DISTRO_NAME}-diagnostic-updater \
                ros-${ROS_DISTRO_NAME}-rmw-cyclonedds-cpp
  else
    log "ROS 2 ${ROS_DISTRO_NAME} уже стоит"
  fi
fi

log "Python-зависимости"
pip3 install --break-system-packages --upgrade \
     pynmea2 smbus2 fastapi 'uvicorn[standard]' pydantic || \
  warn 'Часть Python-зависимостей не поставилась — проверьте вручную'

# ─────────────────────────────────────────── 4. пользователь
if ! id -u "${ATSD_USER}" >/dev/null 2>&1; then
  log "Создаю пользователя ${ATSD_USER}"
  useradd --system --create-home --home-dir "${ATSD_HOME}" \
          --shell /usr/sbin/nologin "${ATSD_USER}"
fi
for g in gpio i2c; do
  getent group "$g" >/dev/null || groupadd "$g"
done
usermod -aG dialout,video,gpio,i2c,plugdev "${ATSD_USER}"
mkdir -p "${ATSD_HOME}"
chown "${ATSD_USER}:${ATSD_USER}" "${ATSD_HOME}"

# ─────────────────────────────────────────── 5. конфиг платы
log "Настраиваю config.txt"
CONFIG=/boot/firmware/config.txt
if [[ -f ${CONFIG} ]]; then
  add_cfg() { grep -qxF "$1" "${CONFIG}" || echo "$1" >> "${CONFIG}"; }
  add_cfg 'enable_uart=1'             # ttyAMA0 на GPIO14/15 — лидар LD19
  add_cfg 'dtoverlay=disable-bt'      # на Pi 5 не обязательно, но не мешает
  add_cfg 'dtoverlay=uart2'           # ttyAMA2 на GPIO4/5 — GNSS BN-880
  add_cfg 'dtparam=i2c_arm=on'        # гейдж X1202 и аудиоплеер
  add_cfg 'usb_max_current_enable=1'  # снимает лимит 600 мА на USB
else
  warn "${CONFIG} не найден — правьте вручную"
fi
# Консоль на последовательном порту съедает поток лидара: 230400 бод
# уходят в getty, а узел видит пустой порт.
for tty in ttyAMA0 ttyAMA2 serial0; do
  systemctl disable --now "serial-getty@${tty}.service" 2>/dev/null || true
done
if [[ -f /boot/firmware/cmdline.txt ]] && grep -q 'console=serial0' /boot/firmware/cmdline.txt; then
  warn "В cmdline.txt осталась console=serial0 — уберите, иначе лидар молчит"
fi

# ─────────────────────────────────────────── 6. сборка
log "Копирую исходники в ${WS_DIR}"
mkdir -p "${WS_DIR}/src"
rsync -a --delete \
      --exclude 'build/' --exclude 'install/' --exclude 'log/' \
      --exclude '__pycache__/' --exclude '.git/' \
      "${SRC_DIR}/" "${WS_DIR}/src/"
chown -R "${ATSD_USER}:${ATSD_USER}" "${ATSD_HOME}"

if [[ -d /opt/ros/${ROS_DISTRO_NAME} ]]; then
  log "Собираю workspace"
  sudo -u "${ATSD_USER}" bash -lc "
    set -eo pipefail
    set +u; source /opt/ros/${ROS_DISTRO_NAME}/setup.bash; set -u
    cd ${WS_DIR}
    rm -rf build install log
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  " || die "Сборка не прошла. Смотрите ${WS_DIR}/log/latest_build"

  # Контроль: без setup.cfg colcon молча не создаёт эти файлы,
  # и launch падает с "libexec directory does not exist"
  for f in atsd_drive/lib/atsd_drive/v5_bridge_node \
           atsd_actuators/lib/atsd_actuators/light_node \
           atsd_actuators/lib/atsd_actuators/lock_node \
           atsd_sensors/lib/atsd_sensors/lidar_node \
           atsd_sensors/lib/atsd_sensors/gnss_node \
           atsd_sensors/lib/atsd_sensors/battery_node \
           atsd_web/lib/atsd_web/web_node; do
    [[ -f "${WS_DIR}/install/${f}" ]] || die "Не собрался исполняемый: ${f}"
  done
  log "Все узлы собрались"
else
  warn "ROS не найден — сборка пропущена"
fi

# ─────────────────────────────────────────── 7. сервисы и udev
log "Ставлю сервисы и правила"
install -m 755 "${WS_DIR}/src/atsd_bringup/systemd/run.sh" "${ATSD_HOME}/run.sh"
install -m 644 "${WS_DIR}"/src/atsd_bringup/systemd/*.service /etc/systemd/system/
install -m 644 "${WS_DIR}/src/atsd_bringup/systemd/atsd.target" /etc/systemd/system/
install -m 644 "${WS_DIR}/src/atsd_bringup/udev/99-atsd.rules" /etc/udev/rules.d/

# ── общее окружение ROS для всех пользователей ──
mkdir -p /etc/atsd
install -m 644 "${WS_DIR}/src/atsd_bringup/config/atsd-ros.sh"  /etc/profile.d/atsd-ros.sh

# profile.d не читается интерактивной ssh-оболочкой, поэтому
# подключаем его строкой в bashrc — новым и существующим пользователям.
HOOK='[ -f /etc/profile.d/atsd-ros.sh ] && . /etc/profile.d/atsd-ros.sh'
grep -qxF "${HOOK}" /etc/skel/.bashrc 2>/dev/null || echo "${HOOK}" >> /etc/skel/.bashrc
for h in /home/*; do
  [[ -f "$h/.bashrc" ]] || continue
  grep -qxF "${HOOK}" "$h/.bashrc" || echo "${HOOK}" >> "$h/.bashrc"
done

# Каталог рабочего пространства доступен на чтение всем:
# иначе source /opt/atsd/ws/install/setup.bash отдаёт Permission denied
chmod -R a+rX "${ATSD_HOME}"

# Доступ к GPIO и I2C без root: на Ubuntu 24.04 они только для root
cat > /etc/udev/rules.d/98-gpio.rules <<'EOF'
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"
SUBSYSTEM=="i2c-dev", GROUP="i2c", MODE="0660"
EOF

udevadm control --reload-rules && udevadm trigger
systemctl daemon-reload
mkdir -p "${ATSD_HOME}/data"
chown -R "${ATSD_USER}:${ATSD_USER}" "${ATSD_HOME}/data"

systemctl enable atsd.target atsd-drive atsd-actuators atsd-sensors atsd-web

# ─────────────────────────────────────────── 8. итог
log "Готово"
cat <<MSG

Дальше по порядку:

  1. Перезагрузиться — правки config.txt применяются после ребута,
     и заодно подхватятся группы пользователя atsd:
       sudo reboot

  2. Проверить актуаторы, они не зависят от USB-устройств:
       sudo systemctl start atsd-actuators
       journalctl -u atsd-actuators -n 20 --no-pager -l
     В логе должно быть:
       [light_node] Свет: фары GPIO18, лента GPIO13, потолки 0.60 / 0.50
       [lock_node]  Замок: GPIO19, импульс 0.4 с

  3. Подключить брейн и проверить устройства:
       lsusb
       udevadm info -a -n /dev/ttyACM1 | grep -E 'idVendor|idProduct|bInterfaceNumber' | head
     Подставить ID брейна в /etc/udev/rules.d/99-atsd.rules, затем:
       sudo udevadm control --reload-rules && sudo udevadm trigger
       ls -la /dev/v5 /dev/lidar /dev/gnss
     Лидар и GNSS сидят на аппаратных UART, их симлинки уже готовы:
       /dev/lidar -> ttyAMA0 (GPIO15, пин 10, 230400)
       /dev/gnss  -> ttyAMA2 (GPIO4/5, пины 7 и 29)

  4. Проверить сенсоры по отдельности:
       source ${ATSD_HOME}/run.sh
       ros2 run atsd_sensors lidar_node --ros-args -p port:=/dev/lidar
       ros2 topic hz /scan          # ждём около 10 Гц
       ros2 run atsd_sensors gnss_node --ros-args -p port:=/dev/gnss
       ros2 topic echo /gnss/status # sats=... hdop=...

  5. Запустить всё:
       sudo systemctl start atsd.target
     Веб-интерфейс: http://<адрес-пи>:8080
     Проверить без робота:
       ros2 launch atsd_web web.launch.py mock:=true
       ros2 topic list

Сторонние драйверы лидара не нужны: узел atsd_sensors/lidar_node
разбирает протокол LD19 сам.

MSG
