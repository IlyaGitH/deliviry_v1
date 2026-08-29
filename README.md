# АТСД-1М · нижняя подсистема

Пакеты ROS 2 для автономного транспортного средства доставки грузов.
Формируют интерфейс, поверх которого строится верхний уровень: SLAM,
Nav2, детектор дорожных знаков, веб-приложение заказов.

**Платформа:** Raspberry Pi 5 · Ubuntu 24.04 Server arm64 · **ROS 2 Jazzy**

> Именно Jazzy. Humble привязан к Ubuntu 22.04 и на 24.04 не ставится.

---

## Состав

| Пакет | Что делает |
|---|---|
| `atsd_drive` | Мост с VEX V5 Brain по USB-serial: одометрия, TF, телеметрия, приём `cmd_vel` |
| `atsd_actuators` | Фары и лента на ШИМ-каналах, соленоидный замок грузового отсека |
| `atsd_sensors` | Драйвер GNSS BN-880 с разбором NMEA |
| `atsd_bringup` | Launch-файлы, общий конфиг, systemd-сервисы, udev-правила, установщик |

Лидар и камера работают штатными драйверами `ydlidar_ros2_driver`
и `v4l2_camera` — своё писать незачем, параметры вынесены в общий конфиг.

---

## Интерфейс для верхнего уровня

Namespace нет, топики плоские: так их без прослоек подхватывают Nav2
и `robot_localization`. Собственных сообщений тоже нет — только
стандартные типы, чтобы не тащить CMake-пакет с msg.

### Публикуется

| Топик | Тип | Частота | Назначение |
|---|---|---|---|
| `/drive/odom` | `nav_msgs/Odometry` | 50 Гц | Одометрия по энкодерам шести моторов |
| `/drive/battery` | `sensor_msgs/BatteryState` | 50 Гц | Напряжение батареи V5 |
| `/drive/estop` | `std_msgs/Bool` | 50 Гц | Аварийная кнопка нажата |
| `/drive/link` | `std_msgs/Bool` | 50 Гц | Связь с брейном жива |
| `/scan` | `sensor_msgs/LaserScan` | 7 Гц | Лидар YDLIDAR X2L |
| `/camera/image_raw` | `sensor_msgs/Image` | 30 Гц | Камера, MJPEG |
| `/gnss/fix` | `sensor_msgs/NavSatFix` | 5 Гц | Координаты |
| `/gnss/status` | `std_msgs/String` | 5 Гц | Спутники, HDOP, качество |
| `/lights/state` | `std_msgs/String` | 1 Гц | Состояние света |
| `/lock/busy` | `std_msgs/Bool` | 5 Гц | Соленоид под током |
| `/lock/closed` | `std_msgs/Bool` | 5 Гц | Крышка закрыта |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Гц | Агрегированное состояние |
| TF | `odom → base_link` | 50 Гц | + статические на лидар, камеру, GNSS |

### Принимается

| Топик или сервис | Тип | Назначение |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Скорость. Nav2 пишет сюда напрямую |
| `/lights/headlights` | `std_msgs/Float32` | Яркость фар, 0.0–1.0 |
| `/lights/strip` | `std_msgs/Float32` | Яркость ленты, 0.0–1.0 |
| `/lights/strip_mode` | `std_msgs/String` | `off` · `solid` · `blink` · `beacon` |
| `/lock/unlock` | `std_srvs/Trigger` | Импульс на замок |

---

## Три решения, зашитых в архитектуру

**Токовый интерлок.** Шина 12 В тянет около 1,4 А, а соленоид замка один
берёт 1,1 А. Поэтому `lock_node` на время импульса публикует `/lock/busy`,
а `light_node` по этому сигналу гасит свет. Без интерлока просадка питания
уронит Raspberry ровно в момент выдачи груза.

**Двойной watchdog.** Прошивка V5 сама тормозит моторы, если команда
не приходила 300 мс. Узел моста дублирует это со своей стороны, отправляя
нули по истечении `cmd_timeout_s`. Механизмы независимы: один на нижнем
уровне, второй на верхнем.

**Честный NO_FIX.** В помещении спутников нет, и узел GNSS публикует
`STATUS_NO_FIX` вместо повтора последних координат. Иначе EKF потянет
робота к призрачной точке.

---

## Установка

```bash
git clone https://github.com/IlyaGitH/deliviry_v1 ~/atsd_ws
chmod +x src/atsd_bringup/scripts/install.sh src/atsd_bringup/systemd/run.sh
cd ~/atsd_ws/src/atsd_bringup/scripts
sudo bash install.sh
sudo reboot
```

Скрипт ставит ROS 2 Jazzy, создаёт системного пользователя `atsd`,
собирает workspace в `/opt/atsd/ws`, прописывает сервисы и udev-правила,
освобождает аппаратный UART под GNSS и снимает лимит 600 мА на USB.
Идемпотентен — повторный запуск безопасен.

**Флаги:**

| Флаг | Когда нужен |
|---|---|
| `--skip-upgrade` | Система уже обновлена, экономит десять минут |
| `--skip-ros` | ROS ставится отдельно |
| `--ros-key=ФАЙЛ` | `raw.githubusercontent.com` недоступен, ключ скачан вручную |

### Если недоступен GitHub

Ключ ROS качается с `raw.githubusercontent.com`, который из России
часто не открывается. На машине с доступом:

```bash
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o ros.key
scp ros.key пользователь@адрес-пи:~
```

Затем на Raspberry:

```bash
sudo bash install.sh --ros-key=/home/пользователь/ros.key
```

Сам `packages.ros.org` при этом обычно доступен напрямую.

### Лидар ставится отдельно

Драйвера YDLIDAR нет в apt:

```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK
cd YDLidar-SDK && mkdir build && cd build && cmake .. && make && sudo make install
cd /opt/atsd/ws/src && sudo -u atsd git clone https://github.com/YDLIDAR/ydlidar_ros2_driver
cd /opt/atsd/ws && sudo -u atsd bash -lc 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'
```

Ключевой параметр в конфиге — `isSingleChannel: true`. Без него драйвер
валится на чтении информации об устройстве: X2L однoканальный.

---

## Первый запуск

**Актуаторы не зависят от USB-устройств, проверяйте их первыми:**

```bash
sudo systemctl start atsd-actuators
journalctl -u atsd-actuators -n 20 --no-pager -l
```

Ожидаемый вывод:

```
[light_node] Свет: фары GPIO18, лента GPIO13, потолки 0.60 / 0.50
[lock_node]  Замок: GPIO19, импульс 0.4 с, датчик крышки выкл
```

**Стабильные имена устройств.** В `/etc/udev/rules.d/99-atsd.rules` лежат
заглушки — без реальных ID симлинки не создадутся, а `ttyACM0` и `ttyUSB0`
будут меняться местами после каждой перезагрузки.

```bash
lsusb
udevadm info -a -n /dev/ttyACM1 | grep -E 'idVendor|idProduct|bInterfaceNumber' | head
```

Подставить значения, затем:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -la /dev/v5 /dev/ydlidar /dev/gnss
```

**Запуск всего:**

```bash
sudo systemctl start atsd.target
systemctl status atsd-drive atsd-actuators atsd-sensors
journalctl -u atsd-drive -f
```

---

## Проверка

Окружение подтягивается автоматически при входе по ssh —
установщик прописывает его через `/etc/profile.d/atsd-ros.sh`.
Отдельно ничего подключать не нужно.

```bash
ros2 node list
ros2 topic list
ros2 topic hz /drive/odom        # ожидается около 50

# движение вперёд 0,3 м/с
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# свет
ros2 topic pub --once /lights/headlights std_msgs/msg/Float32 "{data: 0.5}"
ros2 topic pub --once /lights/strip_mode std_msgs/msg/String "{data: 'beacon'}"

# замок
ros2 service call /lock/unlock std_srvs/srv/Trigger
```

---

## Подводные камни, собранные при развёртывании

Все они уже учтены в коде и установщике. Список нужен, если что-то
пойдёт не так на другой машине.

**`bzip2 : Depends: libbz2-1.0 but it is not installable`**
В образе Ubuntu для Raspberry не было веток `noble-updates`
и `noble-backports`. Установщик теперь проверяет и добавляет их сам.
Вручную: в первой секции `/etc/apt/sources.list.d/ubuntu.sources`
строка должна быть `Suites: noble noble-updates noble-backports`.

**`libexec directory does not exist`**
Нет файлов `setup.cfg` в пакетах. Без них colcon молча не создаёт
исполняемые скрипты узлов. Установщик проверяет их наличие до сборки
и результат после.

**`AMENT_TRACE_SETUP_FILES: unbound variable`**
Флаг `set -u` в `run.sh` ломает скрипты окружения ROS. Теперь `source`
обёрнут в `set +u` / `set -u`.

**`BadPinFactory: Unable to load any default pin factory`**
Два разных источника. Первый — не задана `GPIOZERO_PIN_FACTORY=lgpio`,
на Pi 5 автоопределение не срабатывает. Второй, более коварный —
библиотека lgpio создаёт файл `.lgd-nfy` в **текущем каталоге**,
а у сервиса рабочего каталога не было. Отсюда `FileNotFoundError`
и как следствие отказ фабрики. Лечится `WorkingDirectory=/opt/atsd`
в юните.

**Молчаливое зависание на установке ROS**
`curl` без `--max-time` ждёт вечно, если `raw.githubusercontent.com`
недоступен. Теперь таймаут 25 секунд, скачивание во временный файл
и понятное сообщение с инструкцией.

**Группы не подхватились**
`usermod -aG` действует с момента следующего запуска процесса.
После установки нужна перезагрузка, иначе сервис не получит доступ
к GPIO даже при корректных правах на `/dev/gpiochip*`.

**`lgpio.error: can not open gpiochip`**
Пользователь не в группе `gpio`. Установщик добавляет туда `atsd`,
но своего пользователя надо добавить самому и перезайти по ssh:
`sudo usermod -aG gpio $USER`.

**`ros2 node list` пуст при живых узлах, топики не доставляются**
Fast DDS на связке Raspberry Pi 5 плюс wlan0 ведёт себя нестабильно:
узлы то появляются в списке, то пропадают, а данные по топикам
не доходят вообще, хотя подписки зарегистрированы. Перебор профилей
транспорта, отключение разделяемой памяти и смена режима обнаружения
помогали лишь частично. Решение — **Cyclone DDS**:

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

Установщик ставит пакет и прописывает его и сервисам, и пользователям.
Проверить, что дело именно в транспорте, а не в ваших узлах, можно
штатным примером: `ros2 run demo_nodes_cpp talker` в одной сессии
и `ros2 topic echo /chatter` в другой.

**`ros2: command not found` в новой сессии**
`/etc/profile.d` не читается интерактивной ssh-оболочкой — только
login-shell. Поэтому установщик дописывает в `~/.bashrc` строку
подключения `/etc/profile.d/atsd-ros.sh`. Для нового пользователя
она попадает из `/etc/skel/.bashrc`.

**`Permission denied` на `/opt/atsd/ws/install/setup.bash`**
Каталог принадлежит системному пользователю. Установщик открывает
его на чтение через `chmod -R a+rX /opt/atsd`.

**Обнаружение узлов по вайфаю**
Режим `SUBNET` держится на мультикасте, а точки доступа его часто
режут. Поэтому задан `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`:
все узлы живут на одной машине, обнаружение не зависит от роутера,
топики не утекают в сеть. Обратная сторона — с ноутбука по сети
к топикам не подключиться, отладка только по ssh.

---

## Калибровка одометрии

Три числа в `config/atsd.yaml` определяют всю навигацию:
`wheel_diameter_mm`, `track_mm`, `gear_ratio`. Паспортные значения врут
на два-три процента, а это метр увода на дистанции 50 метров.

Порядок: отметить старт, проехать ровно 2 метра по рулетке, сравнить
с `/drive/odom`, поправить диаметр колеса. Затем разворот на 360° —
сравнить накопленный угол, поправить колею.

**Те же три числа продублированы в прошивке V5** (`v5_final_main.cpp`).
Менять надо в обоих местах.

---

## Обновление кода

```bash
cd /opt/atsd/ws/src && sudo -u atsd git pull
cd /opt/atsd/ws && sudo -u atsd bash -lc 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'
sudo systemctl restart atsd.target
```

Из-за `--symlink-install` правки в Python-узлах подхватываются без
пересборки — достаточно перезапустить сервис.

---

## Что дальше

Нижний уровень готов. Верхний строится поверх этих топиков:

- SLAM и локализация: `slam_toolbox`, затем AMCL по сохранённой карте
- Навигация: Nav2 с деревом поведения под знаки и светофор
- Восприятие: YOLO на кадрах `/camera/image_raw`
- Парковка по контрастному пятну: порог HSV, доводка по смещению от центра
- BLE-зоны точек доставки
- Бэкенд заказов на FastAPI и веб-интерфейс с картой
