# бортовое ПО

Автономное транспортное средство доставки грузов для внутренней логистики
НТЦ АО «АВТОВАЗ». Репозиторий содержит всё ПО робота: прошивку VEX V5 Brain
и пакеты ROS 2 для Raspberry Pi 5.

**Платформа:** Raspberry Pi 5 · Ubuntu 24.04 Server arm64 · **ROS 2 Jazzy** · VEX V5

> Именно Jazzy. Humble привязан к Ubuntu 22.04 и на 24.04 не ставится.

---

## Архитектура: Pi ведёт по лидарной карте, Brain исполняет

```
 ┌──────────────────── Raspberry Pi 5 ────────────────────┐      USB      ┌────── VEX V5 Brain ──────┐
 │ lidar_node ─/scan─> slam_toolbox (карта, поза map)     │               │ состояние рейса           │
 │        │                 │                              │               │ паузы: знак, светофор,    │
 │        │            Nav2: путь + объезд ─nav/cmd_vel─┐   │  G  скорость  │   препятствие, связь      │
 │        └─> obstacle_node ──────────────────────────┐ ├─> v5_bridge ──> │ отъезд при застревании    │
 │ vision_node (знаки, светофор) ─────────────────────┤ │  │  Y/Q  итог    │ гироскоп+энкодеры -> odom │
 │ nav_node: рейс -> цель Nav2 -> «приехал» ──────────┘─┘  │ <── E P S K   │ 6 моторов, бамперы, экран │
 │ web_node (заказы) ── M / C / A · signal_node ─> свет   │               └───────────────────────────┘
 └────────────────────────────────────────────────────────┘
```

Робот ориентируется по лидару: `slam_toolbox` сопоставляет каждый скан
с заранее построенной картой помещения и держит позу робота в карте.
Одометрия Brain (гироскоп + энкодеры) нужна только как грубая подсказка
между сканами — её ошибка не копится, сканы её всё время исправляют.

Путь до точки строит **Nav2** по карте и сам объезжает препятствия,
которые видит лидар. Скорость от Nav2 мост шлёт на Brain командой `G`.
Brain по-прежнему ведёт состояние рейса (погрузка, выдача, отмена, сбой)
и накладывает на скорость Nav2 правила: стоит у знака «Стоп» и на красный,
сбавляет у перехода и неровности, не едет вперёд при препятствии ближе
0,45 м, отъезжает назад при застревании, встаёт без связи с Pi.
Движения по точкам одометрии в прошивке больше нет.

`nav_node` связывает рейс и Nav2: видит в `/mission/state`, что Brain едет
за грузом или к получателю, ставит цель Nav2 в координатах точки из карты,
а по итогу шлёт Brain `Y` (приехал) или `Q` (путь не найден, сбой
`NAV_FAILED`). На паузах у знака, светофора и при отъезде цель снимается
и ставится заново, когда пауза кончится.

Если Pi или Nav2 упадут, Brain встанет: без heartbeat — сразу,
без команд `G` больше 20 с — сбой `NO_NAV`. Вслепую он не поедет.

---

## Как робот находит точку

1. **Лидарная карта.** Координаты точек заданы в метрах карты
   (`/opt/atsd/data/points.json`, правятся на карте в веб-интерфейсе).
   Nav2 доводит робота до точки с допуском 10 см, поза берётся из
   `slam_toolbox`, а не из счисления.
2. **BLE-метка.** На каждой точке лежит телефон. `ble_node` узнаёт его
   по имени или iBeacon и сообщает Brain уровень сигнала. На прибытии
   Brain пишет `ZONE_OK`, `ZONE_MISMATCH` (рядом метка другой точки)
   или `ZONE_UNCONFIRMED`. `lock_node` может открывать отсек только при
   метке нужной точки рядом (`zone_policy: require`).

### Телефоны-метки

MAC-адрес телефонов меняется, поэтому метка узнаётся по содержимому
рекламного пакета. Подходит любой из двух способов.

| Способ | Android | iPhone |
|---|---|---|
| По имени | nRF Connect → Advertiser → новый пакет, Complete Local Name, имя Bluetooth телефона `ATSD-KPP` | nRF Connect → Advertiser, Local Name `ATSD-KPP`, приложение на экране |
| iBeacon | Beacon Simulator → iBeacon, UUID `E2C56DB5-DFFB-48D2-B060-D0F5A71096E0`, major 1, minor = номер точки | не поддерживается в фоне |

| Точка | Имя | minor |
|---|---|---|
| КПП | `ATSD-KPP` | 1 |
| Администрация | `ATSD-ADMIN` | 2 |
| Зона тестирования | `ATSD-TEST` | 3 |
| Склад | `ATSD-SKLAD` | 4 |

Проверка: `sudo python3 /opt/atsd/hw_check.py --no-gpio` покажет найденные
метки и уровень сигнала. Порог «рядом» — `near_rssi: -65` в `atsd.yaml`;
положите телефон на точку, поставьте робота на неё и посмотрите
реальный уровень в `ros2 topic echo /ble/zone`.

### Карта

Строится один раз и заново — после перестановки в помещении.

```bash
sudo systemctl stop atsd-nav
sudo -u atsd /opt/atsd/run.sh ros2 launch atsd_bringup nav.launch.py mode:=mapping
```

Поставьте робота на место старта (оно станет началом карты, точка `home`)
и медленно объездите помещение с пульта VEX (L1 + стики) или через
`/cmd_vel`. Возвращайтесь в уже снятые места — так замыкаются петли.
Затем, не останавливая launch:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/opt/atsd/maps/map'}"
sudo -u atsd /opt/atsd/run.sh ros2 run nav2_map_server map_saver_cli -f /opt/atsd/maps/indoor
sudo systemctl start atsd-nav
```

Первая команда сохраняет карту для локализации (`map.posegraph`,
`map.data`), вторая — `indoor.pgm` + `indoor.yaml` для подложки
веб-интерфейса (см. `atsd_web/static/maps/README.txt`).

В рабочем режиме локализация стартует с начала карты
(`map_start_at_dock: true` в `slam.yaml`), поэтому включайте робота на месте
старта. Если он стоит в другом месте — задайте позу в RViz
(«2D Pose Estimate») или `ros2 topic pub --once /initialpose …`.
Проверка: `ros2 run tf2_ros tf2_echo map base_link` — поза не должна
прыгать, когда робот стоит.

Затем откройте веб-интерфейс и расставьте точки на карте.

---

## Состав

| Каталог | Что делает |
|---|---|
| `firmware/vex_v5` | Прошивка V5 Brain 3.0: состояние рейса, исполнение скорости Nav2, паузы, нештатные ситуации, экран |
| `src/atsd_drive` | Мост с Brain по USB (команды, события, одометрия, TF) и `nav_node` — рейс через Nav2 |
| `src/atsd_perception` | Препятствия по лидару, знаки, светофор и красная разметка по камере |
| `src/atsd_actuators` | Фары, лента, замок, зуммер, автоматика сигнализации |
| `src/atsd_sensors` | Драйвер лидара LD19, GNSS BN-880 с настройкой по UBX, гейдж X1202, BLE-метки |
| `src/atsd_web` | Веб-интерфейс заказов: карта, статус, журнал рейса, отчёт для протокола |
| `src/atsd_bringup` | Launch-файлы, общий конфиг, `slam.yaml` и `nav2.yaml`, systemd, udev, установщик, проверка железа |

---

## Подключение

### USB

| Устройство | Симлинк | Примечание |
|---|---|---|
| VEX V5 Brain | `/dev/v5` | user-порт, interface 02 (`2888:0501`) |
| Лидар LDROBOT D500 (LD19) | `/dev/lidar` | через CP2102 (`10c4:ea60`), 230400 бод |
| Камера Exegate | `/dev/camera` | первый видеоузел USB-устройства |

### Колодка GPIO

| Пин | GPIO | Куда | Примечание |
|---|---|---|---|
| 3 | GPIO2 (SDA) | X1202 | через pogo-контакты, гейдж заряда 0x36 |
| 5 | GPIO3 (SCL) | X1202 | через pogo-контакты |
| 4 | 5 В | BN-880 VCC | |
| 6 | GND | BN-880 GND | |
| 8 | GPIO14 (TXD) | BN-880 RX | линии перекрещиваются |
| 10 | GPIO15 (RXD) | BN-880 TX | `/dev/ttyAMA0` → `/dev/gnss` |
| 12 | GPIO18 | затвор канала 1 — фары | ШИМ |
| 13 | GPIO27 | геркон крышки, второй конец на GND (пин 14) | `use_door_sensor` |
| 31 | GPIO6 | X1202, детектор пропадания питания | занят платой |
| 33 | GPIO13 | затвор канала 2 — лента | ШИМ |
| 35 | GPIO19 | затвор канала 3 — замок | импульс 0,4 с |
| 36 | GPIO16 | X1202, управление зарядом | занят платой |
| 37 | GPIO26 | затвор канала 4 — зуммер SFM-27 | |

Номера пинов по умолчанию зашиты в `atsd_actuators/pins.py`. Узлы
отказываются стартовать, если в конфиге указан занятый пин (I2C, UART,
выводы X1202) или два устройства на одном GPIO.

### VEX V5

Моторы: 1 — переднее левое, 10 — среднее левое, 9 — заднее левое,
19 — переднее правое, 8 — среднее правое, 20 — заднее правое.
Гироскоп 276-2333 — 3-проводной порт A. Бамперы (необязательно) — B и C.

---

## Прошивка VEX V5

Файл `firmware/vex_v5/src/main.cpp`, собирается расширением VEX для VS Code
(Windows) или веб-версией VEXcode V5.

1. VS Code → значок VEX → New Project → V5 → C++ → Empty Project.
2. Заменить `src/main.cpp` файлом из репозитория. Если в проекте есть
   `robot-config.cpp` с `brain Brain;` — удалить там дубли.
3. Подключить Brain по USB, выбрать слот, **Build**, затем **Download**, затем **Run**.
4. Первые 2 секунды после запуска робот должен стоять: калибруется гироскоп.

Перед первым рейсом, вкладки на экране Brain:

| Вкладка | Что проверить |
|---|---|
| TEST → кнопки моторов | каждое колесо катит робот вперёд, иначе поменять реверс в разделе 3 |
| DIAG | поворот рукой влево — YAW растёт, иначе `GYRO_SIGN = 1.0` |
| TEST → DRIVE 1M | сравнить с рулеткой, поправить `WHEEL_DIAM_MM` |
| TEST → TURN 90 | вписать показанную оценку в `TRACK_EFF_MM` |

Координат точек в прошивке больше нет — только имена (`NODES`, номера
0..3 = `point_ids` моста). Координаты живут на Pi, в лидарной карте.

---

## Установка на Raspberry Pi

```bash
git clone https://github.com/IlyaGitH/deliviry_v1 ~/atsd_ws
cd ~/atsd_ws/src/atsd_bringup/scripts
sudo bash install.sh --with-yolo
sudo reboot
```

Скрипт ставит ROS 2 Jazzy, Nav2 и slam_toolbox, создаёт пользователя `atsd`,
собирает workspace в `/opt/atsd/ws`, прописывает сервисы и udev-правила,
включает UART0 на GPIO14/15 под GNSS, I2C под X1202, снимает лимит
600 мА на USB. Идемпотентен.

| Флаг | Когда нужен |
|---|---|
| `--with-yolo` | поставить `ncnn` для распознавания знаков (без torch) |
| `--skip-upgrade` | система уже обновлена |
| `--skip-ros` | ROS ставится отдельно |
| `--ros-key=ФАЙЛ` | `raw.githubusercontent.com` недоступен, ключ скачан вручную |

Ключ ROS вручную, если GitHub не открывается:
`curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o ros.key`
на машине с доступом, затем `scp` на Pi и `--ros-key=`.

**Модель знаков** положить в `/opt/atsd/models/signs_ncnn_model`
(папка после `yolo export model=best.pt format=ncnn imgsz=640`: внутри
`model.ncnn.param`, `model.ncnn.bin`, `metadata.yaml`), затем
`sudo chown -R atsd: /opt/atsd/models`.

Модель запускается через пакет `ncnn` напрямую (`backend: auto`).
**Не ставьте на Pi `ultralytics` и `torch`:** сборка torch для aarch64 с PyPI
тянет гигабайты CUDA-библиотек, а ultralytics требует numpy 2 и ломает
системные numpy, OpenCV и setuptools из apt. Экспорт модели делайте на
компьютере или в Kaggle, на Pi нужна только готовая папка.

### Если на Pi уже пробовали ставить ultralytics

По одному пакету, чтобы ошибка на одном не остановила остальные:

```bash
for p in ultralytics ultralytics-thop ultralytics-platform torch torchvision triton \
         cuda-toolkit cuda-bindings cuda-pathfinder nvidia-ml-py nvidia-cusparselt-cu13 \
         nvidia-nvtx nvidia-nvshmem-cu13 nvidia-nvjitlink nvidia-nccl-cu13 nvidia-curand \
         nvidia-cufile nvidia-cuda-runtime nvidia-cuda-nvrtc nvidia-cuda-cupti nvidia-cusparse \
         nvidia-cufft nvidia-cublas nvidia-cusolver nvidia-cudnn-cu13 polars-runtime-32 \
         sympy mpmath setuptools; do
  sudo pip3 uninstall --break-system-packages -y "$p" 2>/dev/null
done
sudo apt install --reinstall -y python3-setuptools python3-numpy python3-opencv
sudo pip3 cache purge
python3 -c "import setuptools, numpy, cv2; print(setuptools.__version__, numpy.__version__, cv2.__version__)"
```

Ожидается `68.1.2 1.26.4 4.6.0`.

### Обновление с прошлой версии

```bash
cd ~/atsd_ws && git pull
sudo bash src/atsd_bringup/scripts/install.sh --skip-upgrade --skip-ros --with-yolo
sudo reboot
```

Установщик сам уберёт из `config.txt` устаревшие `dtoverlay=uart2`
(GNSS больше не на GPIO4/5) и `dtoverlay=disable-bt` (Bluetooth нужен
для меток), поставит `bleak`, Nav2, slam_toolbox и новый udev.
Прошивку V5 обновить до 3.0 — протокол изменился, старая прошивка
с новым мостом не поедет. После обновления постройте карту (раздел «Карта»).

---

## Проверка железа

Без ROS, по очереди каждое устройство:

```bash
sudo systemctl stop atsd.target
sudo python3 /opt/atsd/hw_check.py          # всё, кроме замка
sudo python3 /opt/atsd/hw_check.py --lock   # плюс импульс на замок
```

Скрипт проверит симлинки, спросит у Brain версию прошивки и посчитает
телеметрию, посчитает кадры лидара, прочитает кадр камеры и поищет на нём
красную разметку, найдёт NMEA от GNSS на любой скорости, найдёт
телефоны-метки, прочитает гейдж X1202, плавно зажжёт фары и ленту,
пискнет зуммером, покажет состояние геркона.

Затем всё вместе:

```bash
sudo systemctl start atsd.target
systemctl status atsd-drive atsd-perception atsd-nav atsd-actuators atsd-sensors atsd-web
journalctl -u atsd-drive -f
```

В логе моста должно быть `Прошивка: ATSD-1M 3.0.0` и `Все источники
восприятия на связи`, в `journalctl -u atsd-nav` — `Managed nodes are active`.

---

## Сценарий доставки

1. Администратор открывает `http://<адрес-пи>:8080`, выбирает «откуда»
   и «куда» (КПП, Администрация, Зона тестирования, Склад) и нажимает
   «Отправить робота». Веб публикует `/mission/start`, мост шлёт `M` на Brain.
   Заказ можно дать и с экрана Brain, вкладка MISSION.
2. Робот едет к точке погрузки. Отправитель по своей ссылке открывает
   отсек, кладёт груз, закрывает крышку и жмёт «Готово» — `C` на Brain.
3. Робот едет к точке выдачи. В пути отсек заблокирован: `lock_node`
   отказывает в открытии, а при открытой крышке (геркон) поднимает тревогу.
4. Получатель открывает отсек, забирает груз, жмёт «Готово». Рейс закрыт,
   таймер остаётся на экране.

Статус заказа берётся из состояния миссии на Brain, а не вычисляется
по расстоянию: робот сам решает, что приехал.

**Отчёт для протокола испытаний:** `GET /api/report/<id>` — общее время,
факт доставки, распознавания знака и светофора, обнаружения и объезда
препятствия, список сбоев и полный журнал событий.

---

## Протокол Pi ↔ Brain

Текст, одна команда в строке. Полное описание — в шапке прошивки
и в `atsd_drive/protocol.py`.

| Pi → Brain | Смысл |
|---|---|
| `M <from> <to>` | старт рейса, точки 0 KPP · 1 ADMIN · 2 TEST · 3 SKLAD |
| `C` · `A` · `U` | груз подтверждён · отмена · продолжить после сбоя |
| `H` | heartbeat, 10 Гц, только пока жив детектор препятствий |
| `G <мм/с> <мрад/с>` | скорость от Nav2 во время рейса, старше 0,3 с — стоп |
| `Y <состояние>` · `Q <состояние>` | Nav2 довёл до точки · не смог (`FAULT NAV_FAILED`) |
| `B <точка> <дБм> <рядом>` | ближайшая BLE-метка, `-1` — нет |
| `F STOP\|CROSS\|BUMP <мм>` | знак и расстояние до него |
| `T R\|Y\|G\|N <мм>` | светофор |
| `O <мм> [сторона …]` | ближайшее препятствие в коридоре, `O 0` — чисто; Brain берёт только расстояние |
| `X` · `R` · `L <%>` | аварийный стоп · снять · ограничение скорости |
| `V <мм/с> <мрад/с>` | прямое управление вне рейса (совместимость) |
| `N <мм/с> <мрад/с>` | подруливание оператором во время рейса, только для отладки |

| Brain → Pi | Частота | Смысл |
|---|---|---|
| `E degL degR yaw ms mV flags` | 50 Гц | энкодеры, курс, батарея, флаги |
| `P x y yaw v w` | 20 Гц | поза, мм и миллиградусы |
| `S state from to pause t` | 5 Гц | состояние рейса |
| `K EVENT args` | по событию | `ARRIVED_PICKUP`, `STOP_SIGN_HOLD`, `OBSTACLE_STOP 420`, `ZONE_OK ADMIN -58`, `FAULT NAV_FAILED`… |

---

## Интерфейс ROS

### Публикуется

| Топик | Тип | Назначение |
|---|---|---|
| `/drive/odom` + TF `odom → base_link` | `nav_msgs/Odometry` | счисление Brain: гироскоп + энкодеры, подсказка для slam_toolbox |
| TF `map → odom` · `/map` | | `slam_toolbox`: поза робота в лидарной карте |
| `nav/cmd_vel` | `geometry_msgs/Twist` | скорость от Nav2, мост шлёт её на Brain (`G`) |
| `nav/result` · `nav/remain` | `std_msgs/String` · `Float32` | `nav_node`: `ok TO_PICKUP` / `fail …` · остаток пути, м |
| `/drive/battery` | `sensor_msgs/BatteryState` | батарея V5 |
| `/drive/link` · `/drive/estop` | `std_msgs/Bool` | Brain на связи · аварийный стоп |
| `/drive/flags` · `/drive/firmware` | `std_msgs/String` | флаги Brain · версия прошивки |
| `/mission/state` | `std_msgs/String` (JSON) | состояние, точки, причины паузы, время рейса |
| `/mission/event` | `std_msgs/String` | события Brain и `LOCK_TAMPER` |
| `/perception/obstacle` | `std_msgs/String` | `0.95 L 0.00 0.00` или `0` |
| `/perception/sign` · `/perception/light` | `std_msgs/String` | `STOP 2.30` · `R 4.10` |
| `/perception/detections` | `std_msgs/String` (JSON) | все рамки YOLO |
| `/perception/spot` | `std_msgs/String` | красная разметка: `1.583 0.204 0.136` (x, y, м²) или `none`, в рейсе не используется |
| `/ble/zone` · `/ble/zones` | `std_msgs/String` | ближайшая метка `admin -58 1` · все метки (JSON) |
| `/perception/debug/compressed` | `sensor_msgs/CompressedImage` | кадр с рамками, 2 Гц |
| `/scan` | `sensor_msgs/LaserScan` | лидар, 10 Гц |
| `/gnss/fix` · `/gnss/vel` · `/gnss/status` | | GNSS |
| `/power/computer` | `sensor_msgs/BatteryState` | X1202 |
| `/lock/busy` · `/lock/closed` · `/lock/tamper` | `std_msgs/Bool` | замок и крышка |

### Принимается

| Топик или сервис | Тип | Назначение |
|---|---|---|
| `/mission/start` | `std_msgs/String` | `"kpp depot"` или `"0 3"` |
| `/mission/cargo` · `/mission/abort` · `/mission/resume` | `std_msgs/Empty` | подтверждение, отмена, продолжение |
| `/drive/estop_cmd` | `std_msgs/Bool` | аварийный стоп |
| `/drive/speed_limit` | `std_msgs/Float32` | 0.0–1.0 |
| `/cmd_vel` | `geometry_msgs/Twist` | вне рейса |
| `/cmd_nudge` | `geometry_msgs/Twist` | подруливание во время рейса (отладка) |
| `/lock/unlock` | `std_srvs/Trigger` | импульс на замок: в пути отказ, без метки точки — по `zone_policy` |
| `/lights/headlights` · `/lights/strip` · `/lights/strip_mode` | | ручное управление светом |
| `/buzzer/pattern` | `std_msgs/String` | `beep` `double` `start` `arrive` `approach` `reverse` `alarm` `fault` `off` |

Ручные команды свету и зуммеру перебивает `signal_node`. Для отладки
выключите его: `ros2 param set /signal_node enabled false`.

---

## Безопасность движения и груза

**Сигнализация** (`signal_node`). В движении горят фары и работает
проблесковый маячок на ленте. При препятствии ближе 1,8 м и после знака
«Пешеходный переход» звучит сигнал о приближении. Задний ход при выходе
из застревания сопровождается отдельным сигналом. На паузе у знака или
светофора лента мигает. При аварии включаются маячок и сирена.

**Токовый интерлок.** Шина 12 В тянет около 1,4 А, соленоид замка берёт
1,1 А. На время импульса `lock_node` публикует `/lock/busy`, `light_node`
гасит свет.

**Heartbeat по живым детекторам.** Мост шлёт `H`, только пока приходят
данные от `obstacle_node` (и `vision_node`, если добавить его в
`require_feeds`). Упал лидар — через полсекунды Brain встаёт на паузу
«NO LINK TO PI».

**Груз.** В пути замок не открыть ни из веба, ни сервисом. Открытие крышки
в пути фиксируется и попадает в журнал рейса. На точке отсек открывается
при BLE-метке этой точки рядом: `zone_policy: warn` — открыть можно,
но в журнале будет `ZONE_UNCONFIRMED_UNLOCK`; `require` — только с меткой.

---

## Калибровка

**Одометрия и курс** — в прошивке, вкладка TEST на экране Brain
(см. раздел о прошивке). Высокая точность не нужна, её исправляет лидар,
но грубые ошибки (перепутан знак гироскопа, колесо вдвое больше) мешают
сопоставлению сканов.

**Лидар** — TF `base_link → laser_frame` в `sensors.launch.py` по реальной
установке. Если лидар видит корпус робота, вырежьте эти углы `crop_deg`
в `lidar_node`, иначе Nav2 сочтёт робота стоящим в препятствии.

**Nav2** — `config/nav2.yaml`: `footprint` по габариту корпуса,
`inflation_radius` (запас от стен), `desired_linear_vel`. Если в узких местах
путь не находится — уменьшите `inflation_radius`.

**Коридор препятствий** — `obstacle_node` в `atsd.yaml`: `front_x`
(передний край корпуса от центра), `half_width`, `laser_x` по реальной
установке лидара. Проверка: `ros2 topic echo /perception/obstacle_info`,
поставить коробку в метре перед роботом.

---

## Тесты---

## Тесты

Логика протокола, ведения рейса через Nav2, детектора препятствий,
красной разметки, цвета светофора, BLE-меток и сигнализации проверяется
без ROS и без робота:

```bash
cd ~/atsd_ws/src
PYTHONPATH=atsd_drive:atsd_perception:atsd_actuators:atsd_sensors python3 -m pytest -q \
  atsd_drive/test atsd_perception/test atsd_actuators/test atsd_sensors/test
```

Веб-интерфейс без робота: `ros2 launch atsd_web web.launch.py mock:=true` —
имитация проходит тот же сценарий, что и прошивка.

---

## Веб-интерфейс

`http://<адрес-пи>:8080`. Роли различаются ссылкой:

| Ссылка | Кто | Что может |
|---|---|---|
| `/` | администратор | создаёт заказ, отменяет рейс, продолжает после сбоя, расставляет точки на карте |
| `/?role=sender&order=ID` | отправитель | открывает отсек на погрузке, подтверждает загрузку |
| `/?role=receiver&order=ID` | получатель | открывает отсек на выдаче, подтверждает получение |

На экране: карта с ровером, этапы доставки, таймер рейса, время до точки,
причина паузы («красный сигнал светофора», «препятствие впереди»),
журнал рейса по-русски, заряд обеих батарей, крышка отсека, связь с роботом.

Подложки карт кладутся в `atsd_web/static/maps/`, привязка — `DEFAULT_MAPS`
в `server.py`. Без файлов рисуется метрическая сетка.
