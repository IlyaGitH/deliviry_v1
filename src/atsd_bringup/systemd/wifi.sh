#!/usr/bin/env bash
# Wi-Fi робота: NetworkManager сам подключается к известным сетям,
# а если устойчивого подключения нет WAIT_S секунд — поднимаем свою точку доступа.
# Устойчивое — держится STABLE_S подряд с сигналом от MIN_SIGNAL %: на краю зоны сеть
# то ловится, то пропадает, и без этого точка не поднималась бы никогда.
set -u
export LC_ALL=C

IFACE=wlan0
AP_SSID=ATSD-1M
AP_PASS=
AP_ADDR=10.42.0.1/24
AP_BAND=bg     # a — 5 ГГц, если поддерживают клиенты и разрешает регион
AP_CHANNEL=0   # 0 — выбор NetworkManager; иначе фиксированный канал
WAIT_S=60
STABLE_S=30     # связь считается, только если продержалась столько подряд
MIN_SIGNAL=25   # % сигнала известной сети; слабее — как будто её нет
AP_RETRY_ENABLED=0  # сохранять AP; поиск известных сетей с отключением AP — только явно
RETRY_S=180     # в режиме точки, если к ней никто не подключён, — раз в столько ищем известные сети
POLL_S=5
CONF=${ATSD_WIFI_CONF:-/etc/atsd/wifi.conf}
[[ -f ${CONF} ]] && . "${CONF}"

AP=atsd-hotspot

# Ограничиваем ожидание nmcli, чтобы запуск не зависал при загрузке NetworkManager.
nmcli() { command nmcli --wait 10 "$@"; }

no_clients() {
  local stations
  stations=$(iw dev "${IFACE}" station dump 2>/dev/null) || return 1
  [[ -z ${stations} ]]
}


dev_state()  { nmcli -g GENERAL.STATE dev show "${IFACE}" 2>/dev/null; }       # «100 (connected)»
dev_conn()   { nmcli -g GENERAL.CONNECTION dev show "${IFACE}" 2>/dev/null; }
signal()     { nmcli -g IN-USE,SIGNAL dev wifi list ifname "${IFACE}" --rescan no 2>/dev/null |
               awk -F: '$1=="*"{print $2; exit}'; }
# Не считать пустой/нечисловой сигнал доказательством устойчивой связи.
link_ok() {
  local strength
  [[ $(dev_state) == 100* ]] || return 1
  strength=$(signal)
  [[ ${strength} =~ ^[0-9]+$ ]] && (( 10#${strength} >= MIN_SIGNAL ))
}

start_ap() {
  if (( ${#AP_PASS} < 8 )); then
    echo "AP_PASS в ${CONF} короче 8 символов — точка доступа не поднята"
    return 1
  fi
  # Профиль пересоздаём каждый раз, чтобы правки wifi.conf применялись без ручной чистки
  nmcli con delete "${AP}" >/dev/null 2>&1
  nmcli con add type wifi ifname "${IFACE}" con-name "${AP}" autoconnect no ssid "${AP_SSID}" \
        802-11-wireless.mode ap 802-11-wireless.band "${AP_BAND}" 802-11-wireless.channel "${AP_CHANNEL}" \
        802-11-wireless.powersave 2 \
        ipv4.method shared ipv4.addresses "${AP_ADDR}" ipv6.method disabled \
        wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn \
        wifi-sec.pairwise ccmp wifi-sec.group ccmp wifi-sec.psk "${AP_PASS}" >/dev/null &&
  nmcli con up "${AP}" >/dev/null
}

main() {
  # NetworkManager и wlan0 могут появиться после запуска сервиса.
  until nmcli -g GENERAL.STATE dev show "${IFACE}" >/dev/null 2>&1; do
    echo "Ожидаю интерфейс ${IFACE} в NetworkManager"
    sleep "${POLL_S}"
  done
  nmcli dev set "${IFACE}" managed yes || exit 1
  nmcli radio wifi on || exit 1
  echo "Ищу известные сети на ${IFACE}, своя точка «${AP_SSID}» через ${WAIT_S} с без связи"
  since=$(date +%s)
  ok_since=0
  ap_since=$(date +%s)
  was=
  while true; do
    now=$(date +%s)
    if [[ $(dev_state) == 100* && $(dev_conn) == "${AP}" ]]; then
      since=${now}
      state=ap
      # в режиме точки модуль Pi не сканирует: чтобы найти сеть, появившуюся позже (раздачу
      # телефона), точку ненадолго гасим — только если к ней никто не подключён
      if (( AP_RETRY_ENABLED && now - ap_since >= RETRY_S )) && command -v iw >/dev/null && no_clients; then
        echo "К точке никто не подключён — на ${WAIT_S} с ищу известные сети"
        nmcli con down "${AP}" >/dev/null 2>&1
        nmcli dev set "${IFACE}" autoconnect yes >/dev/null 2>&1  # после dev disconnect NM сам не подключался бы
        ap_since=${now}
        state=retry
      fi
    elif link_ok; then
      (( ok_since )) || ok_since=${now}
      # на краю зоны сеть то ловится, то нет: короткие подключения таймер не сбрасывают
      (( now - ok_since >= STABLE_S )) && since=${now}
      state="есть: $(dev_conn), сигнал $(signal)%"
    else
      ok_since=0
      state="нет (${WAIT_S} с без связи — своя точка)"
    fi
    if [[ ${state} != ap && ${state} != retry ]] && (( now - since >= WAIT_S )); then
        echo "Устойчивой связи с известной сетью нет ${WAIT_S} с — поднимаю точку доступа «${AP_SSID}», адрес ${AP_ADDR%/*}"
        nmcli dev disconnect "${IFACE}" >/dev/null 2>&1  # иначе NM продолжает цепляться за слабую сеть
        if start_ap; then
          echo "Точка доступа поднята: веб-интерфейс http://${AP_ADDR%/*}:8080"
          state=ap
          ap_since=$(date +%s)
        else
          echo "Точку доступа поднять не удалось, повтор через ${WAIT_S} с"
          nmcli dev set "${IFACE}" autoconnect yes >/dev/null 2>&1  # пусть снова ищет известные сети
        fi
        since=$(date +%s)
        ok_since=0
    fi
    [[ ${state%%:*} != "${was%%:*}" ]] && echo "Связь: ${state}"
    was=${state}
    sleep "${POLL_S}"
  done
}

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main
fi
