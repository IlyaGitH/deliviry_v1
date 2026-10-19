#!/usr/bin/env bash
# Wi-Fi робота: NetworkManager сам подключается к известным сетям,
# а если подключения нет WAIT_S секунд — поднимаем свою точку доступа.
set -u

IFACE=wlan0
AP_SSID=ATSD-1M
AP_PASS=
AP_ADDR=10.42.0.1/24
WAIT_S=60
POLL_S=5
CONF=${ATSD_WIFI_CONF:-/etc/atsd/wifi.conf}
[[ -f ${CONF} ]] && . "${CONF}"

AP=atsd-hotspot

dev_state()  { nmcli -g GENERAL.STATE dev show "${IFACE}" 2>/dev/null; }       # «100 (connected)»
dev_conn()   { nmcli -g GENERAL.CONNECTION dev show "${IFACE}" 2>/dev/null; }

start_ap() {
  if (( ${#AP_PASS} < 8 )); then
    echo "AP_PASS в ${CONF} короче 8 символов — точка доступа не поднята"
    return 1
  fi
  # Профиль пересоздаём каждый раз, чтобы правки wifi.conf применялись без ручной чистки
  nmcli con delete "${AP}" >/dev/null 2>&1
  nmcli con add type wifi ifname "${IFACE}" con-name "${AP}" autoconnect no ssid "${AP_SSID}" \
        802-11-wireless.mode ap 802-11-wireless.band bg \
        ipv4.method shared ipv4.addresses "${AP_ADDR}" ipv6.method disabled \
        wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn \
        wifi-sec.pairwise ccmp wifi-sec.group ccmp wifi-sec.psk "${AP_PASS}" >/dev/null &&
  nmcli con up "${AP}" >/dev/null
}

nmcli radio wifi on
echo "Ищу известные сети на ${IFACE}, своя точка «${AP_SSID}» через ${WAIT_S} с без связи"
since=$(date +%s)
while true; do
  if [[ $(dev_conn) == "${AP}" ]]; then
    # ponytail: из режима точки обратно не уходим — в AP-режиме brcmfmac не сканирует.
    # Вернуться в известную сеть: перезагрузка или  sudo nmcli con up <сеть>
    since=$(date +%s)
  elif [[ $(dev_state) == 100* ]]; then
    since=$(date +%s)
  elif (( $(date +%s) - since >= WAIT_S )); then
    echo "Известных сетей нет ${WAIT_S} с — поднимаю точку доступа «${AP_SSID}», адрес ${AP_ADDR%/*}"
    if start_ap; then
      echo "Точка доступа поднята: веб-интерфейс http://${AP_ADDR%/*}:8080"
    else
      echo "Точку доступа поднять не удалось, повтор через ${WAIT_S} с"
    fi
    since=$(date +%s)
  fi
  sleep "${POLL_S}"
done
