#!/usr/bin/env bash
# Called by both installation paths. Does not modify Wi-Fi or operator SSH access.
set -euo pipefail
[[ $EUID -eq 0 ]]
security_source=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if ! command -v nginx >/dev/null || ! command -v nft >/dev/null || ! /usr/bin/python3 -c 'import cryptography' 2>/dev/null; then
  apt-get update
  apt-get install -y nginx python3-cryptography nftables
fi
# Validate kernel support before update.sh stops existing robot services.
nft --check -f "$security_source/ros-network.nft"
[[ ${1:-} != --dependencies-only ]] || exit 0
install -d -o root -g root -m 755 /usr/local/libexec
install -o root -g root -m 755 "$security_source/manage.py" /usr/local/libexec/atsd-security.py
install -o root -g root -m 755 "$security_source/audit.py" /usr/local/libexec/atsd-audit.py
/usr/bin/python3 -I /usr/local/libexec/atsd-security.py provision
install -o root -g root -m 644 "$security_source/cyclonedds.xml" /etc/atsd/cyclonedds.xml
install -o root -g root -m 600 "$security_source/ros-network.nft" /etc/atsd/security/ros-network.nft
install -d -m 755 /etc/systemd/system/atsd-web.service.d
cat > /etc/systemd/system/atsd-web.service.d/security.conf <<'UNIT'
[Unit]
Requires=atsd-audit.service
After=atsd-audit.service

[Service]
RuntimeDirectory=atsd-web
RuntimeDirectoryMode=0750
UMask=0007
Group=www-data
SupplementaryGroups=atsd
Environment=PYTHONDONTWRITEBYTECODE=1
LimitCORE=0
UNIT
# RuntimeDirectory inherits Group=www-data: nginx can reach only the web socket,
# without joining atsd or gaining read access to the application encryption key.
# Deny writes to executable code by the robot service identity.
# The parent must also be protected, otherwise atsd could rename the code tree.
chown root:root /opt/atsd
chmod 755 /opt/atsd
for directory in data .ros .cache .config; do
  install -d -o atsd -g atsd -m 700 "/opt/atsd/$directory"
done
chown -R root:root /opt/atsd/ws
chmod -R go-w /opt/atsd/ws
chown root:root /opt/atsd/run.sh /opt/atsd/wifi.sh
chmod go-w /opt/atsd/run.sh /opt/atsd/wifi.sh
# Manifest checks use a fixed root-owned executable before each robot subsystem.
for unit in atsd-web atsd-drive atsd-sensors atsd-perception atsd-nav atsd-actuators; do
  install -d -m 755 "/etc/systemd/system/$unit.service.d"
  cat > "/etc/systemd/system/$unit.service.d/integrity.conf" <<'UNIT'
[Unit]
Requires=atsd-ros-network.service
After=atsd-ros-network.service

[Service]
ExecStartPre=+/usr/bin/python3 -I /usr/local/libexec/atsd-security.py verify
Environment=PYTHONDONTWRITEBYTECODE=1
LimitCORE=0
UNIT
done
# Apply device permissions now, not only on the next USB reconnect.
udevadm control --reload-rules
udevadm trigger --action=change --subsystem-match=tty
# Unused serial login console is not the GNSS device driver.
systemctl disable --now serial-getty@ttyAMA0.service 2>/dev/null || true
systemctl mask serial-getty@ttyAMA0.service
/usr/bin/python3 -I /usr/local/libexec/atsd-security.py seal
systemctl daemon-reload
systemctl enable atsd-audit atsd-gateway atsd-ros-network
systemctl restart atsd-ros-network
systemctl restart atsd-audit
# Empty distro nginx site is unrelated to the secured dedicated gateway; disable only that default.
if [[ -L /etc/nginx/sites-enabled/default ]]; then
  rm /etc/nginx/sites-enabled/default
  systemctl reload nginx 2>/dev/null || true
fi
echo 'После запуска: https://rasber.local/ ; отладка: https://rasber.local/debug/'
echo 'Доступ только после установки CA и клиентского сертификата.'
echo 'Экспорт пакета доступа: sudo /usr/bin/python3 -I /usr/local/libexec/atsd-security.py export "$USER"'
