# Грубое место по окрестным точкам Wi-Fi: Яндекс Локатор (с ключом) или beaconDB (без ключа).
# Точность 15–50 м: для проверки GPS на подмену, не для езды.
import json
import re

URL = {'yandex': 'https://locator.api.maps.yandex.ru/v1/locate?apikey={key}',
       'beacondb': 'https://api.beacondb.net/v1/geolocate'}


def parse_nmcli(text):
    """Вывод `nmcli -t -f BSSID,SIGNAL,SSID dev wifi list` -> [(bssid, сигнал %, ssid)].
    В режиме -t двоеточия внутри полей экранированы обратной косой."""
    out = []
    for line in text.splitlines():
        parts = [p.replace('\\:', ':') for p in re.split(r'(?<!\\):', line)]
        if len(parts) >= 3 and re.fullmatch(r'([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}', parts[0]):
            try:
                out.append((parts[0].lower(), int(parts[1]), ':'.join(parts[2:])))
            except ValueError:
                pass
    return out


def usable(aps):
    """Без скрытых сетей и *_nomap (их владельцы просили не класть в базы); раздачи телефонов
    сервисы сами отбрасывают — они ездят."""
    best = {}
    for bssid, sig, ssid in aps:
        if not ssid or ssid.endswith('_nomap'):
            continue
        if sig > best.get(bssid, -1):
            best[bssid] = sig
    return [(b, s / 2 - 100) for b, s in sorted(best.items(), key=lambda kv: -kv[1])]  # % -> дБм


def request_body(provider, aps):
    if provider == 'yandex':
        return json.dumps({'wifi': [{'bssid': b, 'signal_strength': round(d)} for b, d in aps]})
    return json.dumps({'considerIp': False,
                       'wifiAccessPoints': [{'macAddress': b, 'signalStrength': round(d)} for b, d in aps]})


def parse_response(provider, data):
    """Ответ сервиса -> (широта, долгота, точность м) или None, если место не найдено."""
    try:
        loc = data['location']
        if provider == 'yandex':
            return float(loc['point']['lat']), float(loc['point']['lon']), float(loc['accuracy'])
        if data.get('fallback'):
            return None  # по IP или соте — это километры, не нужно
        return float(loc['lat']), float(loc['lng']), float(data['accuracy'])
    except (KeyError, TypeError, ValueError):
        return None
