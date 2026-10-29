# Грубое место по окрестным точкам Wi-Fi: Яндекс Локатор (с ключом) или beaconDB (без ключа).
# Точность 15–50 м: для проверки GPS на подмену, не для езды.
import datetime
import json
import re
import urllib.parse

# yandex — Локатор api.lbs.yandex.net (ключи «API Локатора» из кабинета работают здесь),
# yandex_v1 — новая версия; ей ключ из кабинета отвечал 403
URL = {'yandex': 'https://api.lbs.yandex.net/geolocation',
       'yandex_v1': 'https://locator.api.maps.yandex.ru/v1/locate?apikey={key}',
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


def parse_iw(text):
    """Вывод `iw dev wlan0 scan dump` (всё, что слышал драйвер) -> [(bssid, сигнал %, ssid)].
    NetworkManager показывает не все из них, поэтому читаем и напрямую."""
    out, cur = [], None
    for line in text.splitlines():
        m = re.match(r'BSS ([0-9a-fA-F:]{17})', line)
        if m:
            cur = [m.group(1).lower(), 0, '']
            out.append(cur)
        elif cur is not None:
            t = line.strip()
            if t.startswith('signal:'):
                try:
                    cur[1] = max(0, min(100, int(2 * (float(t.split()[1]) + 100))))  # дБм -> %
                except (IndexError, ValueError):
                    pass
            elif t.startswith('SSID:'):
                cur[2] = t[5:].strip()
    return [tuple(x) for x in out]


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


def request_body(provider, aps, key=''):
    """-> (тело запроса, Content-Type)."""
    if provider == 'yandex':
        data = {'common': {'version': '1.0', 'api_key': key},
                'wifi_networks': [{'mac': b.replace(':', '-').upper(), 'signal_strength': round(d)} for b, d in aps]}
        return urllib.parse.urlencode({'json': json.dumps(data)}), 'application/x-www-form-urlencoded'
    if provider == 'yandex_v1':
        return json.dumps({'wifi': [{'bssid': b, 'signal_strength': round(d)} for b, d in aps]}), 'application/json'
    return json.dumps({'considerIp': False, 'wifiAccessPoints': [
        {'macAddress': b, 'signalStrength': round(d)} for b, d in aps]}), 'application/json'


def parse_response(provider, data):
    """Ответ сервиса -> (широта, долгота, точность м) или None, если место не найдено."""
    try:
        if provider == 'yandex':
            pos = data['position']
            if pos.get('type') == 'ip':
                return None  # по IP — это город, не нужно
            return float(pos['latitude']), float(pos['longitude']), float(pos['precision'])
        loc = data['location']
        if provider == 'yandex_v1':
            return float(loc['point']['lat']), float(loc['point']['lon']), float(loc['accuracy'])
        if data.get('fallback'):
            return None  # по IP или соте — это километры, не нужно
        return float(loc['lat']), float(loc['lng']), float(data['accuracy'])
    except (KeyError, TypeError, ValueError):
        return None


def same_networks(a, b, need=0.6):
    """Те же точки, что при прошлом запросе (доля общих от всех) — значит, робот почти на том же
    месте: для Wi-Fi с точностью ±30 м это так. Запрос не тратим."""
    return bool(a) and bool(b) and len(a & b) / len(a | b) >= need


class Budget:
    """Сколько запросов к платному/лимитному сервису осталось сегодня. Счёт в файле — переживает перезапуск."""

    def __init__(self, path, limit):
        self.path, self.limit = path, limit
        try:
            with open(path, encoding='utf-8') as f:
                self.day, self.used = json.load(f)
        except (OSError, ValueError, TypeError):
            self.day, self.used = '', 0

    def take(self, today=None):
        today = today or datetime.date.today().isoformat()
        if today != self.day:
            self.day, self.used = today, 0
        if self.used >= self.limit:
            return False
        self.used += 1
        try:
            with open(self.path, 'w', encoding='utf-8') as f:
                json.dump([self.day, self.used], f)
        except OSError:
            pass  # не записали — посчитаем заново после перезапуска, не страшно
        return True
