#!/usr/bin/env python3
import asyncio
import json
import math
import os
import secrets
import threading
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from atsd_web.ros_link import RosLink

DATA_DIR = Path(os.environ.get('ATSD_DATA', '/opt/atsd/data'))
POINTS_FILE = DATA_DIR / 'points.json'
ORDERS_FILE = DATA_DIR / 'orders.json'
ROADS_FILE = DATA_DIR / 'roads.json'

DEFAULT_POINTS = [
    {'id': 'kpp',     'name': 'КПП',               'x': 2.0,  'y': 0.0, 'indoor': True},
    {'id': 'admin',   'name': 'Администрация',     'x': 18.0, 'y': 0.0, 'indoor': True},
    {'id': 'testing', 'name': 'Зона тестирования', 'x': 10.0, 'y': 6.0, 'indoor': True},
    {'id': 'depot',   'name': 'Склад',             'x': 0.0,  'y': 3.0, 'indoor': True},
]

_store_lock = threading.Lock()


# Подложка — карта Яндекса вокруг origin графа дорог (начало координат карты робота).
# Static API 1.x отдаёт картинку без ключа; одна картинка 650×450, на z=17 это ~470×325 м
MAP_LAYERS = {'map': ('map', 'png', 'Схема'), 'sat': ('sat', 'jpg', 'Спутник')}
MAP_ZOOM = int(os.environ.get('ATSD_MAP_ZOOM', '17'))
MAP_W, MAP_H = 650, 450
_map_fail = {}  # слой -> когда не удалось скачать: без интернета не тормозим каждую загрузку страницы


def map_origin():
    """Origin из roads.json. Нет — берём текущее место робота (GPS, иначе Wi-Fi) и записываем:
    с ним nav_node начнёт переводить GPS и Wi-Fi в метры карты, road_mark продолжит тот же граф."""
    roads = _load(ROADS_FILE, {})
    if roads.get('origin'):
        return tuple(roads['origin'])
    t = link.snapshot() if link else {}
    g, w = t.get('gnss') or {}, t.get('wifi_geo')
    here = (g['lat'], g['lon']) if g.get('fix') else (w[0], w[1]) if w else None
    if here is None:
        return None
    roads.setdefault('nodes', {})
    roads.setdefault('edges', [])
    roads['origin'] = [round(here[0], 7), round(here[1], 7)]
    with _store_lock:
        _save(ROADS_FILE, roads)
    print(f'Начало координат карты по {"GPS" if g.get("fix") else "Wi-Fi"}: {roads["origin"]}')
    return tuple(roads['origin'])


def yandex_map(layer):
    """{'image', 'meters_per_pixel', 'origin' — левый нижний угол в метрах, 'label'} или None."""
    origin = map_origin()
    if origin is None:
        return None
    name, ext, label = MAP_LAYERS[layer]
    img = DATA_DIR / f'yandex_{layer}.{ext}'
    meta_file = DATA_DIR / f'yandex_{layer}.json'
    want = [origin[0], origin[1], MAP_ZOOM]
    if _load(meta_file, None) != want or not img.exists():
        if time.time() - _map_fail.get(layer, 0) < 60:
            return None
        url = (f'https://static-maps.yandex.ru/1.x/?ll={origin[1]:.7f},{origin[0]:.7f}'
               f'&z={MAP_ZOOM}&size={MAP_W},{MAP_H}&l={name}&lang=ru_RU')
        try:
            with urllib.request.urlopen(url, timeout=5) as r:
                data = r.read()
            if not data.startswith((b'\x89PNG', b'\xff\xd8')):
                raise ValueError('не картинка')
        except (OSError, ValueError) as e:
            _map_fail[layer] = time.time()
            print(f'Карта Яндекса ({label}) не скачалась: {e}')
            return None
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        img.write_bytes(data)
        _save(meta_file, want)
    # проекция Яндекса — Меркатор на эллипсоиде: метров в пикселе на широте origin
    lat = math.radians(origin[0])
    mpp = 2 * math.pi * 6378137.0 / (256 * 2 ** MAP_ZOOM) * math.cos(lat) / math.sqrt(1 - 0.00669438 * math.sin(lat) ** 2)
    return {'image': f'/api/map/{layer}.{ext}?v={int(img.stat().st_mtime)}', 'meters_per_pixel': mpp,
            'origin': [-MAP_W * mpp / 2, -MAP_H * mpp / 2], 'label': label}


def _load(path, fallback):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return fallback


def _save(path, data):
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    tmp.replace(path)


def now_iso():
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


class PointsIn(BaseModel):
    points: list


class PoseIn(BaseModel):
    x: float
    y: float
    yaw: float


class OrderIn(BaseModel):
    from_id: str
    to_id: str
    comment: str = ''


app = FastAPI(title='АТСД-1М', docs_url=None, redoc_url=None)

STATIC = Path(__file__).parent / 'static'
if not STATIC.exists():
    from ament_index_python.packages import get_package_share_directory
    STATIC = Path(get_package_share_directory('atsd_web')) / 'static'
app.mount('/static', StaticFiles(directory=str(STATIC)), name='static')

link: RosLink = None

STATUS_SEQ = ['new', 'to_pickup', 'at_pickup', 'in_transit', 'at_dropoff', 'delivered']
STATUS_RU = {
    'new': 'Принят', 'to_pickup': 'Едет за грузом', 'at_pickup': 'Ожидает погрузки',
    'in_transit': 'В пути', 'at_dropoff': 'Прибыл, ожидает выдачи', 'delivered': 'Выдан',
    'cancelled': 'Отменён',
}
BRAIN_TO_STATUS = {
    'TO_PICKUP': 'to_pickup', 'WAIT_LOAD': 'at_pickup', 'TO_DROP': 'in_transit',
    'WAIT_UNLOAD': 'at_dropoff', 'DONE': 'delivered',
}
PAUSE_RU = {
    'estop': 'аварийный стоп', 'no_pi_link': 'нет связи с бортовым компьютером',
    'stop_sign': 'знак «Стоп» — полная остановка', 'red_light': 'красный сигнал светофора',
    'obstacle': 'препятствие впереди', 'speed_limit_zero': 'остановлен оператором',
    'recovery': 'выход из нештатной ситуации',
}
CRUISE_SPEED = 0.5
START_GRACE_S = 3.0


def points():
    with _store_lock:
        return _load(POINTS_FILE, DEFAULT_POINTS)


def orders():
    with _store_lock:
        return _load(ORDERS_FILE, [])


def save_orders(data):
    with _store_lock:
        _save(ORDERS_FILE, data)


def point_by_id(pid):
    return next((p for p in points() if p['id'] == pid), None)


def active_order():
    return next((o for o in orders() if o['status'] not in ('delivered', 'cancelled')), None)


SHOW_FINISHED_S = 600


def display_order():
    order = active_order()
    if order:
        return order
    done = [o for o in orders() if o['status'] == 'delivered' and o.get('delivered')]
    if not done:
        return None
    last = done[-1]
    try:
        age = (datetime.now(timezone.utc) - datetime.fromisoformat(last['delivered'])).total_seconds()
    except ValueError:
        return None
    return last if age < SHOW_FINISHED_S else None


def advance(order, tele):
    m = tele.get('mission') or {}
    state = m.get('state')
    if not state or not m.get('link', True):
        return False
    if m.get('from') not in (None, order['from_id']) or m.get('to') not in (None, order['to_id']):
        return False

    changed = False
    fault = state == 'FAULT'
    if order.get('fault') != fault:
        order['fault'] = fault
        changed = True

    target = BRAIN_TO_STATUS.get(state)
    if target:
        order['brain_seen'] = True
        cur = STATUS_SEQ.index(order['status']) if order['status'] in STATUS_SEQ else 0
        new = STATUS_SEQ.index(target)
        if new > cur:
            order['status'] = target
            stamp = {'at_pickup': 'arrived_pickup', 'in_transit': 'loaded',
                     'at_dropoff': 'arrived_dropoff', 'delivered': 'delivered'}.get(target)
            if stamp:
                order[stamp] = now_iso()
            if target == 'delivered':
                order['mission_s'] = m.get('mission_s')
            changed = True
    elif state == 'IDLE' and order.get('brain_seen') and \
            time.time() - order.get('started_ts', 0) > START_GRACE_S:
        order['status'] = 'cancelled'
        order['cancel_reason'] = 'Рейс отменён на роботе'
        changed = True
    return changed


def order_view(order, tele):
    if order is None:
        return None
    frm, to = point_by_id(order['from_id']), point_by_id(order['to_id'])
    m = tele.get('mission') or {}
    remain = m.get('remain_m')
    speed = max(tele.get('speed') or 0.0, CRUISE_SPEED * 0.6)
    eta = int(remain / speed) if isinstance(remain, (int, float)) else None
    target = frm if order['status'] in ('new', 'to_pickup') else to
    return {
        **{k: v for k, v in order.items() if not k.endswith('_token')},
        'status_ru': STATUS_RU.get(order['status'], order['status']),
        'from_name': frm['name'] if frm else order['from_id'],
        'to_name': to['name'] if to else order['to_id'],
        'eta_pickup': eta if order['status'] in ('new', 'to_pickup') else 0,
        'eta_dropoff': eta if order['status'] == 'in_transit' else None,
        'target': target['id'] if target else None,
        'pause_ru': [PAUSE_RU.get(p, p) for p in m.get('pause', [])],
        'mission_s': m.get('mission_s', order.get('mission_s')),
        'brain_state': m.get('state'),
    }


@app.get('/')
def index():
    # Метка версии в ссылках: после обновления браузер не возьмёт старые app.js и style.css из кэша
    html = (STATIC / 'index.html').read_text(encoding='utf-8')
    for name in ('app.js', 'style.css'):
        html = html.replace(f'/static/{name}"', f'/static/{name}?v={int((STATIC / name).stat().st_mtime)}"')
    return HTMLResponse(html, headers={'Cache-Control': 'no-cache'})


@app.get('/api/config')
def api_config():
    maps = {k: m for k in MAP_LAYERS if (m := yandex_map(k))}
    roads = _load(ROADS_FILE, {})
    return {'points': points(), 'maps': maps,
            'roads': {'nodes': roads.get('nodes', {}), 'edges': roads.get('edges', [])}}


@app.get('/api/map/{layer}.{ext}')
def api_map_image(layer: str, ext: str):
    if layer not in MAP_LAYERS or MAP_LAYERS[layer][1] != ext:
        raise HTTPException(404, 'Карты нет')
    img = DATA_DIR / f'yandex_{layer}.{ext}'
    if not img.exists():
        raise HTTPException(404, 'Карты нет')
    return FileResponse(str(img), media_type='image/png' if ext == 'png' else 'image/jpeg')


@app.post('/api/points')
def api_points(body: PointsIn):
    for p in body.points:
        if not all(k in p for k in ('id', 'name', 'x', 'y')):
            raise HTTPException(400, 'Точке нужны id, name, x, y')
    with _store_lock:
        _save(POINTS_FILE, body.points)
    return {'ok': True, 'count': len(body.points)}


@app.post('/api/initialpose')
def api_initialpose(body: PoseIn):
    if not link.set_pose(body.x, body.y, body.yaw):
        raise HTTPException(503, 'Мост ROS не поднят')
    return {'ok': True}


@app.get('/api/orders')
def api_orders():
    return [{k: v for k, v in o.items() if not k.endswith('_token')} for o in orders()]


def require_robot():
    tele = link.snapshot()
    if not tele.get('brain_link') and not link.mock:
        raise HTTPException(503, 'Робот не на связи')
    return tele


@app.post('/api/orders')
def api_create(body: OrderIn):
    if body.from_id == body.to_id:
        raise HTTPException(400, 'Точка отправления и назначения совпадают')
    if not point_by_id(body.from_id) or not point_by_id(body.to_id):
        raise HTTPException(400, 'Неизвестная точка')
    if active_order():
        raise HTTPException(409, 'Уже есть активный заказ')
    tele = require_robot()
    m = tele.get('mission') or {}
    if m.get('state') in ('TO_PICKUP', 'WAIT_LOAD', 'TO_DROP', 'WAIT_UNLOAD', 'RETURN'):
        raise HTTPException(409, 'Робот выполняет другой рейс')
    if (tele.get('road') or {}).get('ready') is False:
        # иначе nav_node через 20 с отменит рейс с no_position
        raise HTTPException(409, 'Робот не знает, где он: GPS нет. Нажмите «Робот здесь» и покажите на карте, '
                                 'где он стоит и куда смотрит')

    order = {
        'id': secrets.token_hex(4), 'from_id': body.from_id, 'to_id': body.to_id,
        'comment': body.comment, 'status': 'to_pickup', 'created': now_iso(),
        'started_ts': time.time(),
        'sender_token': secrets.token_urlsafe(8), 'receiver_token': secrets.token_urlsafe(8),
    }
    pts = {p['id']: (p['x'], p['y']) for p in points()}
    link.start_mission(body.from_id, body.to_id, pts)
    data = orders()
    data.append(order)
    save_orders(data)
    return order


def _find(oid):
    data = orders()
    for o in data:
        if o['id'] == oid:
            return data, o
    raise HTTPException(404, 'Заказ не найден')


@app.get('/api/orders/{oid}')
def api_order(oid: str):
    _, o = _find(oid)
    return {k: v for k, v in o.items() if not k.endswith('_token')}


@app.post('/api/orders/{oid}/unlock')
def api_unlock(oid: str, role: str = 'sender'):
    data, order = _find(oid)
    allowed = {'sender': ('at_pickup',), 'receiver': ('at_dropoff',)}
    if order['status'] not in allowed.get(role, ()):
        raise HTTPException(409, 'Сейчас отсек открывать нельзя')
    ok, msg = link.unlock()
    if not ok:
        raise HTTPException(503, msg)
    order.setdefault('unlocks', []).append({'role': role, 'at': now_iso()})
    save_orders(data)
    return {'ok': True, 'message': msg}


@app.post('/api/orders/{oid}/done')
def api_done(oid: str, role: str = 'sender'):
    _, order = _find(oid)
    ok_role = (role == 'sender' and order['status'] == 'at_pickup') or \
              (role == 'receiver' and order['status'] == 'at_dropoff')
    if not ok_role:
        raise HTTPException(409, 'Действие сейчас недоступно')
    tele = require_robot()
    if not tele['lock']['closed']:
        raise HTTPException(409, 'Закройте крышку отсека')
    link.cargo()
    return {'ok': True}


@app.post('/api/orders/{oid}/cancel')
def api_cancel(oid: str):
    data, order = _find(oid)
    link.abort()
    order['status'] = 'cancelled'
    order['cancel_reason'] = 'Отменён оператором'
    save_orders(data)
    return {'ok': True}


@app.post('/api/mission/resume')
def api_resume():
    require_robot()
    if not link.resume():
        raise HTTPException(409, 'Продолжение недоступно')
    return {'ok': True}


@app.get('/api/report/{oid}')
def api_report(oid: str):
    _, o = _find(oid)
    events = [e for e in link.snapshot()['events']]
    kinds = {e['text'].split()[0] for e in events}
    return {
        'order': {k: v for k, v in o.items() if not k.endswith('_token')},
        'total_time_s': o.get('mission_s'),
        'delivered': o['status'] == 'delivered',
        'sign_recognized': bool(kinds & {'STOP_SIGN', 'CROSSWALK_SLOW', 'BUMP_SLOW'}),
        'traffic_light_recognized': 'RED_LIGHT_STOP' in kinds or 'GREEN_GO' in kinds,
        'obstacle_detected': 'OBSTACLE_STOP' in kinds,
        'obstacle_bypassed': 'OBSTACLE_CLEAR' in kinds,
        'zone_confirmed': sum(1 for e in events if e['text'].startswith('ZONE_OK')),
        'faults': [e['text'] for e in events if e['text'].split()[0] in
                   ('FAULT', 'STALL', 'BUMPER', 'LINK_LOST', 'GYRO_FAULT', 'MOTOR_MISSING',
                    'MOTOR_HOT', 'BATT_LOW', 'RECOVERY', 'LOCK_TAMPER', 'ESTOP', 'NAV_FAILED',
                    'ZONE_MISMATCH', 'ZONE_UNCONFIRMED', 'ZONE_UNCONFIRMED_UNLOCK')],
        'events': events,
    }


@app.websocket('/ws')
async def ws(sock: WebSocket):
    await sock.accept()
    try:
        while True:
            tele = link.snapshot()
            order = active_order()
            if order is None:
                order = display_order()
            elif order:
                data = orders()
                for o in data:
                    if o['id'] == order['id']:
                        if advance(o, tele):
                            save_orders(data)
                        order = o
                        break
            events = tele.pop('events', [])
            await sock.send_json({
                'ts': time.time(),
                'telemetry': tele,
                'order': order_view(order, tele),
                'points': points(),
                'events': events,
            })
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        pass
    except Exception:
        pass


def main(args=None):
    global link
    mock = os.environ.get('ATSD_MOCK', '').lower() in ('1', 'true', 'yes')
    port = int(os.environ.get('ATSD_PORT', '8080'))
    debug_port = int(os.environ.get('ATSD_DEBUG_PORT', '8090'))
    try:
        import rclpy
        from rclpy.node import Node
        rclpy.init(args=args)
        cfg = Node('web_config')
        cfg.declare_parameter('mock', mock)
        cfg.declare_parameter('port', port)
        cfg.declare_parameter('debug_port', debug_port)
        mock = bool(cfg.get_parameter('mock').value)
        port = int(cfg.get_parameter('port').value)
        debug_port = int(cfg.get_parameter('debug_port').value)
        cfg.destroy_node()
    except Exception:
        pass

    if not POINTS_FILE.exists():
        _save(POINTS_FILE, DEFAULT_POINTS)  # nav_node берёт координаты точек отсюда
    link = RosLink(mock=mock, logger=print)
    link.start()
    print(f'АТСД-1М · веб-интерфейс на порту {port}{" · режим имитации" if mock else ""}')
    if debug_port:
        # отдельный сервер в своём потоке: сигналы остановки ловит основной, этот уходит вместе с процессом
        from atsd_web.debug import attach, debug_app
        attach(link, STATIC, api_config, api_map_image)
        threading.Thread(target=uvicorn.run, args=(debug_app,), daemon=True,
                         kwargs={'host': '0.0.0.0', 'port': debug_port, 'log_level': 'warning'}).start()
        print(f'Отладочная страница: порт {debug_port}')
    uvicorn.run(app, host='0.0.0.0', port=port, log_level='warning')


if __name__ == '__main__':
    main()
