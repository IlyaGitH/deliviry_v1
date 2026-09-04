#!/usr/bin/env python3
"""
atsd_web · server

Веб-интерфейс заказов АТСД-1М.

    REST
      GET  /api/config              карты и точки
      POST /api/points              сохранить точки (админ)
      GET  /api/orders              список заказов
      POST /api/orders              создать заказ
      GET  /api/orders/{id}         заказ по идентификатору
      POST /api/orders/{id}/unlock  открыть отсек
      POST /api/orders/{id}/done    подтвердить погрузку или выдачу
      POST /api/orders/{id}/cancel  отменить

    WS
      /ws                           телеметрия и состояние, 5 Гц

СЦЕНАРИЙ ДОСТАВКИ
    new → to_pickup → at_pickup → in_transit → at_dropoff → delivered

Отправитель получает ссылку с ролью sender, получатель — receiver.
У каждого своя кнопка «Открыть отсек» и своя кнопка «Готово»:
первая открывает замок повторно, если надо догрузить, вторая
подтверждает и двигает сценарий дальше.

Запуск:
    ros2 launch atsd_web web.launch.py            с роботом
    ros2 launch atsd_web web.launch.py mock:=true без робота
"""

import asyncio
import json
import math
import os
import secrets
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from atsd_web.ros_link import RosLink

# ─────────────────────────────────────────── хранилище
DATA_DIR = Path(os.environ.get('ATSD_DATA', '/opt/atsd/data'))
POINTS_FILE = DATA_DIR / 'points.json'
ORDERS_FILE = DATA_DIR / 'orders.json'

DEFAULT_POINTS = [
    {'id': 'kpp',     'name': 'КПП',              'x': 0.0,  'y': 0.0, 'indoor': True},
    {'id': 'admin',   'name': 'Администрация',    'x': 12.0, 'y': 0.0, 'indoor': True},
    {'id': 'testing', 'name': 'Зона тестирования','x': 12.0, 'y': 8.0, 'indoor': True},
    {'id': 'depot',   'name': 'Склад',            'x': 0.0,  'y': 8.0, 'indoor': True},
]

# Подложки карт. Координаты в метрах, отсчёт от левого нижнего угла
# изображения. meters_per_pixel считается по известному расстоянию
# между двумя объектами на снимке.
DEFAULT_MAPS = {
    'indoor': {
        'image': '/static/maps/indoor.png',
        'meters_per_pixel': 0.05,
        'origin': [-2.0, -2.0],
        'label': 'Помещение',
    },
    'outdoor': {
        'image': '/static/maps/outdoor.jpg',
        'meters_per_pixel': 0.35,
        'origin': [-40.0, -40.0],
        'label': 'Территория',
    },
}

_store_lock = threading.Lock()


def _load(path, fallback):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except Exception:                                # noqa: BLE001
        return fallback


def _save(path, data):
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    tmp.replace(path)


def now_iso():
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


# ─────────────────────────────────────────── модели
class PointsIn(BaseModel):
    points: list


class OrderIn(BaseModel):
    from_id: str
    to_id: str
    comment: str = ''


# ─────────────────────────────────────────── приложение
app = FastAPI(title='АТСД-1М', docs_url=None, redoc_url=None)

STATIC = Path(__file__).parent / 'static'
if not STATIC.exists():                              # установленный пакет
    from ament_index_python.packages import get_package_share_directory
    STATIC = Path(get_package_share_directory('atsd_web')) / 'static'

app.mount('/static', StaticFiles(directory=str(STATIC)), name='static')

link: RosLink = None
STATUS_ORDER = ['new', 'to_pickup', 'at_pickup', 'in_transit', 'at_dropoff', 'delivered']
STATUS_RU = {
    'new':        'Принят',
    'to_pickup':  'Едет за грузом',
    'at_pickup':  'Ожидает погрузки',
    'in_transit': 'В пути',
    'at_dropoff': 'Прибыл, ожидает выдачи',
    'delivered':  'Выдан',
    'cancelled':  'Отменён',
}

ARRIVE_RADIUS = 1.5        # метров до точки, считаем что приехал
CRUISE_SPEED = 0.5         # м/с для оценки времени, когда робот стоит


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
    for p in points():
        if p['id'] == pid:
            return p
    return None


def active_order():
    for o in orders():
        if o['status'] not in ('delivered', 'cancelled'):
            return o
    return None


def eta_seconds(pose, target, speed):
    if target is None:
        return None
    dist = math.hypot(target['x'] - pose['x'], target['y'] - pose['y'])
    v = max(speed, CRUISE_SPEED * 0.6)
    return int(dist / v)


def advance(order, tele):
    """Двигает сценарий по факту прибытия робота."""
    changed = False
    pose = tele['pose']

    if order['status'] == 'new':
        order['status'] = 'to_pickup'
        link.send_goal(order['from_id'])
        changed = True

    target_id = order['from_id'] if order['status'] in ('new', 'to_pickup') else order['to_id']
    target = point_by_id(target_id)

    if target:
        dist = math.hypot(target['x'] - pose['x'], target['y'] - pose['y'])
        if order['status'] == 'to_pickup' and dist < ARRIVE_RADIUS:
            order['status'] = 'at_pickup'
            order['arrived_pickup'] = now_iso()
            changed = True
        elif order['status'] == 'in_transit' and dist < ARRIVE_RADIUS:
            order['status'] = 'at_dropoff'
            order['arrived_dropoff'] = now_iso()
            changed = True
    return changed


def order_view(order, tele):
    if order is None:
        return None
    frm = point_by_id(order['from_id'])
    to = point_by_id(order['to_id'])
    target = frm if order['status'] in ('new', 'to_pickup') else to
    return {
        **order,
        'status_ru': STATUS_RU.get(order['status'], order['status']),
        'from_name': frm['name'] if frm else order['from_id'],
        'to_name': to['name'] if to else order['to_id'],
        'eta_pickup': eta_seconds(tele['pose'], frm, tele['speed'])
                      if order['status'] in ('new', 'to_pickup') else 0,
        'eta_dropoff': eta_seconds(tele['pose'], to, tele['speed'])
                       if order['status'] in ('in_transit',) else None,
        'target': target['id'] if target else None,
    }


# ─────────────────────────────────────────── REST
@app.get('/')
def index():
    return FileResponse(str(STATIC / 'index.html'))


@app.get('/api/config')
def api_config():
    return {'points': points(), 'maps': DEFAULT_MAPS}


@app.post('/api/points')
def api_points(body: PointsIn):
    for p in body.points:
        if not all(k in p for k in ('id', 'name', 'x', 'y')):
            raise HTTPException(400, 'Точке нужны id, name, x, y')
    with _store_lock:
        _save(POINTS_FILE, body.points)
    return {'ok': True, 'count': len(body.points)}


@app.get('/api/orders')
def api_orders():
    return orders()


@app.post('/api/orders')
def api_create(body: OrderIn):
    if body.from_id == body.to_id:
        raise HTTPException(400, 'Точка отправления и назначения совпадают')
    if not point_by_id(body.from_id) or not point_by_id(body.to_id):
        raise HTTPException(400, 'Неизвестная точка')
    if active_order():
        raise HTTPException(409, 'Уже есть активный заказ')

    order = {
        'id': secrets.token_hex(4),
        'from_id': body.from_id,
        'to_id': body.to_id,
        'comment': body.comment,
        'status': 'new',
        'created': now_iso(),
        'sender_token': secrets.token_urlsafe(8),
        'receiver_token': secrets.token_urlsafe(8),
    }
    data = orders()
    data.append(order)
    save_orders(data)
    link.send_goal(order['from_id'])
    order['status'] = 'to_pickup'
    save_orders(data)
    return order


@app.get('/api/orders/{oid}')
def api_order(oid: str):
    for o in orders():
        if o['id'] == oid:
            return o
    raise HTTPException(404, 'Заказ не найден')


def _find(oid):
    data = orders()
    for o in data:
        if o['id'] == oid:
            return data, o
    raise HTTPException(404, 'Заказ не найден')


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
    data, order = _find(oid)
    if role == 'sender' and order['status'] == 'at_pickup':
        order['status'] = 'in_transit'
        order['loaded'] = now_iso()
        link.send_goal(order['to_id'])
    elif role == 'receiver' and order['status'] == 'at_dropoff':
        order['status'] = 'delivered'
        order['delivered'] = now_iso()
    else:
        raise HTTPException(409, 'Действие сейчас недоступно')
    save_orders(data)
    return {'ok': True, 'status': order['status']}


@app.post('/api/orders/{oid}/cancel')
def api_cancel(oid: str):
    data, order = _find(oid)
    order['status'] = 'cancelled'
    save_orders(data)
    return {'ok': True}


# ─────────────────────────────────────────── WebSocket
@app.websocket('/ws')
async def ws(sock: WebSocket):
    await sock.accept()
    try:
        while True:
            tele = link.snapshot()
            order = active_order()
            if order:
                data = orders()
                for o in data:
                    if o['id'] == order['id'] and advance(o, tele):
                        save_orders(data)
                        order = o
                        break

            await sock.send_json({
                'ts': time.time(),
                'telemetry': tele,
                'order': order_view(order, tele),
                'points': points(),
            })
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        pass
    except Exception:                                # noqa: BLE001
        pass


# ─────────────────────────────────────────── запуск
def main(args=None):
    global link

    mock = os.environ.get('ATSD_MOCK', '').lower() in ('1', 'true', 'yes')
    port = int(os.environ.get('ATSD_PORT', '8080'))

    try:
        import rclpy
        from rclpy.node import Node
        rclpy.init(args=args)
        cfg = Node('web_config')
        cfg.declare_parameter('mock', mock)
        cfg.declare_parameter('port', port)
        mock = bool(cfg.get_parameter('mock').value)
        port = int(cfg.get_parameter('port').value)
        cfg.destroy_node()
    except Exception:                                # noqa: BLE001
        pass

    link = RosLink(mock=mock, logger=print)
    link.start()

    print(f'АТСД-1М · веб-интерфейс на порту {port}'
          f'{" · режим имитации" if mock else ""}')
    uvicorn.run(app, host='0.0.0.0', port=port, log_level='warning')


if __name__ == '__main__':
    main()
