#!/usr/bin/env python3
import asyncio
import json
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

DATA_DIR = Path(os.environ.get('ATSD_DATA', '/opt/atsd/data'))
POINTS_FILE = DATA_DIR / 'points.json'
ORDERS_FILE = DATA_DIR / 'orders.json'

DEFAULT_POINTS = [
    {'id': 'kpp',     'name': 'КПП',               'x': 2.0,  'y': 0.0, 'indoor': True},
    {'id': 'admin',   'name': 'Администрация',     'x': 18.0, 'y': 0.0, 'indoor': True},
    {'id': 'testing', 'name': 'Зона тестирования', 'x': 10.0, 'y': 6.0, 'indoor': True},
    {'id': 'depot',   'name': 'Склад',             'x': 0.0,  'y': 3.0, 'indoor': True},
]

DEFAULT_MAPS = {
    'indoor': {'image': '/static/maps/indoor.png', 'meters_per_pixel': 0.05,
               'origin': [-2.0, -2.0], 'label': 'Помещение'},
    'outdoor': {'image': '/static/maps/outdoor.jpg', 'meters_per_pixel': 0.35,
                'origin': [-40.0, -40.0], 'label': 'Территория'},
}

_store_lock = threading.Lock()


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
        'obstacle_bypassed': 'DETOUR_DONE' in kinds,
        'docked_on_mark': sum(1 for e in events if e['text'].startswith('DOCKED')),
        'zone_confirmed': sum(1 for e in events if e['text'].startswith('ZONE_OK')),
        'faults': [e['text'] for e in events if e['text'].split()[0] in
                   ('FAULT', 'STALL', 'BUMPER', 'LINK_LOST', 'GYRO_FAULT', 'MOTOR_MISSING',
                    'MOTOR_HOT', 'BATT_LOW', 'RECOVERY', 'LOCK_TAMPER', 'ESTOP', 'DOCK_NOT_FOUND',
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
    except Exception:
        pass

    link = RosLink(mock=mock, logger=print)
    link.start()
    print(f'АТСД-1М · веб-интерфейс на порту {port}{" · режим имитации" if mock else ""}')
    uvicorn.run(app, host='0.0.0.0', port=port, log_level='warning')


if __name__ == '__main__':
    main()
