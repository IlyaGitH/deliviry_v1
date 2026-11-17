#!/usr/bin/env python3
import asyncio
import json
import hashlib
import math
import os
import secrets
import threading
import time
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from starlette.concurrency import run_in_threadpool

from atsd_web.ros_link import RosLink
from atsd_web import road_map
from atsd_web.secure_runtime import Security, SecurityMiddleware, COOKIE
from atsd_web.security import AccessDenied
from atsd_drive.road_logic import Roads

security = None

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

_store_lock = threading.RLock()


def _load(path, fallback):
    try:
        raw = path.read_bytes()
    except FileNotFoundError:
        return fallback
    if security is not None and path == ORDERS_FILE:
        return json.loads(security.cipher.decrypt(raw))
    try:
        return json.loads(raw)
    except (ValueError, UnicodeError):
        return fallback


def _save(path, data):
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    raw = json.dumps(data, ensure_ascii=False, indent=2).encode()
    if security is not None and path == ORDERS_FILE:
        raw = security.cipher.encrypt(raw)
    tmp = path.with_suffix('.tmp')
    with tmp.open('wb') as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())
    tmp.chmod(0o600)
    tmp.replace(path)
    descriptor = os.open(path.parent, os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def now_iso():
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


class PointsIn(BaseModel):
    points: list


class PoseIn(BaseModel):
    x: float
    y: float
    yaw: float


class RoadsIn(BaseModel):
    roads: dict
    revision: str


class RoadImportIn(BaseModel):
    revision: str
    origin: list[float] | None = None
    radius_m: float = 600.


class RouteIn(BaseModel):
    start: list[float]
    goal: list[float]


class OrderIn(BaseModel):
    from_id: str
    to_id: str
    comment: str = ''


async def server_heartbeat():
    while True:
        if security is not None and link is not None:
            link.server_alive()
        await asyncio.sleep(.5)


@asynccontextmanager
async def lifespan(app):
    task = asyncio.create_task(server_heartbeat())
    try:
        yield
    finally:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


app = FastAPI(title='АТСД-1М', docs_url=None, redoc_url=None, openapi_url=None, lifespan=lifespan)
app.add_middleware(SecurityMiddleware, provider=lambda: security)
app.add_middleware(GZipMiddleware, minimum_size=1000)

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
    'order_server': 'нет подтверждений от сервера заказов',
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
        **{k: v for k, v in order.items() if '_token' not in k and not k.endswith('_certificate')},
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


def action_role(request, requested):
    if security is None:
        return requested
    session = request.state.security_session
    if session.principal.role == 'admin':
        return requested
    if requested != session.principal.role:
        raise HTTPException(403, 'Подмена роли запрещена')
    return session.principal.role


@app.get('/login')
def login_page():
    return HTMLResponse((STATIC / 'login.html').read_text(), headers={'Cache-Control':'no-store'})


@app.post('/api/security/login')
def login(body: dict, request: Request):
    if security is None:
        return {'role':'admin'}
    role = body.get('role', 'admin')
    if role not in ('admin', 'sender', 'receiver'):
        raise AccessDenied('Неизвестная роль')
    secret = str(body.get('secret', ''))
    if len(secret) > 256:
        raise HTTPException(400, 'Некорректные данные')
    with _store_lock:
        order = None
        data = None
        if role != 'admin':
            data, order = _find(str(body.get('order', '')))
            bound = order.get(role+'_certificate')
            if bound and bound != request.state.certificate:
                raise AccessDenied('Доступ к заказу привязан к другому устройству')
        token = security.login(request.state.certificate, role, secret, order)
        if order is not None:
            order[role+'_certificate'] = request.state.certificate
            save_orders(data)
    response = JSONResponse({'role':role, 'order':order['id'] if order else None})
    response.set_cookie(COOKIE, token, max_age=12*3600, secure=True, httponly=True,
                        samesite='strict', path='/')
    return response


@app.get('/api/security/me')
def security_me(request: Request):
    if security is None:
        return {'role':'admin', 'order':None, 'secure':False}
    session = request.state.security_session
    return {'role':session.principal.role, 'order':session.order, 'secure':True}


@app.post('/api/security/ticket')
def security_ticket(body: dict, request: Request):
    session = request.state.security_session
    target = str(body.get('target', ''))
    security.authorize(session, 'POST', target.split('?',1)[0])
    return {'ticket':security.tickets.issue(session.certificate, 'POST', target,
                                           str(body.get('body_hash', '')))}


@app.post('/api/security/logout')
def logout(request: Request):
    security.logout(request.headers)
    response = JSONResponse({'ok':True})
    response.delete_cookie(COOKIE, path='/', secure=True, httponly=True, samesite='strict')
    return response


@app.get('/')
def index():
    # Метка версии в ссылках: после обновления браузер не возьмёт старые app.js и style.css из кэша
    html = (STATIC / 'index.html').read_text(encoding='utf-8')
    for name in ('app.js', 'road_editor.js', 'grid100.js', 'style.css'):
        html = html.replace(f'/static/{name}"', f'/static/{name}?v={int((STATIC / name).stat().st_mtime)}"')
    return HTMLResponse(html, headers={'Cache-Control': 'no-cache'})


@app.get('/api/config')
def api_config():
    pts = points()
    valid = [(p['x'], p['y']) for p in pts if all(isinstance(p.get(k), (int, float)) and math.isfinite(p[k]) for k in ('x','y'))]
    center = [round((min(v[i] for v in valid)+max(v[i] for v in valid))/10)*5 for i in (0,1)] if valid else [0,0]
    roads = _load(ROADS_FILE, {})
    return {'points': pts, 'maps': {}, 'grid': {'size_m':100, 'step_m':5, 'center':center},
            'roads': roads, 'roads_revision': road_revision(roads)}


def road_revision(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


def road_changes_allowed():
    tele=link.snapshot() if link else {}
    if active_order() or (tele.get('mission') or {}).get('state') in ('TO_PICKUP','WAIT_LOAD','TO_DROP','WAIT_UNLOAD','RETURN'):
        raise HTTPException(409, 'Завершите рейс перед изменением дорог или точек доставки')


def save_roads(data, revision):
    try:
        clean=road_map.validate(data)
    except (ValueError,TypeError,KeyError,IndexError) as e:
        raise HTTPException(400,str(e))
    with _store_lock:
        road_changes_allowed()
        current=_load(ROADS_FILE,{})
        if road_revision(current)!=revision:
            raise HTTPException(409,'Карта изменилась. Обновите страницу перед сохранением')
        if current.get('origin') and clean.get('origin')!=current['origin']:
            raise HTTPException(409,'Сохраните прежний центр карты: к нему привязаны точки и позиция робота')
        if current:
            _save(DATA_DIR / ('roads-backup-'+str(time.time_ns())+'.json'),current)
        _save(ROADS_FILE,clean)
    return {'roads':clean,'roads_revision':road_revision(clean)}


@app.post('/api/roads')
def api_roads(body: RoadsIn):
    return save_roads(body.roads,body.revision)


@app.post('/api/roads/import')
def api_import_roads(body: RoadImportIn):
    raise HTTPException(410, 'Географический импорт отключён. Отметьте дороги на локальной сетке.')


def plan_road(start,goal):
    if len(start)!=2 or len(goal)!=2:
        raise HTTPException(400,'Нужны координаты x, y')
    try:
        graph=Roads.from_data(road_map.validate(_load(ROADS_FILE,{})))
        path=graph.route(start,goal)
    except (ValueError,TypeError,KeyError,IndexError):
        raise HTTPException(400,'Некорректные координаты или граф дорог')
    if path is None:
        raise HTTPException(409,'Нет пути по дорогам: проверьте соединения и расположите точки не дальше 3 м от дороги')
    return {'path':path,'distance_m':sum(math.dist(a,b) for a,b in zip(path,path[1:])),
            'source':'roads'}


@app.post('/api/route')
def api_route(body: RouteIn):
    return plan_road(body.start,body.goal)


@app.get('/api/map/{layer}.{ext}')
def api_map_image(layer: str, ext: str):
    raise HTTPException(410, 'Географическая карта отключена. Используется сетка 100 × 100 м.')


@app.post('/api/points')
def api_points(body: PointsIn):
    for p in body.points:
        if not all(k in p for k in ('id', 'name', 'x', 'y')):
            raise HTTPException(400, 'Точке нужны id, name, x, y')
    with _store_lock:
        road_changes_allowed()
        _save(POINTS_FILE, body.points)
    return {'ok': True, 'count': len(body.points)}


@app.post('/api/initialpose')
def api_initialpose(body: PoseIn):
    if not link.set_pose(body.x, body.y, body.yaw):
        raise HTTPException(400, 'Координаты и угол должны быть конечными числами')
    return {'ok': True}


@app.get('/api/orders')
def api_orders():
    return [{k: v for k, v in o.items() if '_token' not in k and not k.endswith('_certificate')} for o in orders()]


def require_robot():
    tele = link.snapshot()
    if not tele.get('brain_link') and not link.mock:
        raise HTTPException(503, 'Робот не на связи')
    return tele


@app.post('/api/orders')
def api_create(body: OrderIn):
    with _store_lock:
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
            raise HTTPException(409, 'Укажите начальную позицию: нажмите «Робот здесь» и покажите на сетке, '
                                     'где он стоит и куда смотрит')

        pts = {p['id']: (p['x'], p['y']) for p in points()}
        pose=tele.get('pose') or {}
        if 'x' not in pose or 'y' not in pose:
            raise HTTPException(409,'Укажите позицию робота на карте')
        routes=[plan_road((pose['x'],pose['y']),pts[body.from_id])['path'],
                plan_road(pts[body.from_id],pts[body.to_id])['path']]

        order = {
            'id': secrets.token_hex(4), 'from_id': body.from_id, 'to_id': body.to_id,
            'comment': body.comment[:500], 'status': 'to_pickup', 'created': now_iso(),
            'started_ts': time.time(),
            'sender_token': secrets.token_urlsafe(32), 'receiver_token': secrets.token_urlsafe(32),
        }
        if not link.start_mission(body.from_id, body.to_id, pts, routes=routes):
            raise HTTPException(503,'Не удалось передать маршрут роботу')
        result = dict(order)
        if security is not None:
            for role in ('sender', 'receiver'):
                order[role+'_token_hash'] = hashlib.sha256(order.pop(role+'_token').encode()).hexdigest()
        data = orders()
        data.append(order)
        save_orders(data)
        return result


def _find(oid):
    data = orders()
    for o in data:
        if o['id'] == oid:
            return data, o
    raise HTTPException(404, 'Заказ не найден')


@app.get('/api/orders/{oid}')
def api_order(oid: str):
    _, o = _find(oid)
    return {k: v for k, v in o.items() if '_token' not in k and not k.endswith('_certificate')}


@app.post('/api/orders/{oid}/access')
def api_rotate_access(oid: str):
    with _store_lock:
        data, order = _find(oid)
        if order['status'] in ('delivered', 'cancelled'):
            raise HTTPException(409, 'Заказ завершён')
        result = {'id': oid}
        order['access_issued_ts'] = time.time()
        for role in ('sender', 'receiver'):
            token = secrets.token_urlsafe(32)
            result[role+'_token'] = token
            order[role+'_token_hash'] = hashlib.sha256(token.encode()).hexdigest()
            order.pop(role+'_token', None)
            order.pop(role+'_certificate', None)
        save_orders(data)
        if security is not None:
            with security.lock:
                security.sessions = {k: s for k, s in security.sessions.items() if s.order != oid}
        return result


@app.post('/api/orders/{oid}/unlock')
def api_unlock(oid: str, role: str = 'sender', request: Request = None):
    role = action_role(request, role)
    data, order = _find(oid)
    allowed = {'sender': ('at_pickup',), 'receiver': ('at_dropoff',)}
    if order['status'] not in allowed.get(role, ()):
        raise HTTPException(409, 'Сейчас отсек открывать нельзя')
    ok, msg = link.unlock()
    if not ok:
        raise HTTPException(503, msg)
    with _store_lock:
        # The ROS service takes seconds. Preserve concurrent status/certificate updates.
        data, order = _find(oid)
        order.setdefault('unlocks', []).append({'role': role, 'at': now_iso()})
        save_orders(data)
    return {'ok': True, 'message': msg}


@app.post('/api/orders/{oid}/done')
def api_done(oid: str, role: str = 'sender', request: Request = None):
    role = action_role(request, role)
    _, order = _find(oid)
    ok_role = (role == 'sender' and order['status'] == 'at_pickup') or \
              (role == 'receiver' and order['status'] == 'at_dropoff')
    if not ok_role:
        raise HTTPException(409, 'Действие сейчас недоступно')
    tele = require_robot()
    if not tele['lock']['closed']:
        raise HTTPException(409, 'Закройте крышку отсека')
    with _store_lock:
        data, current = _find(oid)
        if current['status'] != order['status'] or current.get(role + '_done'):
            raise HTTPException(409, 'Действие уже выполнено или этап изменился')
        current[role + '_done'] = True
        save_orders(data)
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
        'order': {k: v for k, v in o.items() if '_token' not in k and not k.endswith('_certificate')},
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


def status_payload():
    """Disk access and order transitions run outside the ASGI event loop."""
    # Multiple viewers must not advance/write the same order concurrently.
    with _status_lock:
        tele = link.snapshot()
        order = active_order()
        if order is None:
            order = display_order()
        else:
            data = orders()
            for o in data:
                if o['id'] == order['id']:
                    if advance(o, tele):
                        save_orders(data)
                    order = o
                    break
        events = tele.pop('events', [])
        return {'ts': time.time(), 'telemetry': tele,
                'order': order_view(order, tele), 'points': points(), 'events': events}


_status_lock = threading.Lock()


@app.websocket('/ws')
async def ws(sock: WebSocket):
    await sock.accept()
    seq = 0
    try:
        while True:
            payload = await run_in_threadpool(status_payload)
            if security is not None:
                try:
                    session = security.session(sock.headers)
                    payload = security.payload(session, payload)
                except AccessDenied:
                    await sock.close(code=4403)
                    return
            # History on connection, then only new events; slow clients never queue snapshots.
            events = [e for e in payload['events'] if e['seq'] > seq]
            payload['events'] = events
            await asyncio.wait_for(sock.send_json(payload), timeout=3.0)
            if events:
                seq = events[-1]['seq']
            await asyncio.sleep(0.2)
    except (WebSocketDisconnect, RuntimeError):
        pass
    except asyncio.TimeoutError:
        await sock.close(code=1013)


def main(args=None):
    global link, security
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

    if not mock:
        config = json.loads(Path('/etc/atsd/security/web.json').read_text())
        security = Security(config, '/run/atsd-web/tickets.db')
    if not POINTS_FILE.exists():
        _save(POINTS_FILE, DEFAULT_POINTS)  # nav_node берёт координаты точек отсюда
    link = RosLink(mock=mock, logger=print)
    if security is not None:
        link.security_audit = security.record_event
    link.start()
    print(f'АТСД-1М · имитация: http://127.0.0.1:{port}' if mock else
          'АТСД-1М · защищённый веб: Unix-сокет /run/atsd-web/http.sock, HTTPS через шлюз')
    from atsd_web.debug import attach, debug_app
    attach(link, STATIC, api_config, api_map_image, lambda: security)
    app.mount('/debug', debug_app)
    if mock:
        uvicorn.run(app, host='127.0.0.1', port=port, log_level='warning')
    else:
        uvicorn.run(app, uds='/run/atsd-web/http.sock', proxy_headers=False,
                    access_log=False, log_level='warning')


if __name__ == '__main__':
    main()
