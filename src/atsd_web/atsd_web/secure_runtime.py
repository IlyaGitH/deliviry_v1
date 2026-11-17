"""Security boundary shared by HTTP, camera and WebSocket handlers."""
import asyncio
from collections import defaultdict, deque
from dataclasses import dataclass
import hashlib
import hmac
from http.cookies import SimpleCookie
import json
import logging
from pathlib import Path
import secrets
import socket
import queue
import threading
import time

from cryptography.fernet import Fernet
from starlette.responses import JSONResponse, RedirectResponse

from atsd_web.security import AccessDenied, CommandTickets, Principal, authorize, password_hash

COOKIE = '__Host-atsd'
MAX_BODY = 1024 * 1024


def audit_event(event, identity='', detail='', socket_path='/run/atsd-audit/socket'):
    payload = json.dumps({'event': event, 'identity': identity[:80],
                          'detail': detail[:400]}, ensure_ascii=False).encode() + b'\n'
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(2)
        sock.connect(socket_path)
        sock.sendall(payload)
        if sock.recv(128).strip() != b'OK':
            raise RuntimeError('Защищённый журнал недоступен')


@dataclass
class Session:
    principal: Principal
    certificate: str
    order: str | None
    expires: float


class Security:
    def __init__(self, config, tickets_path, audit=audit_event):
        self.config = config
        self.cipher = Fernet(config['data_key'].encode())
        self.tickets = CommandTickets(tickets_path)
        self.audit = audit
        self.sessions = {}
        self.attempts = defaultdict(deque)
        self.lock = threading.RLock()
        self.event_queue = queue.Queue(maxsize=1024)
        self.audit_error = ''
        threading.Thread(target=self._event_worker, daemon=True).start()

    def record_event(self, text):
        code = text.split(' ',1)[0]
        if code in ('SECURITY_REAR_INTRUSION','LOCK_TAMPER','LINK_LOST','LINK_OK',
                    'FAULT','ESTOP','ESTOP_RELEASE','MISSION_START','CARGO_LOADED',
                    'CARGO_UNLOADED','DELIVERED','NAV_FAILED','LOCK_PULSE',
                    'ORDER_SERVER_LOST','ORDER_SERVER_OK'):
            try:
                self.event_queue.put_nowait((code, text[:400]))
            except queue.Full:
                self.audit_error = 'Переполнена очередь журнала безопасности'
                logging.error(self.audit_error)

    def _event_worker(self):
        while True:
            code, text = self.event_queue.get()
            while True:
                try:
                    self.audit(code, 'robot', text)
                    self.audit_error = ''
                    break
                except (OSError, RuntimeError):
                    self.audit_error = 'Журнал безопасности недоступен; запись ожидает повторной отправки'
                    logging.error(self.audit_error)
                    time.sleep(2)
            self.event_queue.task_done()

    def certificate(self, headers):
        cert = headers.get('x-atsd-certificate', '').upper()
        if (headers.get('x-atsd-verified') != 'SUCCESS' or
                not hmac.compare_digest(headers.get('x-atsd-proxy', ''),
                                        self.config['proxy_secret']) or
                cert not in self.config['certificates']):
            raise AccessDenied('Требуется зарегистрированный клиентский сертификат')
        return cert

    def session(self, headers):
        cert = self.certificate(headers)
        cookies = SimpleCookie()
        try:
            cookies.load(headers.get('cookie', ''))
            token = cookies[COOKIE].value
        except (KeyError, ValueError):
            raise AccessDenied('Войдите в систему')
        with self.lock:
            item = self.sessions.get(hashlib.sha256(token.encode()).hexdigest())
            if item is None or item.certificate != cert or item.expires < time.monotonic():
                raise AccessDenied('Сеанс завершён; войдите заново')
            return item

    def login(self, certificate, role, secret, order=None):
        now = time.monotonic()
        with self.lock:
            attempts = self.attempts[certificate]
            while attempts and now-attempts[0] > 60:
                attempts.popleft()
            if len(attempts) >= 5:
                raise AccessDenied('Слишком много попыток; подождите минуту')
            attempts.append(now)
            if role == 'admin':
                valid = hmac.compare_digest(password_hash(secret, self.config['salt']),
                                            self.config['admin_hash'])
                oid = None
            else:
                valid = (role in ('sender', 'receiver') and order is not None and
                         order.get('status') not in ('delivered', 'cancelled') and
                         time.time()-order.get('access_issued_ts', order.get('started_ts', 0)) < 48*3600 and
                         hmac.compare_digest(order.get(role+'_token_hash', ''),
                                             hashlib.sha256(secret.encode()).hexdigest()))
                oid = order['id'] if valid else None
            if not valid:
                self.audit('AUTH_DENIED', certificate)
                raise AccessDenied('Неверные данные доступа')
            self.audit('AUTH_SUCCESS', certificate, role)
            attempts.clear()
            self.sessions = {k:v for k,v in self.sessions.items() if v.expires >= now}
            if len(self.sessions) >= 128:
                raise AccessDenied('Слишком много сеансов')
            token = secrets.token_urlsafe(32)
            self.sessions[hashlib.sha256(token.encode()).hexdigest()] = Session(
                Principal(certificate, role), certificate, oid, now+12*3600)
            return token

    def logout(self, headers):
        session = self.session(headers)
        cookies = SimpleCookie(headers.get('cookie', ''))
        with self.lock:
            self.sessions.pop(hashlib.sha256(cookies[COOKIE].value.encode()).hexdigest(), None)
        self.audit('AUTH_LOGOUT', session.certificate)

    def authorize(self, session, method, path):
        if path == '/ws' and method == 'GET':
            return
        if path.startswith('/api/map/') and method == 'GET':
            return
        if path in ('/api/security/ticket', '/api/security/logout') and method == 'POST':
            return
        authorize(session.principal, method, path, {session.order} if session.order else set())

    def payload(self, session, payload):
        if session.principal.role == 'admin':
            payload['security'] = {'audit_error': self.audit_error}
            return payload
        order = payload.get('order')
        if not order or order.get('id') != session.order:
            order = None
        tele = payload.get('telemetry', {})
        # Customers never receive other orders, raw camera/GPS logs or global event history.
        return {'ts': payload.get('ts'), 'order': order, 'points': payload.get('points', []),
                'events': [], 'telemetry': {k: tele.get(k) for k in
                 ('connected', 'brain_link', 'pose', 'speed', 'lock', 'battery', 'road', 'plan')}}


class SecurityMiddleware:
    def __init__(self, app, provider):
        self.app, self.provider = app, provider

    async def __call__(self, scope, receive, send):
        security = self.provider()
        if security is None:  # Only used by in-process tests or explicit localhost mock mode.
            return await self.app(scope, receive, send)
        if scope['type'] not in ('http', 'websocket'):
            return await self.app(scope, receive, send)
        from starlette.datastructures import Headers
        headers = Headers(scope=scope)
        path = scope['path']
        method = scope.get('method', 'GET')
        response_started = False
        command_started = False
        try:
            cert = security.certificate(headers)
            if scope['type'] == 'http' and (path == '/login' or path.startswith('/static/')):
                if method != 'GET':
                    raise AccessDenied('Метод запрещён')
                return await self.app(scope, receive, send)
            public_login = path == '/api/security/login' and method == 'POST'
            session = None if public_login else security.session(headers)
            if session:
                security.authorize(session, method, path)
            scope.setdefault('state', {})['security_session'] = session
            scope['state']['certificate'] = cert
            if method == 'POST':
                # Non-simple custom header + exact Origin check protect cookie sessions against CSRF.
                if headers.get('x-atsd-request') != '1':
                    raise AccessDenied('Нет подтверждения источника запроса')
                origin = headers.get('origin')
                if origin and origin != 'https://' + headers.get('host', ''):
                    raise AccessDenied('Чужой источник запроса')
                body = bytearray()
                while True:
                    message = await receive()
                    if message['type'] == 'http.disconnect':
                        return
                    body.extend(message.get('body', b''))
                    if len(body) > MAX_BODY:
                        raise AccessDenied('Слишком большой запрос')
                    if not message.get('more_body'):
                        break
                target = path + (('?' + scope['query_string'].decode()) if scope.get('query_string') else '')
                if path not in ('/api/security/login', '/api/security/ticket'):
                    security.tickets.consume(headers.get('x-atsd-ticket'), session.certificate,
                                             method, target, bytes(body))
                    await asyncio.to_thread(security.audit, 'COMMAND_ACCEPTED', cert, path)
                sent = False
                original_receive = receive
                async def replay_receive():
                    nonlocal sent
                    if not sent:
                        sent = True
                        return {'type': 'http.request', 'body': bytes(body), 'more_body': False}
                    return await original_receive()
                receive = replay_receive
            async def secured_send(message):
                nonlocal response_started
                if message['type'] == 'http.response.start':
                    if method == 'POST' and path != '/api/security/ticket':
                        await asyncio.to_thread(security.audit, 'COMMAND_RESULT', cert,
                                                path+' status='+str(message['status']))
                    response_started = True
                    message['headers'] = list(message.get('headers', [])) + [
                        (b'cache-control', b'no-store'), (b'referrer-policy', b'no-referrer'),
                        (b'x-content-type-options', b'nosniff'),
                        (b'x-frame-options', b'DENY')]
                await send(message)
            command_started = method == 'POST'
            return await self.app(scope, receive, secured_send)
        except AccessDenied as exc:
            try:
                await asyncio.to_thread(security.audit, 'ACCESS_DENIED',
                                        headers.get('x-atsd-certificate', '')[:80], path)
            except (OSError, RuntimeError):
                pass
            if scope['type'] == 'websocket':
                return await send({'type':'websocket.close', 'code':4403})
            if method == 'GET' and path in ('/', '/debug/', '/debug'):
                return await RedirectResponse('/login', status_code=303)(scope, receive, send)
            return await JSONResponse({'detail':str(exc)}, status_code=403)(scope, receive, send)
        except (OSError, RuntimeError) as exc:
            if response_started:
                raise
            if scope['type'] == 'websocket':
                return await send({'type':'websocket.close', 'code':1011})
            detail = ('Не удалось подтвердить результат; проверьте состояние робота перед повтором'
                      if command_started else 'Служба защиты недоступна; команда не выполнена')
            return await JSONResponse({'detail':detail},
                                      status_code=503)(scope, receive, send)
