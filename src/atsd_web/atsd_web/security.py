"""Transport-independent access policy and single-use, payload-bound commands.

No passwords, certificates or private keys are embedded in the source tree.
Transport adapters must supply an authenticated principal, never a query role.
"""
from dataclasses import dataclass
import hashlib
import re
import secrets
import sqlite3
import time


class AccessDenied(Exception):
    pass


@dataclass(frozen=True)
class Principal:
    identity: str
    role: str

    def __post_init__(self):
        if self.role not in ('admin', 'sender', 'receiver'):
            raise AccessDenied('Неизвестная роль')


ORDER_ACTION = re.compile(r'^/api/orders/([a-f0-9]{8,64})/(unlock|done)$')
ORDER_READ = re.compile(r'^/api/orders/([a-f0-9]{8,64})$')


def authorize(principal, method, path, order_ids=()):
    """Default deny. Order grants are server-side assignments, not caller input."""
    if principal is None:
        raise AccessDenied('Требуется аутентификация')
    if principal.role == 'admin':
        return
    if method == 'GET' and path in ('/', '/api/config', '/api/security/me'):
        return
    if method == 'GET' and path.startswith('/static/'):
        return
    match = ORDER_READ.fullmatch(path) if method == 'GET' else (
        ORDER_ACTION.fullmatch(path) if method == 'POST' else None)
    if match and match.group(1) in order_ids:
        return
    raise AccessDenied('Недостаточно прав')


class CommandTickets:
    """One-shot capabilities bound to identity, verb, URL and exact body bytes.

    A fresh boot identifier invalidates all old tickets on process restart.
    SQLite transactions prevent two simultaneous requests consuming one ticket.
    """
    def __init__(self, path, ttl=30., clock=time.monotonic, limit=4096):
        self.path, self.ttl, self.clock, self.limit = str(path), ttl, clock, limit
        self.boot = secrets.token_hex(32)
        with self.connect() as db:
            db.execute('CREATE TABLE IF NOT EXISTS tickets '
                       '(token TEXT PRIMARY KEY, owner TEXT, verb TEXT, target TEXT, '
                       'body TEXT, issued REAL, boot TEXT)')
            db.execute('DELETE FROM tickets')

    def connect(self):
        return sqlite3.connect(self.path, timeout=3., isolation_level='IMMEDIATE')

    def issue(self, owner, method, target, body_hash):
        if method != 'POST' or not target.startswith('/api/') or len(target) > 512:
            raise AccessDenied('Некорректная команда')
        if not re.fullmatch('[a-f0-9]{64}', body_hash):
            raise AccessDenied('Некорректный хеш команды')
        token = secrets.token_urlsafe(32)
        now = self.clock()
        with self.connect() as db:
            db.execute('DELETE FROM tickets WHERE issued < ? OR boot != ?', (now-self.ttl, self.boot))
            if db.execute('SELECT count(*) FROM tickets').fetchone()[0] >= self.limit:
                raise AccessDenied('Слишком много ожидающих команд')
            db.execute('INSERT INTO tickets VALUES (?,?,?,?,?,?,?)',
                       (hashlib.sha256(token.encode()).hexdigest(), owner, method,
                        target, body_hash, now, self.boot))
        return token

    def consume(self, token, owner, method, target, body):
        if not isinstance(token, str) or not 20 <= len(token) <= 128:
            raise AccessDenied('Нет одноразового подтверждения команды')
        hashed = hashlib.sha256(token.encode()).hexdigest()
        with self.connect() as db:
            # DELETE ... RETURNING atomically consumes even an invalid attempt.
            row = db.execute('DELETE FROM tickets WHERE token=? RETURNING '
                             'owner,verb,target,body,issued,boot', (hashed,)).fetchone()
        now = self.clock()
        expected = (owner, method, target, hashlib.sha256(body).hexdigest())
        if row is None or row[:4] != expected or row[5] != self.boot or not 0 <= now-row[4] <= self.ttl:
            raise AccessDenied('Команда изменена, устарела или уже использована')


def password_hash(password, salt):
    return hashlib.scrypt(password.encode(), salt=bytes.fromhex(salt),
                          n=16384, r=8, p=1).hex()
