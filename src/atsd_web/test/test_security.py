from concurrent.futures import ThreadPoolExecutor
import hashlib

import pytest

from atsd_web.security import AccessDenied, CommandTickets, Principal, authorize


def test_roles_and_orders_are_not_interchangeable():
    sender = Principal('sender-device', 'sender')
    authorize(sender, 'POST', '/api/orders/1234abcd/unlock', {'1234abcd'})
    for verb, path in [('POST', '/api/initialpose'), ('POST', '/api/orders'),
                       ('POST', '/api/mission/resume'), ('GET', '/api/orders'),
                       ('POST', '/api/orders/abcd1234/unlock'), ('GET', '/frame.jpg')]:
        with pytest.raises(AccessDenied):
            authorize(sender, verb, path, {'1234abcd'})
    with pytest.raises(AccessDenied):
        authorize(None, 'GET', '/')
    with pytest.raises(AccessDenied):
        Principal('device', 'root')


@pytest.fixture
def tickets(tmp_path):
    now = [100.]
    store = CommandTickets(tmp_path/'tickets.db', clock=lambda: now[0])
    return store, now


def issue(store, body=b'{}'):
    return store.issue('admin-cert', 'POST', '/api/initialpose', hashlib.sha256(body).hexdigest())


def test_command_can_only_run_once_even_concurrently(tickets):
    store, _ = tickets
    ticket = issue(store)
    def consume(_):
        try:
            store.consume(ticket, 'admin-cert', 'POST', '/api/initialpose', b'{}')
            return True
        except AccessDenied:
            return False
    with ThreadPoolExecutor(max_workers=8) as workers:
        assert sum(workers.map(consume, range(8))) == 1


@pytest.mark.parametrize('owner,verb,target,body', [
    ('another-cert', 'POST', '/api/initialpose', b'{}'),
    ('admin-cert', 'GET', '/api/initialpose', b'{}'),
    ('admin-cert', 'POST', '/api/roads', b'{}'),
    ('admin-cert', 'POST', '/api/initialpose', b'{"x":100}'),
])
def test_tickets_bind_entire_command(tickets, owner, verb, target, body):
    store, _ = tickets
    ticket = issue(store)
    with pytest.raises(AccessDenied):
        store.consume(ticket, owner, verb, target, body)


def test_expiry_and_restart(tickets):
    store, now = tickets
    ticket = issue(store)
    now[0] += 31
    with pytest.raises(AccessDenied):
        store.consume(ticket, 'admin-cert', 'POST', '/api/initialpose', b'{}')
    ticket = issue(store)
    fresh = CommandTickets(store.path)
    with pytest.raises(AccessDenied):
        fresh.consume(ticket, 'admin-cert', 'POST', '/api/initialpose', b'{}')
