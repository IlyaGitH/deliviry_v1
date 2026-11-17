import hashlib
import json
from types import SimpleNamespace
import pytest
from cryptography.fernet import Fernet
from fastapi.testclient import TestClient
from atsd_web import server
from atsd_web.ros_link import RosLink
from atsd_web.secure_runtime import Security
from atsd_web.security import password_hash

CERT = 'ABCDEF'
HEADERS = {'x-atsd-certificate':CERT,'x-atsd-verified':'SUCCESS',
           'x-atsd-proxy':'test-proxy','x-atsd-request':'1','origin':'https://testserver'}


@pytest.fixture
def secure(monkeypatch, tmp_path):
    events = []
    config = {'data_key':Fernet.generate_key().decode(), 'salt':'ab'*16,
              'admin_hash':password_hash('test-password','ab'*16),
              'proxy_secret':'test-proxy','certificates':{CERT:'android', 'OTHER':'ios'}}
    security = Security(config, tmp_path/'tickets.db', audit=lambda *args: events.append(args))
    monkeypatch.setattr(server,'security',security)
    monkeypatch.setattr(server,'DATA_DIR',tmp_path)
    for field, filename in [('POINTS_FILE','points.json'),('ROADS_FILE','roads.json'),('ORDERS_FILE','orders.json')]:
        monkeypatch.setattr(server,field,tmp_path/filename)
    link = RosLink(mock=True)
    monkeypatch.setattr(server,'link',link)
    server._save(server.ROADS_FILE, {'nodes':{'kpp':[2,0],'admin':[18,0]},'edges':[['kpp','admin']]})
    server.save_orders([{'id':'1234abcd','from_id':'kpp','to_id':'admin','status':'at_pickup',
                        'started_ts':__import__('time').time(),
                        'sender_token_hash':hashlib.sha256(b'sender-secret').hexdigest(),
                        'receiver_token_hash':hashlib.sha256(b'receiver-secret').hexdigest()}])
    client = TestClient(server.app,base_url='https://testserver',headers=HEADERS)
    return client, security, link, events


def login(client, role='admin', secret='test-password'):
    return client.post('/api/security/login',json={'role':role,'secret':secret,'order':'1234abcd'})


def command(client,path,body=b''):
    ticket = client.post('/api/security/ticket',json={'target':path,'body_hash':hashlib.sha256(body).hexdigest()})
    assert ticket.status_code == 200, ticket.text
    return client.post(path,content=body,headers={'x-atsd-ticket':ticket.json()['ticket'],'content-type':'application/json'})


def test_missing_identity_and_anonymous_api_are_denied(secure):
    client, _, _, _ = secure
    assert client.get('/api/orders').status_code == 403
    assert client.get('/login',headers={'x-atsd-proxy':'forged'}).status_code == 403
    assert client.post('/api/initialpose',json={'x':1,'y':2,'yaw':0}).status_code == 403


def test_admin_ticket_body_and_csrf(secure):
    client, _, _, _ = secure
    assert login(client).status_code == 200
    body = b'{"x":1,"y":2,"yaw":0}'
    assert command(client,'/api/initialpose',body).status_code == 200
    ticket = client.post('/api/security/ticket',json={'target':'/api/initialpose',
                'body_hash':hashlib.sha256(body).hexdigest()}).json()['ticket']
    assert client.post('/api/initialpose',content=b'{"x":99,"y":2,"yaw":0}',
                       headers={'x-atsd-ticket':ticket}).status_code == 403
    assert client.post('/api/security/ticket',json={},headers={'origin':'https://evil.test'}).status_code == 403


def test_sender_cannot_change_role_or_other_orders(secure):
    client, _, _, _ = secure
    assert login(client,'sender','sender-secret').status_code == 200
    assert client.get('/api/orders').status_code == 403
    assert client.get('/api/orders/87654321').status_code == 403
    assert client.get('/debug/frame.jpg').status_code == 403
    assert client.post('/api/security/ticket',json={'target':'/api/initialpose',
                'body_hash':hashlib.sha256(b'').hexdigest()}).status_code == 403
    assert command(client,'/api/orders/1234abcd/unlock?role=receiver').status_code == 403
    assert command(client,'/api/orders/1234abcd/unlock?role=sender').status_code == 200


def test_replay_and_cookie_certificate_binding(secure):
    client, _, _, _ = secure
    assert login(client).status_code == 200
    path='/api/orders/1234abcd/cancel'
    token=client.post('/api/security/ticket',json={'target':path,
                     'body_hash':hashlib.sha256(b'').hexdigest()}).json()['ticket']
    assert client.post(path,headers={'x-atsd-ticket':token}).status_code == 200
    assert client.post(path,headers={'x-atsd-ticket':token}).status_code == 403
    assert client.get('/api/orders',headers={'x-atsd-certificate':'OTHER'}).status_code == 403


def test_orders_encrypted_and_hashes_not_returned(secure):
    client, sec, _, _ = secure
    assert b'1234abcd' not in server.ORDERS_FILE.read_bytes()
    assert login(client,'sender','sender-secret').status_code == 200
    order=client.get('/api/orders/1234abcd').json()
    assert 'sender_token_hash' not in order and 'sender_certificate' not in order
    assert json.loads(sec.cipher.decrypt(server.ORDERS_FILE.read_bytes()))[0]['id']=='1234abcd'


def test_audit_failure_prevents_command(secure):
    client, sec, link, _ = secure
    assert login(client).status_code == 200
    original = link.set_pose
    calls=[]
    link.set_pose=lambda *a: calls.append(a)
    def unavailable(*args):
        raise OSError('audit down')
    sec.audit=unavailable
    response=command(client,'/api/initialpose',b'{"x":1,"y":2,"yaw":0}')
    assert response.status_code==503 and not calls


def test_websocket_authentication_and_filtering(secure):
    from starlette.websockets import WebSocketDisconnect
    client, sec, _, _ = secure
    with pytest.raises(WebSocketDisconnect):
        with client.websocket_connect('/ws'):
            pass
    assert login(client,'sender','sender-secret').status_code == 200
    # Directly validate that no other delivery or journal can leak through the stream.
    session=next(iter(sec.sessions.values()))
    filtered=sec.payload(session,{'order':{'id':'87654321'},'events':[{'secret':1}],
                                 'telemetry':{'gnss':{'lat':1},'speed':0}})
    assert filtered['order'] is None and filtered['events']==[]
    assert 'gnss' not in filtered['telemetry']


def test_completion_cannot_be_repeated_with_a_fresh_ticket(secure, monkeypatch):
    client, _, link, _ = secure
    assert login(client, 'sender', 'sender-secret').status_code == 200
    calls = []
    monkeypatch.setattr(link, 'cargo', lambda: calls.append(True))
    path = '/api/orders/1234abcd/done?role=sender'
    assert command(client, path).status_code == 200
    assert command(client, path).status_code == 409
    assert calls == [True]


def test_rotating_access_revokes_old_sessions_and_links(secure):
    client, sec, _, _ = secure
    assert login(client, 'sender', 'sender-secret').status_code == 200
    old = client.cookies.get('__Host-atsd')
    assert login(client).status_code == 200
    response = command(client, '/api/orders/1234abcd/access')
    assert response.status_code == 200
    assert client.get('/api/security/me', headers={'cookie': '__Host-atsd='+old}).status_code == 403
    assert login(client, 'sender', 'sender-secret').status_code == 403
    assert login(client, 'sender', response.json()['sender_token']).status_code == 200
    assert 'sender_certificate' in server.orders()[0]


def test_role_validation_and_device_bound_grants(secure):
    client, _, _, _ = secure
    assert login(client, ['admin']).status_code == 403
    assert login(client, 'sender', 'sender-secret').status_code == 200
    assert client.post('/api/security/login', json={'role':'sender','secret':'sender-secret',
        'order':'1234abcd'}, headers={'x-atsd-certificate':'OTHER'}).status_code == 403
