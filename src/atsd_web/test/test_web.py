import asyncio
import threading

import pytest
from fastapi import WebSocketDisconnect
from fastapi.testclient import TestClient

from atsd_web import server
from atsd_web.ros_link import RosLink


@pytest.fixture
def web(monkeypatch, tmp_path):
    monkeypatch.setattr(server, 'DATA_DIR', tmp_path)
    for name, filename in [('POINTS_FILE', 'points.json'), ('ORDERS_FILE', 'orders.json'),
                           ('ROADS_FILE', 'roads.json')]:
        monkeypatch.setattr(server, name, tmp_path / filename)
    server._save(server.ROADS_FILE, {'nodes': {p['id']:[p['x'],p['y']] for p in server.DEFAULT_POINTS},
        'edges':[['kpp','admin'],['admin','testing'],['testing','depot']]})
    link = RosLink(mock=True)
    monkeypatch.setattr(server, 'link', link)
    return TestClient(server.app), link


def test_web_order_and_incremental_events(web):
    client, link = web
    assert client.get('/').status_code == 200
    assert client.get('/static/app.js').status_code == 200
    assert len(client.get('/api/config').json()['points']) == 4
    order = client.post('/api/orders', json={'from_id': 'kpp', 'to_id': 'depot'}).json()
    link._event('MISSION_START')
    with client.websocket_connect('/ws') as ws:
        first = ws.receive_json()
        assert first['events'][0]['text'] == 'MISSION_START'
        assert not any(k.endswith('_token') for k in first['order'])
        second = ws.receive_json()
        assert second['events'] == []
        link._event('ARRIVED_PICKUP')
        link._set(['mission'], {'state': 'WAIT_LOAD', 'from': 'kpp', 'to': 'depot'})
        while True:
            next_frame = ws.receive_json()
            if next_frame['events']:
                break
        assert next_frame['order']['status'] == 'at_pickup'
        assert next_frame['events'][0]['seq'] > first['events'][0]['seq']
    # A new viewer still receives recent history.
    with client.websocket_connect('/ws') as ws:
        assert len(ws.receive_json()['events']) == 2
    assert client.post(f"/api/orders/{order['id']}/cancel").status_code == 200


def test_slow_storage_does_not_block_event_loop(monkeypatch):
    started, release = threading.Event(), threading.Event()

    def slow_payload():
        started.set()
        assert release.wait(2)
        return {'events': []}

    monkeypatch.setattr(server, 'status_payload', slow_payload)

    class Socket:
        async def accept(self):
            pass

        async def send_json(self, payload):
            raise WebSocketDisconnect()

    async def check():
        task = asyncio.create_task(server.ws(Socket()))
        try:
            # If disk work runs in the event loop, release cannot be signalled in time.
            while not started.is_set():
                await asyncio.sleep(0.001)
            release.set()
            await task
        finally:
            release.set()

    asyncio.run(check())


def test_debug_frame_and_compression(web):
    from atsd_web import debug
    client, link = web
    debug.attach(link, server.STATIC, server.api_config, server.api_map_image)
    client = TestClient(debug.debug_app)
    link._on_jpeg(b'\xff\xd8test-jpeg')
    response = client.get('/frame.jpg')
    assert response.content == b'\xff\xd8test-jpeg'
    assert response.headers['x-seq'] == '1'
    assert response.headers['cache-control'] == 'no-store'
    assert client.get('/static/debug.js', headers={'Accept-Encoding': 'gzip'}).headers['content-encoding'] == 'gzip'


def test_manual_pose_accepted_without_brain_or_ros(web):
    client, link = web
    link.mock = False
    assert link._node is None
    response = client.post('/api/initialpose', json={'x': 10000, 'y': -8000, 'yaw': 2.0})
    assert response.status_code == 200
    assert link.snapshot()['pose'] == {'x': 10000, 'y': -8000, 'yaw': 2.0}
    assert link._pending_pose is not None
    link._flush_pose()  # keep queued before publishers/subscribers exist
    assert link._pending_pose is not None
    client.post('/api/initialpose', json={'x': -5, 'y': 7, 'yaw': 0.0})
    assert link._pending_pose[1:] == (-5, 7, 0)


def test_latest_manual_pose_is_delivered_once_when_nav_appears(web):
    from types import SimpleNamespace as NS
    from unittest.mock import Mock
    _, link = web
    link.mock = False
    link._pub_pose = Mock()
    link._pub_pose.get_subscription_count.return_value = 0
    link._node = Mock()
    link._pose_type = lambda: NS(header=NS(), pose=NS(
        pose=NS(position=NS(), orientation=NS()), covariance=[0.] * 36))
    link.set_pose(10, 20, 0)
    link.set_pose(30, 40, 1)
    link._flush_pose()
    link._pub_pose.publish.assert_not_called()
    link._pub_pose.get_subscription_count.return_value = 1
    link._flush_pose()
    msg = link._pub_pose.publish.call_args.args[0]
    assert msg.pose.pose.position.x == 30 and msg.pose.pose.position.y == 40
    assert link._pending_pose is None
    link._flush_pose()
    assert link._pub_pose.publish.call_count == 1


def test_offline_yandex_does_not_start_download_and_serves_cached_map(monkeypatch, tmp_path):
    monkeypatch.delenv('ATSD_MAP_DOWNLOADS', raising=False)
    monkeypatch.setattr(server, 'DATA_DIR', tmp_path)
    monkeypatch.setattr(server, 'map_origin', lambda: (55.,37.))
    def forbidden(*args, **kwargs):
        raise AssertionError('Offline mode attempted background download')
    monkeypatch.setattr(server.threading, 'Thread', forbidden)
    layer = next(iter(server.MAP_LAYERS))
    assert server.yandex_map(layer) is None
    _, ext, _ = server.MAP_LAYERS[layer]
    (tmp_path / f'yandex_{layer}.{ext}').write_bytes(b'cached image')
    server._save(tmp_path / f'yandex_{layer}.json', [55.,37.,server.MAP_ZOOM])
    assert server.yandex_map(layer)['image'].startswith('/api/map/')


def test_lidar_odometry_diagnostic_does_not_remain_valid_after_publisher_loss(monkeypatch):
    import types
    from atsd_web import ros_link
    link=RosLink(mock=True)
    clock=[10.]
    monkeypatch.setattr(ros_link.time,'monotonic',lambda:clock[0])
    link._on_lidar_odom(types.SimpleNamespace(data='{"valid":true,"reason":"ok","slip_suspected":true}'))
    assert link.snapshot()['lidar_odom']['valid']
    clock[0]=12.
    result=link.snapshot()['lidar_odom']
    assert not result['valid'] and not result['slip_suspected'] and '_received' not in result


def test_unlock_waits_for_cpu_preparation(monkeypatch):
    from types import SimpleNamespace as NS
    from atsd_web import ros_link
    clock = [0.0]
    monkeypatch.setattr(ros_link.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(ros_link.time, 'sleep', lambda dt: clock.__setitem__(0, clock[0] + dt))
    link = RosLink(mock=True)
    link.mock = False
    link._node = object()
    link._trigger_type = NS(Request=lambda: None)
    future = NS(done=lambda: clock[0] >= 5.0,
                result=lambda: NS(success=True, message='Импульс открытия подан'))
    link._unlock_cli = NS(wait_for_service=lambda **kw: True, call_async=lambda _: future)
    assert link.unlock() == (True, 'Импульс открытия подан')
