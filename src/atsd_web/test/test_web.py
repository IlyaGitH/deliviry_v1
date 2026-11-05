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
