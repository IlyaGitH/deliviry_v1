# /debug/ под общим HTTPS-шлюзом: камера, консоль событий, локальная сетка. Только администратор.
import asyncio
import time
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, Response
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.gzip import GZipMiddleware

debug_app = FastAPI(title='АТСД-1М · отладка', docs_url=None, redoc_url=None, openapi_url=None)
debug_app.add_middleware(GZipMiddleware, minimum_size=1000)
_ctx = {}


def attach(link, static_dir: Path, config_fn, map_fn, security_fn=lambda: None):
    """link — RosLink веба, config_fn — точки, граф дорог и подложки, map_fn — картинка подложки."""
    _ctx.update(link=link, static=static_dir, config=config_fn, map=map_fn, security=security_fn)
    debug_app.mount('/static', StaticFiles(directory=str(static_dir)), name='static')


@debug_app.get('/')
def index():
    static = _ctx['static']
    html = (static / 'debug.html').read_text(encoding='utf-8')
    for name in ('debug.js', 'grid100.js'):
        html = html.replace(f'/static/{name}"', f'/static/{name}?v={int((static/name).stat().st_mtime)}"')
    return HTMLResponse(html, headers={'Cache-Control': 'no-cache'})


@debug_app.get('/api/config')
def config():
    return _ctx['config']()


@debug_app.get('/api/map/{layer}.{ext}')
def map_image(layer: str, ext: str):
    return _ctx['map'](layer, ext)


@debug_app.get('/frame.jpg')
async def frame(after: int = -1):
    """Один свежий кадр, новее after (ждём его до 2 с). Браузер просит следующий, только получив этот:
    в сети всегда не больше одного кадра. Поток MJPEG на медленном Wi-Fi копил кадры в буфере —
    картинка отставала на десятки секунд."""
    link = _ctx['link']
    for _ in range(50):
        seq, jpg, _ = link.frame()
        if jpg is not None and seq != after:
            return Response(jpg, media_type='image/jpeg',
                            headers={'X-Seq': str(seq), 'Cache-Control': 'no-store'})
        await asyncio.sleep(0.04)
    return Response(status_code=204)


@debug_app.websocket('/ws')
async def ws(sock: WebSocket):
    await sock.accept()
    link, seq = _ctx['link'], 0
    try:
        while True:
            security = _ctx['security']()
            if security is not None:
                try:
                    if security.session(sock.headers).principal.role != 'admin':
                        raise ValueError('admin required')
                except Exception:
                    await sock.close(code=4403)
                    return
            t = link.snapshot()
            lines = link.console_since(seq)
            if lines:
                seq = lines[-1]['seq']
            _, jpg, age = link.frame()
            m = t.get('mission') or {}
            await asyncio.wait_for(sock.send_json({
                'ts': time.time(),
                'connected': t['connected'], 'brain_link': t['brain_link'],
                'pose': t['pose'], 'speed': t['speed'], 'road': t['road'], 'plan': t['plan'],
                'lane': t['lane'],
                'imu_status': t.get('imu_status', ''),
                'lidar_odom': t.get('lidar_odom', {}),
                'obstacle': t['obstacle'], 'state': m.get('state'), 'pause': m.get('pause', []),
                'frame_age': round(age, 1) if jpg is not None else None,
                'lines': lines,
            }), timeout=3.0)
            await asyncio.sleep(0.25)
    except (WebSocketDisconnect, RuntimeError):
        pass
    except asyncio.TimeoutError:
        await sock.close(code=1013)
