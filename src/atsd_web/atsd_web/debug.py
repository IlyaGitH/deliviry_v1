# Отладочная страница (порт 8090): кадр камеры с отрисовкой распознанного, консоль GPS, карта.
# Только для настройки в локальной сети: камера видна всем, кто откроет страницу.
import asyncio
import time
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

debug_app = FastAPI(title='АТСД-1М · отладка', docs_url=None, redoc_url=None)
_ctx = {}


def attach(link, static_dir: Path, config_fn, map_fn):
    """link — RosLink веба, config_fn — точки, граф дорог и подложки, map_fn — картинка подложки."""
    _ctx.update(link=link, static=static_dir, config=config_fn, map=map_fn)
    debug_app.mount('/static', StaticFiles(directory=str(static_dir)), name='static')


@debug_app.get('/')
def index():
    static = _ctx['static']
    html = (static / 'debug.html').read_text(encoding='utf-8')
    js = static / 'debug.js'
    html = html.replace('/static/debug.js"', f'/static/debug.js?v={int(js.stat().st_mtime)}"')
    return HTMLResponse(html, headers={'Cache-Control': 'no-cache'})


@debug_app.get('/api/config')
def config():
    return _ctx['config']()


@debug_app.get('/api/map/{layer}.{ext}')
def map_image(layer: str, ext: str):
    return _ctx['map'](layer, ext)


@debug_app.get('/stream')
async def stream():
    """MJPEG: браузер показывает его обычным <img>, кадры приходят по мере появления."""
    link = _ctx['link']

    async def frames():
        last = -1
        while True:
            seq, jpg, _ = link.frame()
            if jpg is not None and seq != last:
                last = seq
                yield (b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                       str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
            await asyncio.sleep(0.04)

    return StreamingResponse(frames(), media_type='multipart/x-mixed-replace; boundary=frame',
                             headers={'Cache-Control': 'no-cache'})


@debug_app.websocket('/ws')
async def ws(sock: WebSocket):
    await sock.accept()
    link, seq = _ctx['link'], 0
    try:
        while True:
            t = link.snapshot()
            lines = link.console_since(seq)
            if lines:
                seq = lines[-1]['seq']
            _, jpg, age = link.frame()
            m = t.get('mission') or {}
            await sock.send_json({
                'ts': time.time(),
                'connected': t['connected'], 'brain_link': t['brain_link'],
                'pose': t['pose'], 'speed': t['speed'], 'road': t['road'], 'plan': t['plan'],
                'gnss': t['gnss'], 'gnss_status': t['gnss_status'], 'lane': t['lane'],
                'obstacle': t['obstacle'], 'state': m.get('state'), 'pause': m.get('pause', []),
                'frame_age': round(age, 1) if jpg is not None else None,
                'lines': lines,
            })
            await asyncio.sleep(0.25)
    except (WebSocketDisconnect, RuntimeError):
        pass
