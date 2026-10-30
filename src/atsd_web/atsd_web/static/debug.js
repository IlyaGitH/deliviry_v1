// Отладочная страница: MJPEG с камеры, карта (сырой GPS против оценки), консоль NMEA и событий.
const $ = id => document.getElementById(id);
let CFG = { points: [], roads: { nodes: {}, edges: [] } };
let T = null;                 // последняя телеметрия
const fixes = [], track = []; // история за последние ~2 мин
let paused = false;
let view = null;              // {cx, cy, scale} — подбирается сам, колесо/щипок меняют масштаб

let mapImg = null;  // схема Яндекса вокруг origin (как на главной), светлая
fetch('/api/config').then(r => r.json()).then(c => {
  CFG = c; view = null;
  const m = (c.maps || {}).map;
  if (m) { const img = new Image(); img.onload = () => { mapImg = img; }; img.src = m.image; }
});

// ---------- камера ----------
const cam = $('cam');
const sleep = ms => new Promise(r => setTimeout(r, ms));
// по одному кадру: следующий просим, только показав этот — задержка не копится на медленном Wi-Fi
async function camLoop() {
  let seq = -1, url = null;
  for (;;) {
    try {
      const r = await fetch(`/frame.jpg?after=${seq}`, { cache: 'no-store' });
      if (r.status !== 200) { await sleep(500); continue; }
      seq = +r.headers.get('X-Seq');
      const next = URL.createObjectURL(await r.blob());
      cam.src = next;
      await cam.decode().catch(() => {});
      if (url) URL.revokeObjectURL(url);
      url = next;
    } catch (e) { await sleep(1000); }
  }
}
camLoop();

// ---------- консоль ----------
const con = $('con');
const MAX_LINES = 800;
function hhmmss(ts) { return new Date(ts * 1000).toTimeString().slice(0, 8); }
function log(ts, kind, text) {
  if (paused) return;
  if (kind === 'nmea' && !$('nmea').checked) return;
  const d = document.createElement('div');
  d.className = /SUSPECT|FAILED|LOST|ESTOP/.test(text) ? 'alarm' : kind;
  d.textContent = `${hhmmss(ts)} ${text}`;
  con.appendChild(d);
  while (con.childElementCount > MAX_LINES) con.firstChild.remove();
  if ($('auto').checked) con.scrollTop = con.scrollHeight;
}
$('pause').onclick = e => { paused = !paused; e.target.textContent = paused ? 'Продолжить' : 'Пауза'; };
$('clear').onclick = () => { con.textContent = ''; };

// раз в секунду — сводка: что думает навигация о GPS
let lastSummary = 0;
function summary(t) {
  const r = t.road || {};
  const f = r.fix ? `GPS ${r.fix[0].toFixed(1)}, ${r.fix[1].toFixed(1)} σ${r.sigma ?? '?'}м` : 'GPS —';
  const p = r.pose ? `оценка ${r.pose[0].toFixed(1)}, ${r.pose[1].toFixed(1)} курс ${(r.pose[2] * 180 / Math.PI).toFixed(0)}°` : 'оценки нет';
  const w = r.wifi ? ` | Wi-Fi ${r.wifi[0].toFixed(0)}, ${r.wifi[1].toFixed(0)} ±${r.wifi[2].toFixed(0)}м` : '';
  const d = r.fix && r.pose ? ` расхождение ${Math.hypot(r.fix[0] - r.pose[0], r.fix[1] - r.pose[1]).toFixed(1)}м` : '';
  return `[${r.gnss || '?'}] ${f} | ${p}${d}${w} | ${t.gnss_status || ''}`;
}

// ---------- телеметрия ----------
function connect() {
  const ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws');
  ws.onmessage = e => {
    const t = JSON.parse(e.data);
    T = t;
    t.lines.forEach(l => log(l.ts, l.kind, l.text));
    if (t.ts - lastSummary >= 1) { lastSummary = t.ts; log(t.ts, 'state', summary(t)); }
    const r = t.road || {};
    const now = t.ts;
    if (r.fix) remember(fixes, [r.fix[0], r.fix[1], r.sigma || 2.5, now]);
    if (r.pose) remember(track, [r.pose[0], r.pose[1], 0, now]);
    chips(t);
    $('camoff').classList.toggle('hidden', t.frame_age != null && t.frame_age < 2);
    $('link').textContent = 'на связи';
    $('link').className = 'chip ok';
  };
  ws.onclose = () => {
    $('link').textContent = 'нет связи';
    $('link').className = 'chip bad';
    setTimeout(connect, 1500);
  };
}
function remember(arr, p) {
  const last = arr[arr.length - 1];
  if (last && Math.hypot(p[0] - last[0], p[1] - last[1]) < 0.05 && p[3] - last[3] < 1) return;
  arr.push(p);
  while (arr.length && p[3] - arr[0][3] > 120) arr.shift();
}
connect();

function chip(text, cls) { return `<span class="chip ${cls || ''}">${text}</span>`; }
function chips(t) {
  const r = t.road || {};
  const g = { ok: ['GPS ок', 'ok'], suspect: ['GPS не верю (глушение?)', 'bad'], lost: ['GPS нет', 'warn'] }[r.gnss]
    || ['GPS ' + (r.gnss || '?'), 'warn'];
  const out = [chip(g[0], g[1])];
  if (t.gnss_status) out.push(chip(t.gnss_status));
  const lane = (t.lane || '').split(' ');
  if (lane.length >= 4 && lane[0] !== 'none') {
    out.push(chip(`разметка: линий ${lane[3]}, смещение ${(+lane[0] * 100).toFixed(0)} см, угол ${(+lane[1] * 180 / Math.PI).toFixed(1)}°, полоса ${(+lane[2]).toFixed(2)} м`, 'ok'));
  } else out.push(chip('разметки не вижу', 'warn'));
  if (t.imu_status) out.push(chip(t.imu_status.replace(/^imu /, 'гироскоп '), /ошибка|калибруется/.test(t.imu_status) ? 'warn' : 'ok'));
  else out.push(chip('гироскопа нет', 'warn'));
  out.push(chip('режим ' + (r.mode || '—')));
  out.push(chip(`${(t.speed || 0).toFixed(2)} м/с`));
  if (t.state) out.push(chip('рейс ' + t.state + (t.pause && t.pause.length ? ' (пауза: ' + t.pause.join(', ') + ')' : '')));
  if (t.obstacle) out.push(chip(t.obstacle, /свобод/.test(t.obstacle) ? 'ok' : 'bad'));
  if (!t.brain_link) out.push(chip('нет связи с Brain', 'bad'));
  $('chips').innerHTML = out.join('');
}

// ---------- карта ----------
const cv = $('dmap'), ctx = cv.getContext('2d');
function fit(w, h) {
  const xs = [], ys = [];
  const add = (x, y) => { xs.push(x); ys.push(y); };
  Object.values(CFG.roads.nodes || {}).forEach(([x, y]) => add(x, y));
  fixes.slice(-30).forEach(p => add(p[0], p[1]));
  track.slice(-30).forEach(p => add(p[0], p[1]));
  const wf = T && T.road && T.road.wifi;
  if (wf) { add(wf[0] - wf[2], wf[1] - wf[2]); add(wf[0] + wf[2], wf[1] + wf[2]); }
  if (!xs.length) add(0, 0);
  const pad = 4;
  const x0 = Math.min(...xs) - pad, x1 = Math.max(...xs) + pad;
  const y0 = Math.min(...ys) - pad, y1 = Math.max(...ys) + pad;
  return { cx: (x0 + x1) / 2, cy: (y0 + y1) / 2, scale: Math.min(w / (x1 - x0), h / (y1 - y0)) };
}
let userZoom = 1;
cv.addEventListener('wheel', e => { e.preventDefault(); userZoom *= e.deltaY < 0 ? 1.25 : 0.8; }, { passive: false });
cv.addEventListener('dblclick', () => { userZoom = 1; });

function draw() {
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== w * dpr) { cv.width = w * dpr; cv.height = h * dpr; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  view = fit(w, h);
  const s = view.scale * userZoom;
  // следим за роботом, когда приближено
  const r = T && T.road && T.road.pose;
  const cx = userZoom > 1 && r ? r[0] : view.cx, cy = userZoom > 1 && r ? r[1] : view.cy;
  const X = x => w / 2 + (x - cx) * s, Y = y => h / 2 - (y - cy) * s;

  const meta = (CFG.maps || {}).map;
  if (mapImg && meta) {
    const mpp = meta.meters_per_pixel, [ox, oy] = meta.origin;
    ctx.drawImage(mapImg, X(ox), Y(oy + mapImg.naturalHeight * mpp), mapImg.naturalWidth * mpp * s, mapImg.naturalHeight * mpp * s);
  }
  // сетка 5 м
  ctx.strokeStyle = mapImg ? 'rgba(0,0,0,.08)' : '#1f2b33'; ctx.lineWidth = 1;
  const step = s * 5 < 20 ? 25 : 5;
  for (let gx = Math.floor((cx - w / 2 / s) / step) * step; X(gx) < w; gx += step) { ctx.beginPath(); ctx.moveTo(X(gx), 0); ctx.lineTo(X(gx), h); ctx.stroke(); }
  for (let gy = Math.floor((cy - h / 2 / s) / step) * step; Y(gy) > 0; gy += step) { ctx.beginPath(); ctx.moveTo(0, Y(gy)); ctx.lineTo(w, Y(gy)); ctx.stroke(); }

  // дороги
  const nodes = CFG.roads.nodes || {};
  ctx.strokeStyle = '#56656f'; ctx.lineWidth = Math.max(2, 3 * s / 10);
  (CFG.roads.edges || []).forEach(([a, b]) => {
    if (!nodes[a] || !nodes[b]) return;
    ctx.beginPath(); ctx.moveTo(X(nodes[a][0]), Y(nodes[a][1])); ctx.lineTo(X(nodes[b][0]), Y(nodes[b][1])); ctx.stroke();
  });
  ctx.fillStyle = mapImg ? '#0d1418' : '#8ba0ad'; ctx.font = '11px sans-serif';
  Object.entries(nodes).forEach(([id, [x, y]]) => {
    ctx.beginPath(); ctx.arc(X(x), Y(y), 3, 0, 7); ctx.fill();
    ctx.fillText(id, X(x) + 5, Y(y) - 5);
  });

  if (T) {
    // маршрут
    if (T.plan && T.plan.length > 1) {
      ctx.strokeStyle = '#4b9dff'; ctx.lineWidth = 2;
      ctx.beginPath(); T.plan.forEach(([x, y], i) => i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y))); ctx.stroke();
    }
    // сырой GPS: точки истории + круг точности последнего
    fixes.forEach(([x, y], i) => {
      ctx.fillStyle = `rgba(224,163,58,${0.15 + 0.85 * i / fixes.length})`;
      ctx.beginPath(); ctx.arc(X(x), Y(y), 2.5, 0, 7); ctx.fill();
    });
    const f = T.road && T.road.fix;
    if (f) {
      ctx.strokeStyle = 'rgba(224,163,58,.8)'; ctx.fillStyle = 'rgba(224,163,58,.12)'; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(X(f[0]), Y(f[1]), Math.max(4, (T.road.sigma || 2.5) * s), 0, 7); ctx.fill(); ctx.stroke();
    }
    // место по Wi-Fi: грубый круг, им проверяется GPS на подмену
    const wf = T.road && T.road.wifi;
    if (wf) {
      ctx.strokeStyle = 'rgba(178,120,255,.9)'; ctx.fillStyle = 'rgba(178,120,255,.08)'; ctx.lineWidth = 1.5;
      ctx.setLineDash([6, 4]);
      ctx.beginPath(); ctx.arc(X(wf[0]), Y(wf[1]), Math.max(6, wf[2] * s), 0, 7); ctx.fill(); ctx.stroke();
      ctx.setLineDash([]);
    }
    // оценка: след + стрелка
    ctx.strokeStyle = '#37c8a4'; ctx.lineWidth = 2;
    ctx.beginPath(); track.forEach(([x, y], i) => i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y))); ctx.stroke();
    if (r) {
      const [x, y, yaw] = r;
      ctx.save(); ctx.translate(X(x), Y(y)); ctx.rotate(-yaw);
      ctx.fillStyle = '#37c8a4'; ctx.strokeStyle = '#05090b'; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(12, 0); ctx.lineTo(-8, 7); ctx.lineTo(-4, 0); ctx.lineTo(-8, -7); ctx.closePath(); ctx.fill(); ctx.stroke();
      ctx.restore();
    }
  }
  ctx.fillStyle = '#8ba0ad'; ctx.fillText(`сетка ${step} м · колесо — масштаб, двойной клик — сброс`, 8, h - 8);
  setTimeout(() => requestAnimationFrame(draw), 100);  // 10 кадров/с хватает, телефону легче
}
requestAnimationFrame(draw);
