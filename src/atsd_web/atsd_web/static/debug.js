// Камера, локальная сетка и консоль событий.
const $ = id => document.getElementById(id);
let CFG = { points: [], roads: { nodes: {}, edges: [] } };
let T = null;                 // последняя телеметрия
const track = []; // история за последние ~2 мин
let paused = false;
let view = null;              // {cx, cy, scale} — подбирается сам, колесо/щипок меняют масштаб

fetch('/api/config').then(r => r.json()).then(c => { CFG = c; view = null; });

// ---------- камера ----------
const cam = $('cam');
const sleep = ms => new Promise(r => setTimeout(r, ms));
// по одному кадру: следующий просим, только показав этот — задержка не копится на медленном Wi-Fi
async function camLoop() {
  let seq = -1, url = null;
  for (;;) {
    if (document.hidden) { await sleep(1000); continue; }
    const started = performance.now();
    try {
      const r = await fetch(`/debug/frame.jpg?after=${seq}`, { cache: 'no-store', signal: AbortSignal.timeout(5000) });
      if (r.status !== 200) { await sleep(500); continue; }
      seq = +r.headers.get('X-Seq');
      const next = URL.createObjectURL(await r.blob());
      cam.src = next;
      await cam.decode().catch(() => {});
      if (url) URL.revokeObjectURL(url);
      url = next;
      await sleep(Math.max(0, 100 - (performance.now() - started)));
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
  const d = document.createElement('div');
  d.className = /SUSPECT|FAILED|LOST|ESTOP/.test(text) ? 'alarm' : kind;
  d.textContent = `${hhmmss(ts)} ${text}`;
  con.appendChild(d);
  while (con.childElementCount > MAX_LINES) con.firstChild.remove();
}
$('pause').onclick = e => { paused = !paused; e.target.textContent = paused ? 'Продолжить' : 'Пауза'; };
$('clear').onclick = () => { con.textContent = ''; };

let lastSummary = 0;
function summary(t) {
  const r = t.road || {};
  return r.pose ? `Локальная поза: x ${r.pose[0].toFixed(1)}, y ${r.pose[1].toFixed(1)}, курс ${(r.pose[2] * 180 / Math.PI).toFixed(0)}°` : 'Укажите начальную позицию «Робот здесь»';
}

// ---------- телеметрия ----------
function connect() {
  const ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/debug/ws');
  ws.onmessage = e => {
    const t = JSON.parse(e.data);
    T = t;
    t.lines.forEach(l => log(l.ts, l.kind, l.text));
    if (t.ts - lastSummary >= 1) { lastSummary = t.ts; log(t.ts, 'state', summary(t)); }
    if (!paused && $('auto').checked) con.scrollTop = con.scrollHeight;
    const r = t.road || {};
    const now = t.ts;
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

function chip(text, cls) {
  const element = document.createElement('span');
  element.className = 'chip ' + (cls || '');
  element.textContent = text;
  return element.outerHTML;
}
function chips(t) {
  const r = t.road || {};
  const out = [chip(r.ready ? 'Локальная позиция задана' : 'Укажите «Робот здесь»', r.ready ? 'ok' : 'warn')];
  const lane = (t.lane || '').split(' ');
  if (lane.length >= 4 && lane[0] !== 'none') {
    out.push(chip(`разметка: линий ${lane[3]}, смещение ${(+lane[0] * 100).toFixed(0)} см, угол ${(+lane[1] * 180 / Math.PI).toFixed(1)}°, полоса ${(+lane[2]).toFixed(2)} м`, 'ok'));
  } else out.push(chip('разметки не вижу', 'warn'));
  if (t.imu_status) out.push(chip(t.imu_status.replace(/^imu /, 'гироскоп '), /ошибка|калибруется/.test(t.imu_status) ? 'warn' : 'ok'));
  else out.push(chip('гироскопа нет', 'warn'));
  const lo = t.lidar_odom || {};
  if (lo.reason) {
    const reasons = { initializing: 'сбор сканов', scan_gap: 'пропуск скана', stale_scan: 'нет свежих сканов',
      few_returns: 'мало отражений', degenerate: 'движение неразличимо', low_overlap: 'мало совпадений',
      budget: 'долгий расчёт', not_converged: 'нет сходимости', residual: 'низкое качество',
      large_step: 'скачок', implausible_motion: 'неправдоподобное движение',
      stale_after_matching: 'результат устарел', stale_or_wrong_frame: 'скан устарел или неверная система координат' };
    out.push(chip('Лидарная одометрия: ' + (lo.valid ? 'измерение принято (наблюдение)' : (reasons[lo.reason] || 'нет измерения')), lo.valid ? 'ok' : 'warn'));
    if (lo.valid && lo.slip_suspected) out.push(chip('Подозрение на пробуксовку', 'warn'));
    if (lo.valid && !lo.wheel_comparison) out.push(chip('Нет синхронной одометрии колёс', 'warn'));
  }
  out.push(chip('режим ' + (r.mode || '—')));
  if (r.mode !== 'idle' && r.avoidance) {
    const states = { following: 'путь свободен', avoid: 'объезд по лидару', gear_stop: 'остановка перед сменой направления',
      blocked: 'свободного манёвра нет', braking_blocked: 'препятствие на пути торможения',
      stale_scan: 'нет свежего лидара', stale_odometry: 'нет свежей одометрии', waiting_scan: 'ожидание лидара',
      stale_after_planning: 'данные устарели', stalled: 'нет движения', escape_timeout: 'объезд не найден', local_goal: 'у цели' };
    out.push(chip(states[r.avoidance] || 'остановка планировщика', ['following', 'avoid'].includes(r.avoidance) ? 'ok' : 'warn'));
    if (r.reverse_m > 0) out.push(chip(`откат ${r.reverse_m.toFixed(2)} м`));
  }
  out.push(chip(`${(t.speed || 0).toFixed(2)} м/с`));
  if (t.state) out.push(chip('рейс ' + t.state + (t.pause && t.pause.length ? ' (пауза: ' + t.pause.join(', ') + ')' : '')));
  if (t.obstacle) out.push(chip(t.obstacle, /свобод/.test(t.obstacle) ? 'ok' : 'bad'));
  if (!t.brain_link) out.push(chip('нет связи с Brain', 'bad'));
  $('chips').innerHTML = out.join('');
}

// ---------- карта ----------
const cv = $('dmap'), ctx = cv.getContext('2d');
function fit(w, h) {
  const b = Grid100.bounds(CFG.grid);
  return {cx: b.cx, cy: b.cy, scale: Math.max(1,Math.min(w,h)-48)/100};
}
let userZoom = 1;
cv.addEventListener('wheel', e => { e.preventDefault(); userZoom = Math.min(20, Math.max(.3, userZoom * (e.deltaY < 0 ? 1.25 : .8))); }, { passive: false });
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

  Grid100.draw(ctx, CFG.grid, {s, ox:w/2-cx*s, oy:h/2+cy*s});

  // дороги
  const nodes = CFG.roads.nodes || {};
  ctx.strokeStyle = '#56656f'; ctx.lineWidth = Math.max(2, 3 * s / 10);
  (CFG.roads.edges || []).forEach(([a, b]) => {
    if (!nodes[a] || !nodes[b]) return;
    ctx.beginPath(); ctx.moveTo(X(nodes[a][0]), Y(nodes[a][1])); ctx.lineTo(X(nodes[b][0]), Y(nodes[b][1])); ctx.stroke();
  });
  ctx.fillStyle = '#8ba0ad'; ctx.font = '11px sans-serif';
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
  ctx.fillStyle = '#8ba0ad'; ctx.fillText(`сетка 100 × 100 м, шаг 5 м · колесо — масштаб, двойной клик — сброс`, 8, h - 8);
  setTimeout(() => requestAnimationFrame(draw), 100);  // 10 кадров/с хватает, телефону легче
}
requestAnimationFrame(draw);
