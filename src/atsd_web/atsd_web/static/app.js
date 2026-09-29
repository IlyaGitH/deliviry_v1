const qs = new URLSearchParams(location.search);
const ROLE = qs.get('role') || 'admin';
const ORDER_ID = qs.get('order') || null;

const $ = (id) => document.getElementById(id);
const cv = $('map'), ctx = cv.getContext('2d');

let CFG = { points: [], maps: {} };
let LAST = null;
let mapMode = 'auto';
let placing = null;
let localPoints = null;
const images = {};

const EVENT_RU = {
  MISSION_START: 'Рейс начат', CARGO_LOADED: 'Груз загружен, выезд', CARGO_UNLOADED: 'Груз выдан',
  ARRIVED_PICKUP: 'Прибыл на погрузку', ARRIVED_DROP: 'Прибыл на выдачу', DELIVERED: 'Доставка завершена',
  STOP_SIGN: 'Распознан знак «Стоп»', STOP_SIGN_HOLD: 'Полная остановка у знака «Стоп»',
  STOP_SIGN_GO: 'Продолжаю движение после «Стоп»', CROSSWALK_SLOW: 'Пешеходный переход — снижаю скорость',
  BUMP_SLOW: 'Искусственная неровность — снижаю скорость', RED_LIGHT_STOP: 'Красный сигнал — остановка',
  GREEN_GO: 'Зелёный сигнал — движение', OBSTACLE_STOP: 'Препятствие — остановка',
  OBSTACLE_CLEAR: 'Путь свободен', DETOUR_START: 'Объезд препятствия', DETOUR_DONE: 'Объезд завершён',
  BLOCKED_WAITING: 'Проезд перекрыт, ожидание', RECOVERY: 'Нештатная ситуация: отъезд и объезд',
  STALL: 'Застревание колёс', BUMPER: 'Касание бампера', FAULT: 'Сбой', LINK_LOST: 'Потеряна связь с Pi',
  LINK_OK: 'Связь с Pi восстановлена', GYRO_FAULT: 'Отказ гироскопа — курс по энкодерам',
  GYRO_READY: 'Гироскоп откалиброван', NUDGE_ON: 'Оператор подруливает', NUDGE_OFF: 'Подруливание закончено', GYRO_SIGN_FLIPPED: 'Знак гироскопа был перепутан — исправлен автоматически', MOTOR_MISSING: 'Отключился мотор', MOTOR_BACK: 'Мотор снова на связи',
  MOTOR_HOT: 'Перегрев мотора — снижаю скорость', MOTOR_COOL: 'Моторы остыли', BATT_LOW: 'Низкий заряд привода',
  ESTOP: 'Аварийный стоп', ESTOP_RELEASE: 'Аварийный стоп снят', MISSION_ABORT: 'Рейс отменён',
  LOCK_TAMPER: 'Крышку открыли в пути!', REJECT: 'Команда отклонена', RETURNED_BASE: 'Вернулся на базу',
  DOCK_SEARCH: 'Подъезд к точке, ищу красную разметку', DOCK_SPOT: 'Разметка найдена',
  DOCK_WAIT: 'Разметка не видна, жду', DOCKED: 'Встал на разметку, координаты уточнены',
  DOCK_NOT_FOUND: 'Разметка не найдена, стою по счислению', ZONE_OK: 'BLE-метка точки подтверждена',
  ZONE_MISMATCH: 'Рядом метка другой точки!', ZONE_UNCONFIRMED: 'BLE-метка точки не обнаружена',
  ZONE_UNCONFIRMED_UNLOCK: 'Отсек открыт без подтверждения BLE-метки',
};
const POINT_NAME = (id) => ((CFG.points || []).find(p => p.id === id) || {}).name || id;
const EVENT_SKIP = new Set(['STATE', 'IGNORED_C', 'IGNORED_U', 'POSE_RESET', 'GYRO_CALIBRATING']);
const EVENT_WARN = new Set(['FAULT', 'STALL', 'BUMPER', 'LINK_LOST', 'GYRO_FAULT', 'MOTOR_MISSING',
  'MOTOR_HOT', 'BATT_LOW', 'ESTOP', 'LOCK_TAMPER', 'REJECT', 'RECOVERY', 'BLOCKED_WAITING',
  'DOCK_NOT_FOUND', 'ZONE_MISMATCH', 'ZONE_UNCONFIRMED', 'ZONE_UNCONFIRMED_UNLOCK', 'GYRO_SIGN_FLIPPED']);
const EVENT_GOOD = new Set(['ARRIVED_PICKUP', 'ARRIVED_DROP', 'DELIVERED', 'DETOUR_DONE', 'GREEN_GO',
  'STOP_SIGN_GO', 'CARGO_LOADED', 'CARGO_UNLOADED', 'DOCKED', 'ZONE_OK']);
let lastEventSeq = 0;

function renderEvents(list) {
  const box = $('eventLog');
  (list || []).forEach(e => {
    if (e.seq <= lastEventSeq) return;
    lastEventSeq = e.seq;
    const [code, ...args] = e.text.split(' ');
    if (EVENT_SKIP.has(code)) return;
    const li = document.createElement('li');
    li.className = EVENT_WARN.has(code) ? 'warn' : (EVENT_GOOD.has(code) ? 'good' : '');
    const t = new Date(e.ts * 1000).toLocaleTimeString('ru-RU', { hour12: false });
    const text = (EVENT_RU[code] || code) + (args.length ? ' · ' + args.join(' ') : '');
    li.innerHTML = `<time>${t}</time><span></span>`;
    li.querySelector('span').textContent = text;
    box.prepend(li);
    while (box.children.length > 80) box.removeChild(box.lastChild);
  });
}

function fmtTimer(sec) {
  if (sec === null || sec === undefined) return '00:00.0';
  const m = Math.floor(sec / 60), s = sec - m * 60;
  return String(m).padStart(2, '0') + ':' + s.toFixed(1).padStart(4, '0');
}

function toast(text, err = false) {
  const t = $('toast');
  t.textContent = text;
  t.className = 'toast show' + (err ? ' err' : '');
  clearTimeout(toast._t);
  toast._t = setTimeout(() => (t.className = 'toast'), 2600);
}

function fmtEta(sec) {
  if (sec === null || sec === undefined) return '—';
  if (sec < 60) return sec + ' с';
  const m = Math.floor(sec / 60), s = sec % 60;
  return m + ' мин ' + (s < 10 ? '0' : '') + s + ' с';
}

async function api(path, opts) {
  const r = await fetch(path, opts);
  if (!r.ok) {
    let msg = 'Ошибка ' + r.status;
    try { msg = (await r.json()).detail || msg; } catch (e) {}
    throw new Error(msg);
  }
  return r.json();
}

function activeMapKey() {
  if (mapMode !== 'auto') return mapMode;
  if (!LAST) return 'indoor';
  return LAST.telemetry.indoor ? 'indoor' : 'outdoor';
}

function loadImage(key) {
  const meta = CFG.maps[key];
  if (!meta || images[key] !== undefined) return;
  const img = new Image();
  img.onload = () => { images[key] = img; draw(); };
  img.onerror = () => { images[key] = null; };
  images[key] = undefined;
  img.src = meta.image;
  images[key] = img;
}

function fitCanvas() {
  const r = cv.parentElement.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  cv.width = Math.round(r.width * dpr);
  cv.height = Math.round(r.height * dpr);
  cv.style.width = r.width + 'px';
  cv.style.height = r.height + 'px';
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function view() {
  const pts = (localPoints || CFG.points);
  const xs = pts.map(p => p.x), ys = pts.map(p => p.y);
  if (LAST) { xs.push(LAST.telemetry.pose.x); ys.push(LAST.telemetry.pose.y); }
  if (!xs.length) { xs.push(0); ys.push(0); }

  const pad = 6;
  const minX = Math.min(...xs) - pad, maxX = Math.max(...xs) + pad;
  const minY = Math.min(...ys) - pad, maxY = Math.max(...ys) + pad;

  const w = cv.clientWidth, h = cv.clientHeight;
  const s = Math.min(w / (maxX - minX), h / (maxY - minY));
  return {
    s,
    ox: (w - (maxX - minX) * s) / 2 - minX * s,
    oy: h - ((h - (maxY - minY) * s) / 2 - minY * s),
  };
}

const toScreen = (v, x, y) => [v.ox + x * v.s, v.oy - y * v.s];
const toWorld = (v, px, py) => [(px - v.ox) / v.s, (v.oy - py) / v.s];

function draw() {
  if (!cv.clientWidth) return;
  const v = view();
  const w = cv.clientWidth, h = cv.clientHeight;
  ctx.clearRect(0, 0, w, h);

  const key = activeMapKey();
  const meta = CFG.maps[key];
  const img = images[key];

  if (img && img.complete && img.naturalWidth) {
    const mpp = meta.meters_per_pixel;
    const [ox, oy] = meta.origin;
    const wm = img.naturalWidth * mpp, hm = img.naturalHeight * mpp;
    const [sx, sy] = toScreen(v, ox, oy + hm);
    ctx.globalAlpha = 0.85;
    ctx.drawImage(img, sx, sy, wm * v.s, hm * v.s);
    ctx.globalAlpha = 1;
  } else {
    ctx.strokeStyle = '#1b262e';
    ctx.lineWidth = 1;
    const step = 5;
    for (let x = -100; x <= 200; x += step) {
      const [px] = toScreen(v, x, 0);
      ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, h); ctx.stroke();
    }
    for (let y = -100; y <= 200; y += step) {
      const [, py] = toScreen(v, 0, y);
      ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(w, py); ctx.stroke();
    }
  }

  const pts = localPoints || CFG.points;
  const targetId = LAST && LAST.order ? LAST.order.target : null;

  if (LAST && targetId) {
    const t = pts.find(p => p.id === targetId);
    if (t) {
      const [ax, ay] = toScreen(v, LAST.telemetry.pose.x, LAST.telemetry.pose.y);
      const [bx, by] = toScreen(v, t.x, t.y);
      ctx.strokeStyle = 'rgba(75,157,255,.55)';
      ctx.lineWidth = 2; ctx.setLineDash([7, 6]);
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  pts.forEach(p => {
    const [x, y] = toScreen(v, p.x, p.y);
    const isTarget = p.id === targetId;
    ctx.fillStyle = isTarget ? '#4b9dff' : '#7e8ea0';
    ctx.beginPath(); ctx.arc(x, y, isTarget ? 9 : 7, 0, 7); ctx.fill();
    ctx.fillStyle = '#0d1418';
    ctx.beginPath(); ctx.arc(x, y, 3, 0, 7); ctx.fill();
    ctx.fillStyle = '#e8eef2';
    ctx.font = '600 12px IBM Plex Sans, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(p.name, x, y - 14);
  });

  if (LAST) {
    const { x: rx, y: ry, yaw } = LAST.telemetry.pose;
    const [x, y] = toScreen(v, rx, ry);
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-yaw);
    ctx.fillStyle = '#37c8a4';
    ctx.beginPath();
    ctx.moveTo(15, 0); ctx.lineTo(-9, 9); ctx.lineTo(-4, 0); ctx.lineTo(-9, -9);
    ctx.closePath(); ctx.fill();
    ctx.restore();
    ctx.strokeStyle = 'rgba(55,200,164,.35)';
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(x, y, 17, 0, 7); ctx.stroke();
  }

  $('mapHint').textContent =
    (meta ? meta.label : '') + (img && img.complete && img.naturalWidth ? '' : ' · сетка 5 м');
}

function render(msg) {
  LAST = msg;
  const t = msg.telemetry, o = msg.order;

  if (msg.points && !localPoints) CFG.points = msg.points;

  const online = t.connected && t.brain_link;
  $('linkState').className = 'link' + (online ? ' ok' : '');
  $('linkState').querySelector('span').textContent = online ? 'робот на связи' : 'нет связи с роботом';

  $('statusText').textContent = o ? o.status_ru : 'Нет активного заказа';
  $('routeText').textContent = o ? `${o.from_name} → ${o.to_name}` : '';

  const seq = ['to_pickup', 'at_pickup', 'in_transit', 'at_dropoff', 'delivered'];
  const cur = o ? seq.indexOf(o.status) : -1;
  document.querySelectorAll('.steps li').forEach(li => {
    const i = seq.indexOf(li.dataset.s);
    li.className = i < cur ? 'done' : (i === cur ? 'now' : '');
  });

  $('missionTimer').textContent = fmtTimer(o ? o.mission_s : null);
  const banner = $('pauseBanner');
  if (o && o.fault) {
    banner.className = 'banner fault';
    banner.textContent = 'Сбой на роботе. Смотрите журнал; администратор может продолжить рейс.';
  } else if (o && o.pause_ru && o.pause_ru.length) {
    banner.className = 'banner';
    banner.textContent = 'Пауза: ' + o.pause_ru.join(', ');
  } else {
    banner.className = 'banner hidden';
  }
  $('btnResume').disabled = !(o && o.fault);
  $('btnCancel').disabled = !o;

  $('etaPickup').textContent = o ? fmtEta(o.eta_pickup) : '—';
  $('etaDropoff').textContent = o ? fmtEta(o.eta_dropoff) : '—';

  const lb = $('lockBox');
  if (t.lock.busy) {
    lb.className = 'lock busy';
    $('lockText').textContent = 'замок срабатывает';
  } else if (!t.lock.closed) {
    lb.className = 'lock open';
    $('lockText').textContent = 'крышка открыта';
  } else {
    lb.className = 'lock';
    $('lockText').textContent = 'крышка закрыта';
  }

  setGauge('bDrive', t.battery.drive);
  setGauge('bComp', t.battery.compute);

  $('chipSpeed').textContent = t.speed.toFixed(1).replace('.', ',') + ' м/с';
  $('chipWhere').textContent = t.indoor ? 'помещение' : 'улица';
  $('chipEstop').classList.toggle('hidden', !t.estop);
  $('chipTamper').classList.toggle('hidden', !(t.lock && t.lock.tamper));
  $('chipBrain').classList.toggle('hidden', !!t.brain_link);
  $('chipObstacle').textContent = t.obstacle || 'коридор свободен';
  const ble = t.ble || {};
  $('chipZone').textContent = ble.zone
    ? `BLE: ${POINT_NAME(ble.zone)} ${ble.rssi} дБм${ble.near ? ' · рядом' : ''}`
    : 'BLE: меток нет';
  $('chipZone').classList.toggle('ok', !!ble.near);
  renderEvents(msg.events);
  $('coord').textContent =
    `x ${t.pose.x.toFixed(1)}  y ${t.pose.y.toFixed(1)}`.replace(/\./g, ',');

  updateActions(o);
  draw();
}

function setGauge(prefix, b) {
  const bar = $(prefix + 'Bar'), val = $(prefix + 'Val');
  if (!b || b.percent === null || b.percent === undefined) {
    bar.style.width = '0%'; val.textContent = 'нет данных'; return;
  }
  const p = Math.max(0, Math.min(100, b.percent));
  bar.style.width = p + '%';
  bar.className = p < 20 ? 'low' : (p < 40 ? 'mid' : '');
  val.textContent = Math.round(p) + ' %' + (b.volts ? '  ·  ' + b.volts + ' В' : '');
}

function updateActions(o) {
  const bu = $('btnUnlock'), bd = $('btnDone'), hint = $('actionHint');

  if (ROLE === 'admin') {
    $('actionPanel').classList.add('hidden');
    return;
  }
  $('actionPanel').classList.remove('hidden');

  if (!o || (ORDER_ID && o.id !== ORDER_ID)) {
    bu.disabled = bd.disabled = true;
    hint.textContent = 'Заказ не активен.';
    return;
  }

  const mine = (ROLE === 'sender' && o.status === 'at_pickup') ||
               (ROLE === 'receiver' && o.status === 'at_dropoff');

  bu.disabled = !mine;
  bd.disabled = !mine;
  bd.textContent = ROLE === 'sender' ? 'Готово, груз внутри' : 'Готово, груз забрал';

  if (mine) {
    hint.textContent = ROLE === 'sender'
      ? 'Откройте отсек, положите груз и нажмите «Готово». Открыть можно повторно.'
      : 'Откройте отсек, заберите груз и нажмите «Готово».';
  } else if (o.status === 'in_transit') {
    hint.textContent = 'Робот в пути. Отсек заблокирован.';
  } else {
    hint.textContent = 'Дождитесь прибытия робота.';
  }
}

$('btnUnlock').onclick = async () => {
  try {
    const o = LAST && LAST.order;
    await api(`/api/orders/${o.id}/unlock?role=${ROLE}`, { method: 'POST' });
    toast('Отсек открыт');
  } catch (e) { toast(e.message, true); }
};

$('btnResume').onclick = async () => {
  try { await api('/api/mission/resume', { method: 'POST' }); toast('Рейс продолжен'); }
  catch (e) { toast(e.message, true); }
};

$('btnCancel').onclick = async () => {
  const o = LAST && LAST.order;
  if (!o || !confirm('Отменить рейс? Робот остановится.')) return;
  try { await api(`/api/orders/${o.id}/cancel`, { method: 'POST' }); toast('Рейс отменён'); }
  catch (e) { toast(e.message, true); }
};

$('btnDone').onclick = async () => {
  try {
    const o = LAST && LAST.order;
    await api(`/api/orders/${o.id}/done?role=${ROLE}`, { method: 'POST' });
    toast(ROLE === 'sender' ? 'Команда отправлена роботу' : 'Выдача подтверждена');
  } catch (e) { toast(e.message, true); }
};

function fillSelects() {
  ['selFrom', 'selTo'].forEach(id => {
    const s = $(id);
    s.innerHTML = '';
    CFG.points.forEach(p => {
      const opt = document.createElement('option');
      opt.value = p.id; opt.textContent = p.name;
      s.appendChild(opt);
    });
  });
  if (CFG.points.length > 1) $('selTo').selectedIndex = 1;
}

function renderPointList() {
  const box = $('pointList');
  box.innerHTML = '';
  (localPoints || CFG.points).forEach(p => {
    const row = document.createElement('div');
    row.className = 'prow' + (placing === p.id ? ' sel' : '');
    row.innerHTML = `<b>${p.name}</b><span>${p.x.toFixed(1)} ; ${p.y.toFixed(1)}</span>`;
    row.onclick = () => {
      placing = placing === p.id ? null : p.id;
      $('btnPlace').classList.toggle('on', !!placing);
      renderPointList();
      toast(placing ? `Ткните в карту: ${p.name}` : 'Расстановка выключена');
    };
    box.appendChild(row);
  });
}

$('btnPlace').onclick = () => {
  if (!localPoints) localPoints = JSON.parse(JSON.stringify(CFG.points));
  placing = placing ? null : (localPoints[0] && localPoints[0].id);
  $('btnPlace').classList.toggle('on', !!placing);
  renderPointList();
  toast(placing ? 'Выберите точку в списке и ткните в карту' : 'Расстановка выключена');
};

cv.addEventListener('pointerdown', (ev) => {
  if (!placing || !localPoints) return;
  const r = cv.getBoundingClientRect();
  const v = view();
  const [wx, wy] = toWorld(v, ev.clientX - r.left, ev.clientY - r.top);
  const p = localPoints.find(q => q.id === placing);
  if (p) { p.x = Math.round(wx * 10) / 10; p.y = Math.round(wy * 10) / 10; }
  renderPointList();
  draw();
});

$('btnSavePoints').onclick = async () => {
  if (!localPoints) { toast('Ничего не менялось'); return; }
  try {
    await api('/api/points', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ points: localPoints }),
    });
    CFG.points = localPoints; localPoints = null; placing = null;
    $('btnPlace').classList.remove('on');
    renderPointList(); fillSelects();
    toast('Расстановка сохранена');
  } catch (e) { toast(e.message, true); }
};

$('btnCreate').onclick = async () => {
  try {
    const o = await api('/api/orders', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ from_id: $('selFrom').value, to_id: $('selTo').value }),
    });
    const base = location.origin;
    $('lkSender').value = `${base}/?role=sender&order=${o.id}&t=${o.sender_token}`;
    $('lkReceiver').value = `${base}/?role=receiver&order=${o.id}&t=${o.receiver_token}`;
    $('orderLinks').classList.remove('hidden');
    toast('Заказ создан, робот выехал');
  } catch (e) { toast(e.message, true); }
};

document.querySelectorAll('#mapSeg button').forEach(b => {
  b.onclick = () => {
    document.querySelectorAll('#mapSeg button').forEach(x => x.classList.remove('on'));
    b.classList.add('on');
    mapMode = b.dataset.map;
    loadImage(activeMapKey());
    draw();
  };
});

document.querySelectorAll('.lk input').forEach(i => {
  i.onclick = () => { i.select(); document.execCommand('copy'); toast('Ссылка скопирована'); };
});

function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const sock = new WebSocket(`${proto}://${location.host}/ws`);
  sock.onmessage = (e) => render(JSON.parse(e.data));
  sock.onclose = () => {
    $('linkState').className = 'link';
    $('linkState').querySelector('span').textContent = 'переподключение';
    setTimeout(connect, 1500);
  };
}

async function boot() {
  if (ROLE !== 'admin') {
    document.querySelectorAll('.admin-only').forEach(e => e.classList.add('hidden'));
    $('roleLabel').textContent =
      ROLE === 'sender' ? 'вы отправитель' : 'вы получатель';
  }
  CFG = await api('/api/config');
  fillSelects();
  renderPointList();
  loadImage('indoor'); loadImage('outdoor');
  fitCanvas(); draw();
  connect();
}

window.addEventListener('resize', () => { fitCanvas(); draw(); });
boot();
