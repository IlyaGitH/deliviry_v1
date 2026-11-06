const qs = new URLSearchParams(location.search);
const ROLE = qs.get('role') || 'admin';
const ORDER_ID = qs.get('order') || null;

const $ = (id) => document.getElementById(id);
const cv = $('map'), ctx = cv.getContext('2d');

let CFG = { points: [], maps: {} };
let LAST = null;
const MIN_SPAN = 150;
let mapZoom = 1;      // колесо мыши над картой
let mapMode = 'map';  // подложка Яндекса: map — схема, sat — спутник
let placing = null;
let settingPose = null;  // null | 'pos' | {x, y} — «Робот здесь»
let localPoints = null;
let roadEditor = null, roadView = null, roadRevision = null, roadDelete = false;
let previewPlan = null;
let mapPan = {x:0,y:0};
const images = {};

const EVENT_RU = {
  MISSION_START: 'Рейс начат', CARGO_LOADED: 'Груз загружен, выезд', CARGO_UNLOADED: 'Груз выдан',
  ARRIVED_PICKUP: 'Прибыл на погрузку', ARRIVED_DROP: 'Прибыл на выдачу', DELIVERED: 'Доставка завершена',
  STOP_SIGN: 'Распознан знак «Стоп»', STOP_SIGN_HOLD: 'Полная остановка у знака «Стоп»',
  STOP_SIGN_GO: 'Продолжаю движение после «Стоп»', CROSSWALK_SLOW: 'Пешеходный переход — снижаю скорость',
  BUMP_SLOW: 'Искусственная неровность — снижаю скорость', RED_LIGHT_STOP: 'Красный сигнал — остановка',
  GREEN_GO: 'Зелёный сигнал — движение', OBSTACLE_STOP: 'Препятствие — остановка',
  OBSTACLE_CLEAR: 'Путь свободен', NAV_FAILED: 'Не удалось проложить или пройти путь по карте',
  RECOVERY: 'Нештатная ситуация: отъезд назад',
  STALL: 'Застревание колёс', BUMPER: 'Касание бампера', FAULT: 'Сбой', LINK_LOST: 'Потеряна связь с Pi',
  LINK_OK: 'Связь с Pi восстановлена', GYRO_FAULT: 'Отказ гироскопа — курс по энкодерам',
  GYRO_READY: 'Гироскоп откалиброван', NUDGE_ON: 'Оператор подруливает', NUDGE_OFF: 'Подруливание закончено', GYRO_SIGN_FLIPPED: 'Знак гироскопа был перепутан — исправлен автоматически', MOTOR_MISSING: 'Отключился мотор', MOTOR_BACK: 'Мотор снова на связи',
  MOTOR_HOT: 'Перегрев мотора — снижаю скорость', MOTOR_COOL: 'Моторы остыли', BATT_LOW: 'Низкий заряд привода',
  ESTOP: 'Аварийный стоп', ESTOP_RELEASE: 'Аварийный стоп снят', MISSION_ABORT: 'Рейс отменён',
  LOCK_TAMPER: 'Крышку открыли в пути!', REJECT: 'Команда отклонена', RETURNED_BASE: 'Вернулся на базу',
  ZONE_OK: 'BLE-метка точки подтверждена',
  ZONE_MISMATCH: 'Рядом метка другой точки!', ZONE_UNCONFIRMED: 'BLE-метка точки не обнаружена',
  ZONE_UNCONFIRMED_UNLOCK: 'Отсек открыт без подтверждения BLE-метки',
};
const POINT_NAME = (id) => ((CFG.points || []).find(p => p.id === id) || {}).name || id;
EVENT_RU.POSE_SET = 'Положение робота задано вручную';
const EVENT_SKIP = new Set(['STATE', 'IGNORED_C', 'IGNORED_U', 'GYRO_CALIBRATING']);
const EVENT_WARN = new Set(['FAULT', 'STALL', 'BUMPER', 'LINK_LOST', 'GYRO_FAULT', 'MOTOR_MISSING',
  'MOTOR_HOT', 'BATT_LOW', 'ESTOP', 'LOCK_TAMPER', 'REJECT', 'RECOVERY', 'NAV_FAILED',
  'ZONE_MISMATCH', 'ZONE_UNCONFIRMED', 'ZONE_UNCONFIRMED_UNLOCK', 'GYRO_SIGN_FLIPPED']);
const EVENT_GOOD = new Set(['ARRIVED_PICKUP', 'ARRIVED_DROP', 'DELIVERED', 'OBSTACLE_CLEAR', 'GREEN_GO',
  'STOP_SIGN_GO', 'CARGO_LOADED', 'CARGO_UNLOADED', 'ZONE_OK']);
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
  return mapMode;
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

// место робота известно: GPS, «Робот здесь» или (без nav_node) нет сведений об обратном
function poseKnown() {
  return !!LAST && (LAST.telemetry.road || {}).ready !== false;
}

function view() {
  if (roadView) return roadView;
  const pts = (localPoints || CFG.points);
  const xs = pts.map(p => p.x), ys = pts.map(p => p.y);
  if (poseKnown()) { xs.push(LAST.telemetry.pose.x); ys.push(LAST.telemetry.pose.y); }
  Object.values((CFG.roads || {}).nodes || {}).forEach(([x, y]) => { xs.push(x); ys.push(y); });
  if (!xs.length) { xs.push(0); ys.push(0); }

  const pad = 6;
  const minX = Math.min(...xs) - pad, maxX = Math.max(...xs) + pad;
  const minY = Math.min(...ys) - pad, maxY = Math.max(...ys) + pad;

  // не меньше MIN_SPAN м: карта Яндекса растянутой до пары точек — каша из пикселей
  const w = cv.clientWidth, h = cv.clientHeight;
  const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
  const s = Math.min(w / Math.max(maxX - minX, MIN_SPAN), h / Math.max(maxY - minY, MIN_SPAN * h / w)) * mapZoom;
  return { s, ox: w / 2 - cx * s + mapPan.x*s, oy: h / 2 + cy * s + mapPan.y*s };
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

  const onMap = !!(img && img.complete && img.naturalWidth);  // светлая карта — тёмные линии и подписи
  if (onMap) {
    const mpp = meta.meters_per_pixel;
    const [ox, oy] = meta.origin;
    const wm = img.naturalWidth * mpp, hm = img.naturalHeight * mpp;
    const [sx, sy] = toScreen(v, ox, oy + hm);
    ctx.drawImage(img, sx, sy, wm * v.s, hm * v.s);
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

  // граф дорог (roads.json): по нему строится маршрут
  const roads = roadEditor ? roadEditor.data : (CFG.roads || {});
  const roadEdges = [...(roads.edges || []), ...(roads.directed_edges || [])];
  ctx.strokeStyle = onMap ? 'rgba(13,20,24,.4)' : 'rgba(232,238,242,.28)';
  ctx.lineWidth = 6; ctx.lineCap = 'round';
  roadEdges.forEach(([a, b]) => {
    const na = roads.nodes[a], nb = roads.nodes[b];
    if (!na || !nb) return;
    const [ax, ay] = toScreen(v, na[0], na[1]), [bx, by] = toScreen(v, nb[0], nb[1]);
    ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
  });

  (roads.directed_edges || []).forEach(([a,b]) => {
    const A=roads.nodes[a],B=roads.nodes[b];if(!A||!B)return;
    const [x,y]=toScreen(v,(A[0]+B[0])/2,(A[1]+B[1])/2);
    ctx.save();ctx.translate(x,y);ctx.rotate(-Math.atan2(B[1]-A[1],B[0]-A[0]));
    ctx.strokeStyle='#bd7020';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(-5,-4);ctx.lineTo(2,0);ctx.lineTo(-5,4);ctx.stroke();ctx.restore();
  });
  if (roadEditor) Object.entries(roads.nodes).forEach(([id,p])=>{
    const [x,y]=toScreen(v,...p);ctx.fillStyle=id===roadEditor.selected?'#ff943d':'#ce7443';
    ctx.beginPath();ctx.arc(x,y,id===roadEditor.selected?6:3,0,7);ctx.fill();
  });

  // место по Wi-Fi (±десятки метров) и насколько робот уверен в своём месте
  const road = (LAST && LAST.telemetry.road) || {};
  if (road.wifi) {
    const [wx, wy] = toScreen(v, road.wifi[0], road.wifi[1]);
    ctx.strokeStyle = 'rgba(140,70,230,.9)'; ctx.fillStyle = 'rgba(140,70,230,.10)';
    ctx.lineWidth = 2; ctx.setLineDash([8, 5]);
    ctx.beginPath(); ctx.arc(wx, wy, Math.max(8, road.wifi[2] * v.s), 0, 7); ctx.fill(); ctx.stroke();
    ctx.setLineDash([]);
  }
  if (poseKnown() && road.sigma_xy > 2) {
    const [px, py] = toScreen(v, LAST.telemetry.pose.x, LAST.telemetry.pose.y);
    ctx.fillStyle = 'rgba(55,200,164,.15)';
    ctx.beginPath(); ctx.arc(px, py, road.sigma_xy * v.s, 0, 7); ctx.fill();
  }

  const pts = localPoints || CFG.points;
  const targetId = LAST && LAST.order ? LAST.order.target : null;

  // Draw actual road paths only; there is no straight-line fallback.
  const actualPlan = LAST && LAST.telemetry.plan;
  const moving = LAST && LAST.order && ['TO_PICKUP', 'TO_DROP', 'RETURN'].includes(LAST.order.brain_state);
  const plan = moving ? actualPlan : previewPlan;
  if (plan && plan.length > 1) {
    ctx.strokeStyle = '#4b9dff';
    ctx.lineWidth = 3; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    ctx.beginPath();
    plan.forEach(([px, py], i) => {
      const [sx, sy] = toScreen(v, px, py);
      if (i) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy);
    });
    ctx.stroke();
  }

  pts.forEach(p => {
    const [x, y] = toScreen(v, p.x, p.y);
    const isTarget = p.id === targetId;
    ctx.fillStyle = isTarget ? '#4b9dff' : '#7e8ea0';
    ctx.beginPath(); ctx.arc(x, y, isTarget ? 9 : 7, 0, 7); ctx.fill();
    ctx.fillStyle = '#0d1418';
    ctx.beginPath(); ctx.arc(x, y, 3, 0, 7); ctx.fill();
    ctx.font = '600 12px IBM Plex Sans, sans-serif';
    ctx.textAlign = 'center';
    ctx.lineWidth = 3; ctx.lineJoin = 'round';
    ctx.strokeStyle = onMap ? 'rgba(255,255,255,.9)' : 'rgba(13,20,24,.9)';  // обводка — читается на любом фоне
    ctx.strokeText(p.name, x, y - 14);
    ctx.fillStyle = onMap ? '#0d1418' : '#e8eef2';
    ctx.fillText(p.name, x, y - 14);
  });

  if (poseKnown()) {
    const { x: rx, y: ry, yaw } = LAST.telemetry.pose;
    const [x, y] = toScreen(v, rx, ry);
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-yaw);
    ctx.fillStyle = '#37c8a4';
    ctx.beginPath();
    ctx.moveTo(15, 0); ctx.lineTo(-9, 9); ctx.lineTo(-4, 0); ctx.lineTo(-9, -9);
    ctx.closePath(); ctx.fill();
    ctx.strokeStyle = '#0d1418'; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.restore();
    ctx.strokeStyle = 'rgba(20,140,110,.6)';
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(x, y, 17, 0, 7); ctx.stroke();
  }

  $('mapHint').textContent =
    (meta ? meta.label + ' · колесо — масштаб, Shift + перетаскивание — сдвиг' : '') + (img && img.complete && img.naturalWidth ? '' : ' · сетка 5 м')
    + (LAST && !poseKnown() ? ' · где робот — неизвестно: «Робот здесь»' : '');
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
  const r = t.road || {};
  const gnss = { ok: 'GPS ок', suspect: 'GPS не верю (глушение?)', none: 'GPS нет' }[r.gnss] || '';
  const lane = r.lane ? `разметка ${r.lane === 2 ? '2 линии' : '1 линия'}` : 'разметки нет';
  $('coord').textContent = `${gnss} · ${lane} · ` +
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
  if(roadEditor){toast('Сохраните или отмените правки дорог',true);return;}
  previewPlan=null;
  if (!localPoints) localPoints = JSON.parse(JSON.stringify(CFG.points));
  placing = placing ? null : (localPoints[0] && localPoints[0].id);
  $('btnPlace').classList.toggle('on', !!placing);
  renderPointList();
  toast(placing ? 'Выберите точку в списке и ткните в карту' : 'Расстановка выключена');
};

$('btnHere').onclick = () => {
  if(roadEditor){toast('Сохраните или отмените правки дорог',true);return;}
  previewPlan=null;
  settingPose = settingPose ? null : 'pos';
  $('btnHere').classList.toggle('on', !!settingPose);
  toast(settingPose ? 'Ткните в карту, где робот стоит на самом деле' : 'Отменено');
};

cv.addEventListener('wheel', (ev) => {
  ev.preventDefault();
  if (roadView) {
    const factor=ev.deltaY<0?1.25:.8; const px=cv.clientWidth/2,py=cv.clientHeight/2;
    roadView={s:roadView.s*factor,ox:px+(roadView.ox-px)*factor,oy:py+(roadView.oy-py)*factor};draw();return;
  }
  mapZoom = Math.min(20, Math.max(0.3, mapZoom * (ev.deltaY < 0 ? 1.25 : 0.8)));
  draw();
}, { passive: false });

cv.addEventListener('pointerdown', async (ev) => {
  if(ev.shiftKey || ev.button===1)return;
  const r = cv.getBoundingClientRect();
  const v = view();
  const [wx, wy] = toWorld(v, ev.clientX - r.left, ev.clientY - r.top);
  if (roadEditor) { roadEditor.click(wx,wy,Math.min(3,12/v.s),roadDelete); previewPlan=null; draw(); return; }
  if (settingPose === 'pos') {
    settingPose = { x: wx, y: wy };
    toast('Теперь ткните, куда робот смотрит');
    return;
  }
  if (settingPose) {
    const pose = { ...settingPose, yaw: Math.atan2(wy - settingPose.y, wx - settingPose.x) };
    settingPose = null;
    $('btnHere').classList.remove('on');
    try {
      await api('/api/initialpose', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(pose),
      });
      toast('Поза задана, проверьте стрелку при езде');
    } catch (e) { toast(e.message, true); }
    return;
  }
  if (!placing || !localPoints) return;
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
    previewPlan=null;
    CFG.points = localPoints; localPoints = null; placing = null;
    $('btnPlace').classList.remove('on');
    renderPointList(); fillSelects();
    toast('Расстановка сохранена');
  } catch (e) { toast(e.message, true); }
};

$('btnCreate').onclick = async () => {
  if(roadEditor || localPoints){toast('Сначала сохраните дороги и точки',true);return;}
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
  roadStatus();
  fillSelects();
  renderPointList();
  loadImage('map'); loadImage('sat');
  // карты ещё нет (нет интернета или места робота) — спрашиваем снова, пока не появится
  const retry = setInterval(async () => {
    if (Object.keys(CFG.maps || {}).length) return clearInterval(retry);
    try {
      const current=await api('/api/config');CFG.maps=current.maps;
      if(!roadEditor){CFG.roads=current.roads;CFG.roads_revision=current.roads_revision;roadStatus();}
      loadImage('map');loadImage('sat');draw();
    } catch (e) {}
  }, 15000);
  fitCanvas(); draw();
  connect();
}

window.addEventListener('resize', () => { fitCanvas(); draw(); });
boot();


function roadStatus() {
  const r=CFG.roads||{},n=(r.edges||[]).length+(r.directed_edges||[]).length;
  $('roadsStatus').textContent=n ? `${n} участков сохранено. Точки должны быть рядом с дорогой (до ${r.snap_m||3} м).` : 'Дорог пока нет: загрузите их или отметьте по карте. Прямой маршрут отключён.';
}
function endRoadEdit(){roadEditor=null;roadView=null;roadDelete=false;$('roadTools').classList.add('hidden');$('btnRoadDelete').classList.remove('on');draw();}
$('btnEditRoads').onclick=()=>{
  if(localPoints){toast('Сначала сохраните расстановку точек',true);return;}
  if(roadEditor)return;
  roadView={...view()};roadRevision=CFG.roads_revision;roadEditor=new RoadEditor(CFG.roads);
  settingPose=null;placing=null;previewPlan=null;$('btnHere').classList.remove('on');$('btnPlace').classList.remove('on');
  $('roadTools').classList.remove('hidden');toast('Нажимайте на повороты дороги. Для ответвления нажмите «Новая ветка».');draw();
};
$('btnRoadNew').onclick=()=>{if(roadEditor)roadEditor.selected=null;roadDelete=false;$('btnRoadDelete').classList.remove('on');draw();};
$('btnRoadUndo').onclick=()=>{roadEditor?.undo();draw();};
$('btnRoadDelete').onclick=()=>{roadDelete=!roadDelete;$('btnRoadDelete').classList.toggle('on',roadDelete);};
$('btnCancelRoads').onclick=endRoadEdit;
$('btnSaveRoads').onclick=async()=>{
  if(!roadEditor)return;
  try{const r=await api('/api/roads',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({roads:roadEditor.data,revision:roadRevision})});
    Object.assign(CFG,r);endRoadEdit();previewPlan=null;roadStatus();toast('Дороги сохранены');
  }catch(e){toast(e.message,true);}
};
$('btnImportRoads').onclick=async()=>{
  if(roadEditor){toast('Сохраните или отмените правки дорог',true);return;}
  const r=CFG.roads||{};
  if(((r.edges||[]).length+(r.directed_edges||[]).length)>0 && !confirm('Заменить дороги данными OpenStreetMap? Предыдущая версия сохранится в резервной копии.'))return;
  let origin=r.origin;
  if(!origin){const value=prompt('Центр карты: широта, долгота (например 53.5078, 49.4204). Возьмите координаты нужного места на Яндекс Картах.');if(value===null)return;origin=value.trim().split(/[,;\s]+/).map(Number);}
  const value=prompt('Радиус загрузки дорог в метрах (100–1500)', '600');if(value===null)return;
  $('btnImportRoads').disabled=true;$('roadsStatus').textContent='Загружаю дороги… Нужен интернет на Pi.';
  try{const result=await api('/api/roads/import',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({revision:CFG.roads_revision,origin,radius_m:Number(value)})});
    Object.assign(CFG,result);previewPlan=null;CFG.maps=(await api('/api/config')).maps;delete images.map;delete images.sat;loadImage('map');loadImage('sat');draw();toast('Дороги загружены и сохранены на Pi');
  }catch(e){toast(e.message,true);}finally{$('btnImportRoads').disabled=false;roadStatus();}
};
$('btnPreview').onclick=async()=>{
  if(roadEditor || localPoints){toast('Сначала сохраните дороги и точки',true);return;}
  previewPlan=null;
  try{
    const a=CFG.points.find(p=>p.id===$('selFrom').value),b=CFG.points.find(p=>p.id===$('selTo').value);
    const route=async(start,goal)=>api('/api/route',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({start,goal})});
    const trip=await route([a.x,a.y],[b.x,b.y]);let path=trip.path,length=trip.distance_m;
    if(poseKnown()){const p=LAST.telemetry.pose;const pickup=await route([p.x,p.y],[a.x,a.y]);path=[...pickup.path,...trip.path.slice(1)];length+=pickup.distance_m;}
    previewPlan=path;toast(`Маршрут по дорогам: ${length.toFixed(0)} м`);
  }catch(e){toast(e.message,true);}draw();
};
$('selFrom').onchange=$('selTo').onchange=()=>{previewPlan=null;draw();};

let panDrag=null;
cv.addEventListener('pointerdown',e=>{
  if(e.shiftKey || e.button===1 || (!roadEditor && !placing && !settingPose)){
    panDrag={x:e.clientX,y:e.clientY};cv.setPointerCapture(e.pointerId);e.preventDefault();
  }
});
cv.addEventListener('pointermove',e=>{
  if(!panDrag)return;
  const dx=e.clientX-panDrag.x,dy=e.clientY-panDrag.y;
  if(roadView){roadView.ox+=dx;roadView.oy+=dy;}else{const v=view();mapPan.x+=dx/v.s;mapPan.y+=dy/v.s;}
  panDrag={x:e.clientX,y:e.clientY};draw();
});
cv.addEventListener('pointerup',()=>{panDrag=null;});
cv.addEventListener('pointercancel',()=>{panDrag=null;});
