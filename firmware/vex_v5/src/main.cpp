// АТСД-1М · прошивка VEX V5 Brain · протокол описан в README.md
// Моторы: 1 FL, 10 ML, 9 RL, 19 FR, 8 MR, 20 RR. Гироскоп 276-2333 — порт A, бамперы — B, C.
#include "vex.h"
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>

using namespace vex;

namespace atsd {

#define FW_VERSION "2.1.2"

#define ATSD_USE_STDIO 0  // 1, если в SDK нет vexSerialReadChar

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Подстроить под робот: колесо, колея, знак гироскопа, скорости
namespace cfg {

const gearSetting GEARSET        = ratio18_1;
const double MOTOR_MAX_RPM       = 200.0;
const double WHEEL_DIAM_MM       = 101.6;
const double WHEEL_PER_MOTOR_REV = 1.0;
const double TRACK_EFF_MM        = 430.0;

const double GYRO_SIGN  = -1.0;
const double GYRO_SCALE = 1.0;
const double GYRO_FAULT_ENC_W  = 0.6;
const double GYRO_FAULT_GYRO_W = 0.05;
const uint32_t GYRO_FAULT_MS   = 800;
const bool   GYRO_AUTO_SIGN = true;
const double GYRO_SIGN_ENC_W  = 0.2;
const double GYRO_SIGN_GYRO_W = 0.15;
const double GYRO_SIGN_EVIDENCE_S = 0.3;
const double GYRO_SPIKE_W     = 4.0;
const double GYRO_SPIKE_STRAIGHT_W = 1.2;

const double V_CRUISE   = 550.0;
const double V_MAX_EXT  = 900.0;
const double V_CREEP    = 70.0;
const double V_CORNER   = 220.0;
const double V_BACKUP   = 180.0;
const double V_MANUAL   = 600.0;
const double W_MANUAL   = 2.0;
const double ACCEL      = 450.0;
const double DECEL      = 800.0;
const double DECEL_PLAN = 450.0;
const double W_MAX      = 1.6;
const double W_ACCEL    = 5.0;

const double KP_HOLD        = 2.0;
const double HOLD_ERR_DEG   = 12.0;
const double HOLD_W_MAX     = 0.8;
const double V_NUDGE_MAX    = 400.0;
const double W_NUDGE_MAX    = 1.0;
const uint32_t NUDGE_TIMEOUT_MS = 400;
const double KP_PATH        = 2.2;
const double KP_TURN        = 2.5;
const double W_TURN_MAX     = 1.4;
const double W_TURN_MIN     = 0.35;
const double TURN_IN_PLACE_DEG  = 50.0;
const double TURN_EXIT_DEG      = 12.0;
const double LOOKAHEAD_MM   = 450.0;
const double WP_PASS_MM     = 120.0;
const double GOAL_TOL_MM    = 90.0;
const double NEAR_GOAL_MM   = 250.0;

const bool     DOCK_ENABLE       = true;
const bool     DOCK_RESET_POSE   = true;
const double   DOCK_SEARCH_MM    = 2500.0;
const double   DOCK_MAX_JUMP_MM  = 1500.0;
const double   DOCK_TOL_MM       = 50.0;
const double   V_DOCK            = 180.0;
const uint32_t DOCK_MSG_TIMEOUT_MS = 300;
const uint32_t DOCK_WAIT_MS      = 3000;
const uint32_t ZONE_TIMEOUT_MS   = 4000;
const double CORNER_ANGLE_DEG = 30.0;

const double   STOP_SIGN_OFFSET_MM  = 350.0;
const uint32_t STOP_HOLD_MS         = 3000;
const double   STOP_IGNORE_MM       = 2500.0;
const double   CROSS_ZONE_MM        = 2000.0;
const double   CROSS_FACTOR         = 0.5;
const double   BUMP_ZONE_MM         = 1800.0;
const double   BUMP_FACTOR          = 0.35;
const double   LIGHT_STOP_OFFSET_MM = 450.0;

const double   OBST_SLOW_MM        = 1800.0;
const double   OBST_STOP_MM        = 900.0;
const double   OBST_STOP_DETOUR_MM = 450.0;
const uint32_t OBST_MSG_TIMEOUT_MS = 400;
const uint32_t OBST_WAIT_MS        = 2000;
const double   DETOUR_OFFSET_MM    = 650.0;
const double   DETOUR_ENTRY_MM     = 600.0;
const double   DETOUR_EXIT_MM      = 700.0;
const double   OBST_DEPTH_MM       = 500.0;
const double   ROBOT_HALF_LEN_MM   = 350.0;
const char     DEFAULT_DETOUR_SIDE = 'L';

const uint32_t LINK_TIMEOUT_MS   = 500;
const uint32_t V_WATCHDOG_MS     = 300;
const double   STALL_CMD_RPM     = 25.0;
const double   STALL_MEAS_RPM    = 6.0;
const double   STALL_CURRENT_A   = 1.6;
const uint32_t STALL_TIME_MS     = 700;
const double   REC_BACK_MM       = 250.0;
const uint32_t REC_TIMEOUT_MS    = 3000;
const int      REC_MAX           = 3;
const uint32_t NO_PROGRESS_MS    = 12000;
const double   TEMP_HOT_C        = 55.0;
const double   TEMP_CRIT_C       = 65.0;
const int      BATT_LOW_PCT      = 20;
const bool     ENABLE_BUMPERS    = true;
const bool     ENABLE_HEADING_HOLD = true;
}

// Карта трассы в мм, старт (0,0) лицом по +x. Узлы 0..3 — точки заказа
enum NodeId { N_KPP = 0, N_ADMIN, N_TEST, N_SKLAD, N_START, N_C1, N_C2, N_C3, N_COUNT };
const int N_POINTS = 4;

struct Node { const char* name; double x, y; double parkDeg; };

Node NODES[N_COUNT] = {
  {"KPP",   2000.0,     0.0, NAN},
  {"ADMIN", 18000.0,    0.0, NAN},
  {"TEST",  10000.0, 6000.0, NAN},
  {"SKLAD",     0.0, 3000.0, NAN},
  {"START",     0.0,    0.0, 0.0},
  {"C1",    20000.0,    0.0, NAN},
  {"C2",    20000.0, 6000.0, NAN},
  {"C3",        0.0, 6000.0, NAN},
};

struct Edge { int a, b; };
const Edge EDGES[] = {
  {N_START, N_KPP}, {N_KPP, N_ADMIN}, {N_ADMIN, N_C1}, {N_C1, N_C2},
  {N_C2, N_TEST},   {N_TEST, N_C3},   {N_C3, N_SKLAD}, {N_SKLAD, N_START},
};
const int EDGE_COUNT = sizeof(EDGES) / sizeof(EDGES[0]);

brain      Brain;
controller Ctl(controllerType::primary);

// Если мотор крутится назад — поменять true/false
motor mFL(PORT1,  cfg::GEARSET, false);
motor mML(PORT10, cfg::GEARSET, false);
motor mRL(PORT9,  cfg::GEARSET, false);
motor mFR(PORT19, cfg::GEARSET, true);
motor mMR(PORT8,  cfg::GEARSET, true);
motor mRR(PORT20, cfg::GEARSET, true);

gyro   Gyro(Brain.ThreeWirePort.A);
bumper BumpL(Brain.ThreeWirePort.B);
bumper BumpR(Brain.ThreeWirePort.C);

struct MotorSlot { motor* m; bool left; const char* name; };
MotorSlot MOT[6] = {
  {&mFL, true, "FL 1"}, {&mML, true, "ML 10"}, {&mRL, true, "RL 9"},
  {&mFR, false, "FR 19"}, {&mMR, false, "MR 8"}, {&mRR, false, "RR 20"},
};

static inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline double wrapPi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
static inline double d2r(double d) { return d * M_PI / 180.0; }
static inline double r2d(double r) { return r * 180.0 / M_PI; }
static inline uint32_t nowMs() { return (uint32_t)timer::system(); }

static const double MM_PER_MOTOR_DEG = M_PI * cfg::WHEEL_DIAM_MM * cfg::WHEEL_PER_MOTOR_REV / 360.0;
static const double RPM_PER_MM_S     = 60.0 / (M_PI * cfg::WHEEL_DIAM_MM * cfg::WHEEL_PER_MOTOR_REV);

struct Pose { double x = 0, y = 0, yaw = 0; } pose;
double tripMm = 0;
double vMeas = 0, wMeas = 0;
double encDegL = 0, encDegR = 0;
double encMmL = 0, encMmR = 0;

double vOut = 0, wOut = 0;
double rpmCmd[6] = {0};

bool   estop = false;
bool   gyroCal = false, gyroResync = true, gyroFault = false;
double gyroPrevDeg = 0, gyroSuspectMs = 0;
double gyroSignFix = 1.0, gyroSignEvidence = 0, gyroAccum = 0;
uint32_t gyroSpikes = 0;

struct Stats { uint32_t rxLines = 0, rxBad = 0, rxOverflow = 0, txDrop = 0, vIgnored = 0; } stats;

struct Link { uint32_t lastRx = 0; bool ok = false; bool everSeen = false; } piLink;
struct VelCmd { double v = 0, w = 0; uint32_t last = 0; bool ever = false; } velCmd;
double holdYaw = 0; bool holdActive = false;
int extLimitPct = 100;

struct Health {
  bool   inst[6]; double temp[6], cur[6], rpm[6];
  bool   missReported[6];
  double maxTemp = 0; int missing = 0; bool sideDead = false;
  int    hotLevel = 0;
  int    battPct = 100; int battMv = 0; bool battLowReported = false;
  uint32_t stallSince = 0; bool stall = false; bool stallPrev = false;
  bool   bump = false, bumpPrev = false;
} hl;

enum MState { MS_IDLE, MS_TO_PICKUP, MS_WAIT_LOAD, MS_TO_DROP, MS_WAIT_UNLOAD, MS_DONE, MS_RETURN, MS_FAULT };
const char* MS_NAME[] = {"IDLE", "TO_PICKUP", "WAIT_LOAD", "TO_DROP", "WAIT_UNLOAD", "DONE", "RETURN", "FAULT"};
const char* MS_HEAD[] = {"READY", "TO PICKUP", "LOADING", "TO DEST", "UNLOADING", "DELIVERED", "RETURNING", "FAULT"};

struct Mission {
  MState st = MS_IDLE; MState faultFrom = MS_IDLE;
  int from = -1, to = -1, goal = -1;
  uint32_t t0 = 0, t1 = 0;
  bool needsPi = true; bool returnAfter = false;
  bool parking = false; int recCount = 0;
  char faultMsg[24] = "";
} ms;

enum { P_ESTOP = 1, P_LINK = 2, P_STOPSIGN = 4, P_RED = 8, P_OBST = 16, P_EXT = 32, P_REC = 64 };
int pauseMask = 0;

struct Wp { double x, y; bool detour; };
const int MAX_WP = 48;
Wp wps[MAX_WP]; int wpCount = 0, wpIdx = 0;
double segSX = 0, segSY = 0;
bool followPivot = false;

struct StopSign { bool pending = false, holding = false; double stopAt = 0; uint32_t holdUntil = 0; double ignoreUntil = -1; } stopSign;
double zoneCrossEnd = -1, zoneBumpEnd = -1;
struct Light { bool red = false; double stopAt = 0; char last = 'N'; } tLight;
struct Obst { int dist = 0; char side = '?'; int depth = 0; int offset = 0; uint32_t last = 0; } obst;
char lastFreeSide = 0;
struct ObstWait { bool active = false; uint32_t t0 = 0; uint32_t lastReport = 0; } obstWait;
struct Recovery { bool active = false; double startTrip = 0, yaw = 0; uint32_t t0 = 0; } rec;
struct Progress { double best = 1e18; uint32_t t = 0; int idx = -1; } prog;
struct Dock { bool search = false, have = false, done = false; double tx = 0, ty = 0; uint32_t seen = 0, waitStart = 0; } dock;
struct Zone { int node = -1; int rssi = -127; bool near = false; uint32_t last = 0; } zone;
struct Nudge { double v = 0, w = 0; uint32_t last = 0; bool active = false; } nudge;

enum { T_NONE = 0, T_MOTOR, T_DRIVE, T_TURN };
struct Test {
  int kind = T_NONE; int motorIdx = -1; uint32_t until = 0, t0 = 0;
  double startTrip = 0, startYaw = 0, targetYaw = 0, startL = 0, startR = 0;
  char result[64] = "";
} test;

enum Page { PG_MAIN = 0, PG_MISSION, PG_DIAG, PG_TEST };
int page = PG_MAIN;
int selFrom = N_SKLAD, selTo = N_ADMIN;
char evLog[4][44]; int evIdx = 0;

#if ATSD_USE_STDIO
static int serialReadChar() {
  int c = getchar();
  if (c == EOF) { clearerr(stdin); return -1; }
  return c;
}
static void serialWrite(const char* s, int n) { fwrite(s, 1, n, stdout); fflush(stdout); }
#else
static int serialReadChar() { return vexSerialReadChar(1); }
static void serialWrite(const char* s, int n) {
  if (vexSerialWriteFree(1) >= n) vexSerialWriteBuffer(1, (uint8_t*)s, n);
  else stats.txDrop++;
}
#endif

static void sendf(const char* fmt, ...) {
  char buf[160];
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n >= (int)sizeof buf) n = sizeof(buf) - 1;
  serialWrite(buf, n);
}

static void logEvent(const char* fmt, ...) {
  char b[96];
  va_list ap; va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  sendf("K %s\n", b);
  uint32_t s = nowMs() / 1000;
  snprintf(evLog[evIdx], sizeof evLog[0], "%02lu:%02lu %s", (unsigned long)(s / 60), (unsigned long)(s % 60), b);
  evIdx = (evIdx + 1) % 4;
}

struct LineMsg { char s[96]; bool local; };
const int QSIZE = 24;
LineMsg lineQ[QSIZE]; int qHead = 0, qTail = 0;

static void qPush(const char* s, bool local) {
  int n = (qHead + 1) % QSIZE;
  if (n == qTail) { stats.rxOverflow++; return; }
  strncpy(lineQ[qHead].s, s, sizeof(lineQ[0].s) - 1);
  lineQ[qHead].s[sizeof(lineQ[0].s) - 1] = 0;
  lineQ[qHead].local = local;
  qHead = n;
}
static bool qPop(LineMsg& out) {
  if (qTail == qHead) return false;
  out = lineQ[qTail];
  qTail = (qTail + 1) % QSIZE;
  return true;
}
static void localCmd(const char* fmt, ...) {
  char b[96];
  va_list ap; va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  qPush(b, true);
}

static int rxThread() {
  char buf[96]; int len = 0;
  while (true) {
    int got = 0;
    while (got < 256) {
      int c = serialReadChar();
      if (c < 0) break;
      got++;
      if (c == '\n' || c == '\r') {
        if (len > 0) { buf[len] = 0; qPush(buf, false); len = 0; }
      } else if (len < (int)sizeof(buf) - 1) {
        buf[len++] = (char)c;
      } else {
        len = 0; stats.rxBad++;
      }
    }
    this_thread::sleep_for(2);
  }
  return 0;
}

static double nodeDist(int a, int b) { return hypot(NODES[a].x - NODES[b].x, NODES[a].y - NODES[b].y); }

static int nearestNode(double x, double y) {
  int best = 0; double bd = 1e18;
  for (int i = 0; i < N_COUNT; i++) {
    double d = hypot(NODES[i].x - x, NODES[i].y - y);
    if (d < bd) { bd = d; best = i; }
  }
  return best;
}

static int dijkstra(int s, int g, int* out) {
  double dist[N_COUNT]; int prev[N_COUNT]; bool done[N_COUNT];
  for (int i = 0; i < N_COUNT; i++) { dist[i] = 1e18; prev[i] = -1; done[i] = false; }
  dist[s] = 0;
  for (int it = 0; it < N_COUNT; it++) {
    int u = -1;
    for (int i = 0; i < N_COUNT; i++)
      if (!done[i] && (u < 0 || dist[i] < dist[u])) u = i;
    if (u < 0 || dist[u] >= 1e17) break;
    done[u] = true;
    if (u == g) break;
    for (int e = 0; e < EDGE_COUNT; e++) {
      int v = -1;
      if (EDGES[e].a == u) v = EDGES[e].b;
      else if (EDGES[e].b == u) v = EDGES[e].a;
      if (v < 0) continue;
      double c = dist[u] + nodeDist(u, v);
      if (c < dist[v]) { dist[v] = c; prev[v] = u; }
    }
  }
  if (dist[g] >= 1e17) return 0;
  int tmp[N_COUNT]; int n = 0;
  for (int v = g; v != -1 && n < N_COUNT; v = prev[v]) tmp[n++] = v;
  for (int i = 0; i < n; i++) out[i] = tmp[n - 1 - i];
  return n;
}

static void addWp(double x, double y, bool detour) {
  if (wpCount >= MAX_WP) return;
  wps[wpCount].x = x; wps[wpCount].y = y; wps[wpCount].detour = detour;
  wpCount++;
}

static void resetTravelBehaviours() {
  stopSign = StopSign();
  tLight = Light();
  obstWait = ObstWait();
  rec = Recovery();
  prog = Progress();
  followPivot = false;
  dock = Dock();
}

static bool planRoute(int goal) {
  int start = nearestNode(pose.x, pose.y);
  int seq[N_COUNT];
  int len = dijkstra(start, goal, seq);
  if (len <= 0) return false;
  wpCount = 0; wpIdx = 0;
  segSX = pose.x; segSY = pose.y;
  for (int k = 0; k < len; k++) {
    const Node& nd = NODES[seq[k]];
    if (k == 0 && len > 1 && hypot(nd.x - pose.x, nd.y - pose.y) < cfg::WP_PASS_MM * 1.5) continue;
    addWp(nd.x, nd.y, false);
  }
  ms.goal = goal;
  ms.parking = false;
  dock = Dock();
  nudge = Nudge();
  prog = Progress();
  followPivot = false;
  return true;
}

static double remainingRouteMm() {
  if (wpIdx >= wpCount) return 0;
  double r = hypot(wps[wpIdx].x - pose.x, wps[wpIdx].y - pose.y);
  for (int i = wpIdx + 1; i < wpCount; i++) r += hypot(wps[i].x - wps[i - 1].x, wps[i].y - wps[i - 1].y);
  return r;
}

double prevPos[6]; bool prevOk[6] = {false, false, false, false, false, false};

static double gyroYawDeg() { return cfg::GYRO_SIGN * cfg::GYRO_SCALE * Gyro.rotation(rotationUnits::deg); }

static void startGyroCalibration() {
  gyroCal = true; gyroFault = false; gyroSuspectMs = 0;
  Gyro.calibrate();
  logEvent("GYRO_CALIBRATING");
}

static void odomUpdate(double dt) {
  double dSum[2] = {0, 0}; int dN[2] = {0, 0};
  for (int i = 0; i < 6; i++) {
    bool ok = MOT[i].m->installed();
    double p = ok ? MOT[i].m->position(rotationUnits::deg) : 0;
    if (ok && prevOk[i]) {
      int s = MOT[i].left ? 0 : 1;
      dSum[s] += p - prevPos[i]; dN[s]++;
    }
    prevPos[i] = p; prevOk[i] = ok;
  }
  double degL = dN[0] ? dSum[0] / dN[0] : 0;
  double degR = dN[1] ? dSum[1] / dN[1] : 0;
  encDegL += degL; encDegR += degR;
  double dL = degL * MM_PER_MOTOR_DEG, dR = degR * MM_PER_MOTOR_DEG;
  encMmL += dL; encMmR += dR;

  double ds = 0.5 * (dL + dR);
  double dyawE = (dR - dL) / cfg::TRACK_EFF_MM;

  double dyawG = 0; bool gValid = false;
  if (gyroCal && !Gyro.isCalibrating()) {
    gyroCal = false; gyroResync = true;
    logEvent("GYRO_READY");
  }
  if (!gyroCal) {
    double g = gyroYawDeg();
    if (gyroResync) { gyroPrevDeg = g; gyroResync = false; }
    else { dyawG = gyroSignFix * d2r(g - gyroPrevDeg); gyroPrevDeg = g; gValid = true; }
  }

  if (gValid && dt > 0) {
    double wE = dyawE / dt, wG = dyawG / dt;
    if (cfg::GYRO_AUTO_SIGN && fabs(wE) > cfg::GYRO_SIGN_ENC_W && fabs(wG) > cfg::GYRO_SIGN_GYRO_W) {
      gyroSignEvidence += (wE * wG < 0 ? -dt : dt);
      if (gyroSignEvidence > 2.0) gyroSignEvidence = 2.0;
      if (gyroSignEvidence < -cfg::GYRO_SIGN_EVIDENCE_S) {
        gyroSignFix = -gyroSignFix;
        gyroSignEvidence = 0;
        dyawG = -dyawG;
        pose.yaw -= 2.0 * gyroAccum;
        gyroAccum = -gyroAccum;
        holdYaw = pose.yaw;
        logEvent("GYRO_SIGN_FLIPPED set GYRO_SIGN %+.0f", cfg::GYRO_SIGN * gyroSignFix);
      }
    }
    if (fabs(wG) > cfg::GYRO_SPIKE_W || (fabs(wE) < 0.15 && fabs(wG) > cfg::GYRO_SPIKE_STRAIGHT_W)) {
      dyawG = dyawE;
      gyroSpikes++;
    }
  }

  if (gValid && !gyroFault && dt > 0) {
    double wE = dyawE / dt, wG = dyawG / dt;
    if (fabs(wE) > cfg::GYRO_FAULT_ENC_W && fabs(wG) < cfg::GYRO_FAULT_GYRO_W) {
      gyroSuspectMs += dt * 1000.0;
      if (gyroSuspectMs > cfg::GYRO_FAULT_MS) {
        gyroFault = true;
        logEvent("GYRO_FAULT ENCODER_YAW");
      }
    } else if (fabs(wG) > 0.15) {
      gyroSuspectMs = 0;
    }
  }

  bool useGyro = gValid && !gyroFault;
  double dyaw = useGyro ? dyawG : dyawE;
  if (useGyro) gyroAccum += dyaw;
  double mid = pose.yaw + 0.5 * dyaw;
  pose.x += ds * cos(mid);
  pose.y += ds * sin(mid);
  pose.yaw += dyaw;
  tripMm += fabs(ds);

  if (dt > 0) {
    vMeas += 0.3 * (ds / dt - vMeas);
    wMeas += 0.3 * (dyaw / dt - wMeas);
  }
}

static void healthUpdate() {
  uint32_t now = nowMs();
  hl.maxTemp = 0;
  int miss = 0, missL = 0, missR = 0;
  for (int i = 0; i < 6; i++) {
    bool inst = MOT[i].m->installed();
    hl.inst[i] = inst;
    if (!inst) {
      miss++;
      if (MOT[i].left) missL++; else missR++;
      hl.temp[i] = hl.cur[i] = hl.rpm[i] = 0;
      if (!hl.missReported[i]) { logEvent("MOTOR_MISSING %s", MOT[i].name); hl.missReported[i] = true; }
      continue;
    }
    if (hl.missReported[i]) { logEvent("MOTOR_BACK %s", MOT[i].name); hl.missReported[i] = false; }
    hl.temp[i] = MOT[i].m->temperature(temperatureUnits::celsius);
    hl.cur[i]  = MOT[i].m->current(currentUnits::amp);
    hl.rpm[i]  = MOT[i].m->velocity(velocityUnits::rpm);
    if (hl.temp[i] > hl.maxTemp) hl.maxTemp = hl.temp[i];
  }
  hl.missing = miss;
  hl.sideDead = (missL == 3 || missR == 3);

  int lvl = hl.maxTemp >= cfg::TEMP_CRIT_C ? 2 : (hl.maxTemp >= cfg::TEMP_HOT_C ? 1 : 0);
  if (lvl > hl.hotLevel) logEvent("MOTOR_HOT %.0fC", hl.maxTemp);
  if (lvl == 0 && hl.hotLevel > 0 && hl.maxTemp < cfg::TEMP_HOT_C - 5) logEvent("MOTOR_COOL");
  if (lvl > 0 || hl.maxTemp < cfg::TEMP_HOT_C - 5) hl.hotLevel = lvl;

  hl.battPct = (int)Brain.Battery.capacity();
  hl.battMv  = (int)Brain.Battery.voltage(voltageUnits::mV);
  if (hl.battPct < cfg::BATT_LOW_PCT && !hl.battLowReported) { logEvent("BATT_LOW %d", hl.battPct); hl.battLowReported = true; }
  if (hl.battPct > cfg::BATT_LOW_PCT + 5) hl.battLowReported = false;

  bool stallNow = false;
  for (int s = 0; s < 2; s++) {
    double c = 0, m = 0, a = 0; int n = 0;
    for (int i = 0; i < 6; i++) {
      if (MOT[i].left != (s == 0) || !hl.inst[i]) continue;
      c += fabs(rpmCmd[i]); m += fabs(hl.rpm[i]); a += hl.cur[i]; n++;
    }
    if (n == 0) continue;
    c /= n; m /= n; a /= n;
    if (c > cfg::STALL_CMD_RPM && m < cfg::STALL_MEAS_RPM && a > cfg::STALL_CURRENT_A) stallNow = true;
  }
  if (stallNow) { if (hl.stallSince == 0) hl.stallSince = now ? now : 1; }
  else hl.stallSince = 0;
  hl.stall = hl.stallSince != 0 && (now - hl.stallSince) > cfg::STALL_TIME_MS;
  if (hl.stall && !hl.stallPrev) logEvent("STALL");
  hl.stallPrev = hl.stall;
}

static void bumperUpdate() {
  hl.bump = cfg::ENABLE_BUMPERS && (BumpL.pressing() || BumpR.pressing());
  if (hl.bump && !hl.bumpPrev) logEvent("BUMPER");
  hl.bumpPrev = hl.bump;
}

static double speedFactor(bool mission) {
  double f = extLimitPct / 100.0;
  if (mission) {
    if (tripMm < zoneCrossEnd) f = fmin(f, cfg::CROSS_FACTOR);
    if (tripMm < zoneBumpEnd)  f = fmin(f, cfg::BUMP_FACTOR);
  }
  if (hl.battPct < cfg::BATT_LOW_PCT) f = fmin(f, 0.8);
  if (hl.hotLevel == 2) f = fmin(f, 0.35);
  else if (hl.hotLevel == 1) f = fmin(f, 0.6);
  if (hl.missing > 0) f = fmin(f, 0.7);
  return clampd(f, 0.0, 1.0);
}

static void stopAll() {
  for (int i = 0; i < 6; i++) { MOT[i].m->stop(brakeType::brake); rpmCmd[i] = 0; }
}

static void applyDrive(double vT, double wT, double dt) {
  bool speedingUp = fabs(vT) > fabs(vOut) && vT * vOut >= 0;
  double a = speedingUp ? cfg::ACCEL : cfg::DECEL;
  vOut += clampd(vT - vOut, -a * dt, a * dt);
  wOut += clampd(wT - wOut, -cfg::W_ACCEL * dt, cfg::W_ACCEL * dt);

  bool idle = fabs(vT) < 1.0 && fabs(wT) < 0.01 && fabs(vOut) < 4.0 && fabs(wOut) < 0.03;
  if (idle) { vOut = 0; wOut = 0; stopAll(); return; }

  double vl = vOut - wOut * cfg::TRACK_EFF_MM * 0.5;
  double vr = vOut + wOut * cfg::TRACK_EFF_MM * 0.5;
  double rl = vl * RPM_PER_MM_S, rr = vr * RPM_PER_MM_S;
  double mx = fmax(fabs(rl), fabs(rr));
  if (mx > cfg::MOTOR_MAX_RPM) { rl *= cfg::MOTOR_MAX_RPM / mx; rr *= cfg::MOTOR_MAX_RPM / mx; }
  for (int i = 0; i < 6; i++) {
    double r = MOT[i].left ? rl : rr;
    rpmCmd[i] = r;
    MOT[i].m->spin(directionType::fwd, r, velocityUnits::rpm);
  }
}

static double turnRate(double e) {
  double w = clampd(cfg::KP_TURN * e, -cfg::W_TURN_MAX, cfg::W_TURN_MAX);
  if (fabs(e) > d2r(1.5) && fabs(w) < cfg::W_TURN_MIN) w = e > 0 ? cfg::W_TURN_MIN : -cfg::W_TURN_MIN;
  return w;
}

static bool isTravel(MState s) { return s == MS_TO_PICKUP || s == MS_TO_DROP || s == MS_RETURN; }
static bool isBusy(MState s) { return isTravel(s) || s == MS_WAIT_LOAD || s == MS_WAIT_UNLOAD; }

static void sendStatus();

static void setState(MState s) {
  ms.st = s;
  logEvent("STATE %s", MS_NAME[s]);
  sendStatus();
}

static void missionFault(const char* why) {
  if (isTravel(ms.st)) ms.faultFrom = ms.st;
  strncpy(ms.faultMsg, why, sizeof(ms.faultMsg) - 1);
  ms.faultMsg[sizeof(ms.faultMsg) - 1] = 0;
  rec.active = false;
  setState(MS_FAULT);
  logEvent("FAULT %s", why);
}

static void startMission(int a, int b) {
  if (estop) { logEvent("REJECT ESTOP"); return; }
  if (gyroCal) { logEvent("REJECT GYRO_CAL"); return; }
  if (a < 0 || a >= N_POINTS || b < 0 || b >= N_POINTS || a == b) { logEvent("REJECT BAD_POINTS"); return; }
  if (isBusy(ms.st)) { logEvent("REJECT BUSY"); return; }
  test.kind = T_NONE;
  ms.from = a; ms.to = b;
  ms.t0 = nowMs(); ms.t1 = 0;
  ms.recCount = 0;
  zoneCrossEnd = zoneBumpEnd = -1;
  resetTravelBehaviours();
  logEvent("MISSION_START %s %s", NODES[a].name, NODES[b].name);
  setState(MS_TO_PICKUP);
  if (!planRoute(a)) missionFault("NO_ROUTE");
}

static bool zoneFresh() { return zone.node >= 0 && (nowMs() - zone.last) < cfg::ZONE_TIMEOUT_MS; }

static void checkZone(int node) {
  if (node < 0 || node >= N_POINTS) return;
  if (zoneFresh() && zone.near && zone.node == node) logEvent("ZONE_OK %s %d", NODES[node].name, zone.rssi);
  else if (zoneFresh() && zone.near) logEvent("ZONE_MISMATCH %s %s", NODES[node].name, NODES[zone.node].name);
  else logEvent("ZONE_UNCONFIRMED %s", NODES[node].name);
}

static void arrivedAtGoal() {
  ms.recCount = 0;
  checkZone(ms.goal);
  stopSign.pending = stopSign.holding = false;
  obstWait.active = false;
  switch (ms.st) {
    case MS_TO_PICKUP:
      logEvent("ARRIVED_PICKUP %s", NODES[ms.from].name);
      setState(MS_WAIT_LOAD);
      break;
    case MS_TO_DROP:
      logEvent("ARRIVED_DROP %s", NODES[ms.to].name);
      setState(MS_WAIT_UNLOAD);
      break;
    case MS_RETURN:
      logEvent("RETURNED_BASE");
      setState(MS_IDLE);
      break;
    default: break;
  }
}

static void confirmCargo() {
  if (ms.st == MS_WAIT_LOAD) {
    logEvent("CARGO_LOADED");
    resetTravelBehaviours();
    setState(MS_TO_DROP);
    if (!planRoute(ms.to)) missionFault("NO_ROUTE");
  } else if (ms.st == MS_WAIT_UNLOAD) {
    ms.t1 = nowMs();
    uint32_t d = ms.t1 - ms.t0;
    logEvent("CARGO_UNLOADED");
    logEvent("DELIVERED %lu.%lu s", (unsigned long)(d / 1000), (unsigned long)((d % 1000) / 100));
    setState(MS_DONE);
    if (ms.returnAfter) {
      resetTravelBehaviours();
      setState(MS_RETURN);
      if (!planRoute(N_START)) missionFault("NO_ROUTE");
    }
  } else {
    logEvent("IGNORED_C %s", MS_NAME[ms.st]);
  }
}

static void resumeMission() {
  if (ms.st != MS_FAULT || !isTravel(ms.faultFrom)) { logEvent("IGNORED_U"); return; }
  ms.recCount = 0;
  resetTravelBehaviours();
  setState(ms.faultFrom);
  if (!planRoute(ms.goal)) missionFault("NO_ROUTE");
}

static void segGeom(double& ux, double& uy, double& L, double& along) {
  const Wp& t = wps[wpIdx];
  double dx = t.x - segSX, dy = t.y - segSY;
  L = hypot(dx, dy);
  if (L < 1.0) { ux = cos(pose.yaw); uy = sin(pose.yaw); L = 0; }
  else { ux = dx / L; uy = dy / L; }
  along = (pose.x - segSX) * ux + (pose.y - segSY) * uy;
}

static void startDetour(char side, double obstDist, double depth, double offset) {
  if (wpIdx >= wpCount) return;
  double ux, uy, L, along;
  segGeom(ux, uy, L, along);
  double nx = -uy, ny = ux;
  if (side == 'R') { nx = -nx; ny = -ny; }
  double off = offset > 0 ? offset : cfg::DETOUR_OFFSET_MM;
  double dep = depth > 0 ? depth : cfg::OBST_DEPTH_MM;
  double base = along > 0 ? along : 0;
  double a1 = base + cfg::DETOUR_ENTRY_MM;
  double a2 = base + obstDist + dep + cfg::ROBOT_HALF_LEN_MM;
  double a3 = a2 + cfg::DETOUR_EXIT_MM;

  Wp p[3]; int k = 0;
  p[k].x = segSX + ux * a1 + nx * off; p[k].y = segSY + uy * a1 + ny * off; p[k].detour = true; k++;
  p[k].x = segSX + ux * a2 + nx * off; p[k].y = segSY + uy * a2 + ny * off; p[k].detour = true; k++;
  if (L > 0 && a3 < L - cfg::WP_PASS_MM) {
    p[k].x = segSX + ux * a3; p[k].y = segSY + uy * a3; p[k].detour = true; k++;
  }
  if (wpCount + k > MAX_WP) { logEvent("DETOUR_NO_SPACE"); return; }
  for (int i = wpCount - 1; i >= wpIdx; i--) wps[i + k] = wps[i];
  for (int i = 0; i < k; i++) wps[wpIdx + i] = p[i];
  wpCount += k;
  segSX = pose.x; segSY = pose.y;
  prog = Progress();
  followPivot = false;
  logEvent("DETOUR_START %c", side);
}

static void startRecovery(const char* why) {
  if (rec.active) return;
  ms.recCount++;
  if (ms.recCount > cfg::REC_MAX) { missionFault("STUCK"); return; }
  rec.active = true;
  rec.startTrip = tripMm;
  rec.yaw = pose.yaw;
  rec.t0 = nowMs();
  hl.stallSince = 0; hl.stall = false;
  followPivot = false;
  logEvent("RECOVERY %d %s", ms.recCount, why);
}

static bool obstPresent() { return obst.dist > 0 && (nowMs() - obst.last) < cfg::OBST_MSG_TIMEOUT_MS; }

static bool followPath(double vMax, double& v, double& w) {
  for (int guard = 0; guard < 4; guard++) {
    if (wpIdx >= wpCount) { v = 0; w = 0; return true; }
    double ux, uy, L, along;
    segGeom(ux, uy, L, along);
    const Wp& t = wps[wpIdx];
    double distT = hypot(t.x - pose.x, t.y - pose.y);
    double remain = L - along;
    bool last = (wpIdx == wpCount - 1);

    double goalTol = dock.have ? cfg::DOCK_TOL_MM : cfg::GOAL_TOL_MM;
    bool reached = last ? (distT < goalTol)
                        : (distT < cfg::WP_PASS_MM || (L > 0 && remain < cfg::WP_PASS_MM));
    if (last && !reached && distT < cfg::NEAR_GOAL_MM) {
      double eg = wrapPi(atan2(t.y - pose.y, t.x - pose.x) - pose.yaw);
      if (fabs(eg) > d2r(100.0)) reached = true;
      else {
        followPivot = false;
        double c = cos(eg);
        v = fmin(vMax, 120.0) * (c > 0 ? c : 0);
        w = clampd(cfg::KP_PATH * eg, -0.8, 0.8);
        return false;
      }
    }
    if (reached) {
      bool wasDetour = t.detour;
      segSX = t.x; segSY = t.y;
      wpIdx++;
      followPivot = false;
      if (wasDetour && (wpIdx >= wpCount || !wps[wpIdx].detour)) logEvent("DETOUR_DONE");
      if (wpIdx >= wpCount) { v = 0; w = 0; return true; }
      continue;
    }

    double la = (along > 0 ? along : 0) + cfg::LOOKAHEAD_MM;
    double lx, ly;
    if (L <= 0 || la >= L) { lx = t.x; ly = t.y; }
    else { lx = segSX + ux * la; ly = segSY + uy * la; }
    double e = wrapPi(atan2(ly - pose.y, lx - pose.x) - pose.yaw);

    double vLim = vMax;
    if (last) {
      double d = distT - goalTol * 0.5;
      vLim = fmin(vLim, sqrt(2.0 * cfg::DECEL_PLAN * (d > 0 ? d : 0)) + 40.0);
      if (dock.search && distT < 1000.0) vLim = fmin(vLim, cfg::V_DOCK);
    } else {
      const Wp& n2 = wps[wpIdx + 1];
      double h1 = atan2(t.y - segSY, t.x - segSX);
      double h2 = atan2(n2.y - t.y, n2.x - t.x);
      if (fabs(wrapPi(h2 - h1)) > d2r(cfg::CORNER_ANGLE_DEG)) {
        double d = remain - cfg::WP_PASS_MM;
        vLim = fmin(vLim, cfg::V_CORNER + sqrt(2.0 * cfg::DECEL_PLAN * (d > 0 ? d : 0)));
      }
    }

    double ae = fabs(e);
    if (!followPivot && ae > d2r(cfg::TURN_IN_PLACE_DEG)) followPivot = true;
    if (followPivot && ae < d2r(cfg::TURN_EXIT_DEG)) followPivot = false;

    if (followPivot) {
      v = 0; w = turnRate(e);
    } else {
      double c = cos(e);
      v = vLim * (c > 0 ? c * c : 0);
      w = clampd(cfg::KP_PATH * e, -cfg::W_MAX, cfg::W_MAX);
    }
    return false;
  }
  v = 0; w = 0;
  return false;
}

static void travelTick(double& v, double& w) {
  uint32_t now = nowMs();
  v = 0; w = 0;
  pauseMask = 0;

  if (hl.sideDead) { missionFault("SIDE_MOTORS_DEAD"); return; }

  if (ms.parking) {
    double e = wrapPi(d2r(NODES[ms.goal].parkDeg) - pose.yaw);
    if (fabs(e) < d2r(3.0) && fabs(wMeas) < 0.15) { ms.parking = false; arrivedAtGoal(); return; }
    w = turnRate(e);
    return;
  }

  if (rec.active) {
    pauseMask |= P_REC;
    double back = tripMm - rec.startTrip;
    bool timeout = (now - rec.t0) > cfg::REC_TIMEOUT_MS;
    if (hl.stall && (now - rec.t0) > 800) { missionFault("STUCK_BACK"); return; }
    if (back >= cfg::REC_BACK_MM || timeout) {
      rec.active = false;
      if (timeout && back < 50) { missionFault("STUCK_BACK"); return; }
      char side = cfg::DEFAULT_DETOUR_SIDE;
      if (obstPresent() && (obst.side == 'L' || obst.side == 'R')) side = obst.side;
      else if (lastFreeSide) side = lastFreeSide;
      if (wpIdx < wpCount && !wps[wpIdx].detour)
        startDetour(side, cfg::REC_BACK_MM + 250.0, cfg::OBST_DEPTH_MM, 0);
      return;
    }
    v = -cfg::V_BACKUP;
    w = clampd(cfg::KP_HOLD * wrapPi(rec.yaw - pose.yaw), -cfg::W_MAX, cfg::W_MAX);
    return;
  }

  if (hl.stall || hl.bump) { startRecovery(hl.bump ? "BUMPER" : "STALL"); return; }

  double vLim = cfg::V_CRUISE * speedFactor(true);
  bool inDetour = wpIdx < wpCount && wps[wpIdx].detour;

  if (ms.needsPi && !piLink.ok) pauseMask |= P_LINK;
  if (extLimitPct == 0) pauseMask |= P_EXT;

  if (stopSign.holding) {
    pauseMask |= P_STOPSIGN;
    if (now >= stopSign.holdUntil) {
      stopSign.holding = false; stopSign.pending = false;
      stopSign.ignoreUntil = tripMm + cfg::STOP_IGNORE_MM;
      logEvent("STOP_SIGN_GO");
    }
  } else if (stopSign.pending) {
    double rem = stopSign.stopAt - tripMm;
    if (rem <= 30.0) {
      pauseMask |= P_STOPSIGN;
      if (fabs(vMeas) < 25.0) {
        stopSign.holding = true;
        stopSign.holdUntil = now + cfg::STOP_HOLD_MS;
        logEvent("STOP_SIGN_HOLD");
      }
    } else {
      vLim = fmin(vLim, fmax(cfg::V_CREEP, sqrt(2.0 * cfg::DECEL_PLAN * rem)));
    }
  }

  if (tLight.red) {
    double rem = tLight.stopAt - tripMm;
    if (rem <= 30.0) pauseMask |= P_RED;
    else vLim = fmin(vLim, fmax(cfg::V_CREEP, sqrt(2.0 * cfg::DECEL_PLAN * rem)));
  }

  if (obstPresent()) {
    double stopD = inDetour ? cfg::OBST_STOP_DETOUR_MM : cfg::OBST_STOP_MM;
    if (obst.dist <= stopD + 50) {
      pauseMask |= P_OBST;
      if (!obstWait.active) {
        obstWait.active = true; obstWait.t0 = now; obstWait.lastReport = now;
        logEvent("OBSTACLE_STOP %d", obst.dist);
      } else if (now - obstWait.t0 > cfg::OBST_WAIT_MS && fabs(vMeas) < 20.0 && !inDetour) {
        char side = obst.side;
        if (side == '?') side = lastFreeSide ? lastFreeSide : cfg::DEFAULT_DETOUR_SIDE;
        if (side == 'L' || side == 'R') {
          obstWait.active = false;
          startDetour(side, obst.dist, obst.depth, obst.offset);
        } else if (now - obstWait.lastReport > 5000) {
          obstWait.lastReport = now;
          logEvent("BLOCKED_WAITING");
        }
      }
    } else {
      if (obst.dist < cfg::OBST_SLOW_MM) {
        double rem = obst.dist - stopD;
        vLim = fmin(vLim, fmax(cfg::V_CREEP, sqrt(2.0 * cfg::DECEL_PLAN * (rem > 0 ? rem : 0))));
      }
    }
  } else if (obstWait.active) {
    obstWait.active = false;
    logEvent("OBSTACLE_CLEAR");
  }

  if (cfg::DOCK_ENABLE && !dock.search && !dock.done && ms.goal >= 0 && ms.goal < N_POINTS && wpCount > 0) {
    const Node& g = NODES[ms.goal];
    double dg = hypot(g.x - pose.x, g.y - pose.y);
    bool bleNear = zoneFresh() && zone.near && zone.node == ms.goal;
    if (dg < cfg::DOCK_SEARCH_MM || (bleNear && dg < 2.0 * cfg::DOCK_SEARCH_MM)) {
      dock.search = true;
      logEvent("DOCK_SEARCH %s%s", g.name, bleNear ? " BLE" : "");
    }
  }

  double vF, wF;
  bool arrived = followPath(vLim, vF, wF);
  if (arrived) {
    if (cfg::DOCK_ENABLE && dock.search && !dock.done && ms.goal < N_POINTS) {
      if (dock.have) {
        const Node& g = NODES[ms.goal];
        double drift = hypot(g.x - pose.x, g.y - pose.y);
        if (cfg::DOCK_RESET_POSE) { pose.x = g.x; pose.y = g.y; }
        dock.done = true;
        logEvent("DOCKED %s %d", g.name, (int)drift);
      } else {
        if (dock.waitStart == 0) { dock.waitStart = now; logEvent("DOCK_WAIT"); }
        if (now - dock.waitStart < cfg::DOCK_WAIT_MS) return;
        dock.done = true;
        logEvent("DOCK_NOT_FOUND %s", NODES[ms.goal].name);
      }
    }
    if (ms.goal >= 0 && !std::isnan(NODES[ms.goal].parkDeg)) { ms.parking = true; return; }
    arrivedAtGoal();
    return;
  }

  if (pauseMask) { prog.t = now; return; }

  if (wpIdx < wpCount) {
    double d = hypot(wps[wpIdx].x - pose.x, wps[wpIdx].y - pose.y);
    if (prog.idx != wpIdx || d < prog.best - 150.0) { prog.best = d; prog.t = now; prog.idx = wpIdx; }
    else if (now - prog.t > cfg::NO_PROGRESS_MS) { prog.t = now; startRecovery("NO_PROGRESS"); return; }
  }

  v = vF; w = wF;
}

static void velTick(double dt, double& v, double& w) {
  double vc = clampd(velCmd.v, -cfg::V_MAX_EXT, cfg::V_MAX_EXT) * speedFactor(false);
  double wc = clampd(velCmd.w, -cfg::W_MAX, cfg::W_MAX);

  if (vc > 0 && ((obstPresent() && obst.dist < cfg::OBST_STOP_DETOUR_MM) || hl.bump)) vc = 0;

  if (!cfg::ENABLE_HEADING_HOLD || (fabs(vc) < 1.0 && fabs(wc) < 0.005)) {
    holdActive = false;
    v = vc; w = wc;
    return;
  }
  if (!holdActive || fabs(wc) > 0.02) { holdYaw = pose.yaw; holdActive = true; }
  v = vc;
  if (fabs(wc) > 0.02) { w = wc; return; }
  double e = holdYaw - pose.yaw;
  const double lim = d2r(cfg::HOLD_ERR_DEG);
  if (e > lim) { holdYaw = pose.yaw + lim; e = lim; }
  if (e < -lim) { holdYaw = pose.yaw - lim; e = -lim; }
  w = clampd(clampd(cfg::KP_HOLD * e, -cfg::HOLD_W_MAX, cfg::HOLD_W_MAX), -cfg::W_MAX, cfg::W_MAX);
}

static bool testTick(double& v, double& w) {
  uint32_t now = nowMs();
  v = 0; w = 0;
  switch (test.kind) {
    case T_MOTOR:
      if (now >= test.until) {
        test.kind = T_NONE;
        snprintf(test.result, sizeof test.result, "%s: wheel must roll robot FORWARD", MOT[test.motorIdx].name);
        return false;
      }
      return true;
    case T_DRIVE: {
      double d = tripMm - test.startTrip;
      if (d >= 1000.0 || now - test.t0 > 10000) {
        test.kind = T_NONE;
        snprintf(test.result, sizeof test.result, "enc %.0fmm yaw %+.1fdeg. Tape=? ->WHEEL_DIAM",
                 d, r2d(pose.yaw - test.startYaw));
        return false;
      }
      double rem = 1000.0 - d;
      v = fmin(300.0, 50.0 + sqrt(2.0 * cfg::DECEL_PLAN * rem));
      w = clampd(cfg::KP_HOLD * wrapPi(test.startYaw - pose.yaw), -cfg::W_MAX, cfg::W_MAX);
      return false;
    }
    case T_TURN: {
      double e = wrapPi(test.targetYaw - pose.yaw);
      if ((fabs(e) < d2r(1.5) && fabs(wMeas) < 0.1) || now - test.t0 > 8000) {
        test.kind = T_NONE;
        double dyaw = pose.yaw - test.startYaw;
        double dl = encMmL - test.startL, dr = encMmR - test.startR;
        double track = fabs(dyaw) > 0.1 ? (dr - dl) / dyaw : 0;
        snprintf(test.result, sizeof test.result, "gyro %+.1fdeg  TRACK_EFF est %.0fmm", r2d(dyaw), track);
        return false;
      }
      w = turnRate(e);
      return false;
    }
    default: return false;
  }
}

static void startTest(int kind, int idx) {
  if (isBusy(ms.st) || estop || gyroCal) { snprintf(test.result, sizeof test.result, "busy: stop mission/estop"); return; }
  test.kind = kind; test.motorIdx = idx; test.t0 = nowMs();
  test.until = test.t0 + 1500;
  test.startTrip = tripMm; test.startYaw = pose.yaw;
  test.targetYaw = pose.yaw + d2r(90.0);
  test.startL = encMmL; test.startR = encMmR;
  snprintf(test.result, sizeof test.result, "running...");
}

static void handleLine(char* line, bool local) {
  char* tok[8]; int n = 0;
  for (char* p = strtok(line, " \t"); p && n < 8; p = strtok(nullptr, " \t")) tok[n++] = p;
  if (n == 0) return;
  const char* c = tok[0];
  bool ok = true;
  uint32_t now = nowMs();

  if (!strcmp(c, "V") && n >= 3) {
    if (isBusy(ms.st)) stats.vIgnored++;
    else {
      velCmd.v = atof(tok[1]); velCmd.w = atof(tok[2]) / 1000.0;
      velCmd.last = now; velCmd.ever = true;
    }
  } else if (!strcmp(c, "H")) {
  } else if (!strcmp(c, "M") && n >= 3) {
    startMission(atoi(tok[1]), atoi(tok[2]));
  } else if (!strcmp(c, "C")) {
    confirmCargo();
  } else if (!strcmp(c, "A")) {
    if (ms.st != MS_IDLE) { wpCount = 0; wpIdx = 0; logEvent("MISSION_ABORT"); setState(MS_IDLE); }
  } else if (!strcmp(c, "U")) {
    resumeMission();
  } else if (!strcmp(c, "X")) {
    if (!estop) { estop = true; logEvent("ESTOP"); }
  } else if (!strcmp(c, "R")) {
    if (estop) { estop = false; logEvent("ESTOP_RELEASE"); }
  } else if (!strcmp(c, "L") && n >= 2) {
    extLimitPct = (int)clampd(atoi(tok[1]), 0, 100);
  } else if (!strcmp(c, "P") && n >= 3) {
    pose.x = atof(tok[1]); pose.y = atof(tok[2]);
    if (n >= 4) {
      double delta = wrapPi(d2r(atof(tok[3]) / 1000.0) - pose.yaw);
      pose.yaw += delta; holdYaw += delta; gyroAccum = 0;
    }
  } else if (!strcmp(c, "Z")) {
    int nd = n >= 2 ? atoi(tok[1]) : N_START;
    if (isTravel(ms.st) || nd < 0 || nd >= N_COUNT) { ok = false; }
    else {
      pose.x = NODES[nd].x; pose.y = NODES[nd].y;
      double hd = std::isnan(NODES[nd].parkDeg) ? 0.0 : d2r(NODES[nd].parkDeg);
      double delta = wrapPi(hd - pose.yaw);
      pose.yaw += delta; holdYaw += delta; gyroAccum = 0;
      logEvent("POSE_RESET %s", NODES[nd].name);
    }
  } else if (!strcmp(c, "F") && n >= 2) {
    double dist = n >= 3 ? atof(tok[2]) : 0;
    if (dist < 0) dist = 0;
    if (!strcmp(tok[1], "STOP")) {
      if (isTravel(ms.st) && tripMm >= stopSign.ignoreUntil && !stopSign.pending && !stopSign.holding) {
        double rem = dist - cfg::STOP_SIGN_OFFSET_MM;
        stopSign.pending = true;
        stopSign.stopAt = tripMm + (rem > 0 ? rem : 0);
        logEvent("STOP_SIGN %d", (int)dist);
      }
    } else if (!strcmp(tok[1], "CROSS")) {
      if (isTravel(ms.st)) {
        if (tripMm >= zoneCrossEnd) logEvent("CROSSWALK_SLOW");
        zoneCrossEnd = fmax(zoneCrossEnd, tripMm + dist + cfg::CROSS_ZONE_MM);
      }
    } else if (!strcmp(tok[1], "BUMP")) {
      if (isTravel(ms.st)) {
        if (tripMm >= zoneBumpEnd) logEvent("BUMP_SLOW");
        zoneBumpEnd = fmax(zoneBumpEnd, tripMm + dist + cfg::BUMP_ZONE_MM);
      }
    } else ok = false;
  } else if (!strcmp(c, "T") && n >= 2) {
    char s = (char)toupper(tok[1][0]);
    double dist = n >= 3 ? atof(tok[2]) : 0;
    if (s == 'R' || s == 'Y') {
      if (isTravel(ms.st)) {
        double rem = dist > 0 ? dist - cfg::LIGHT_STOP_OFFSET_MM : 0;
        double need = vMeas * vMeas / (2.0 * cfg::DECEL);
        bool cannotStop = (s == 'Y' && dist > 0 && rem < need);
        if (!cannotStop) {
          double at = tripMm + (rem > 0 ? rem : 0);
          if (!tLight.red) { tLight.red = true; tLight.stopAt = at; logEvent("RED_LIGHT_STOP %d", (int)dist); }
          else if (dist > 0) tLight.stopAt = fmin(tLight.stopAt, at);
        }
      }
    } else if (s == 'G') {
      if (tLight.red) { tLight.red = false; logEvent("GREEN_GO"); }
    } else if (s != 'N') ok = false;
    if (ok) tLight.last = s;
  } else if (!strcmp(c, "D") && n >= 3) {
    if (isTravel(ms.st) && dock.search && !dock.done && ms.goal >= 0 && ms.goal < N_POINTS && wpCount > 0) {
      double f = atof(tok[1]), l = atof(tok[2]);
      double wx = pose.x + f * cos(pose.yaw) - l * sin(pose.yaw);
      double wy = pose.y + f * sin(pose.yaw) + l * cos(pose.yaw);
      const Node& g = NODES[ms.goal];
      if (hypot(wx - g.x, wy - g.y) <= cfg::DOCK_MAX_JUMP_MM) {
        if (!dock.have) { dock.tx = wx; dock.ty = wy; logEvent("DOCK_SPOT %d %d", (int)f, (int)l); }
        else { dock.tx += 0.5 * (wx - dock.tx); dock.ty += 0.5 * (wy - dock.ty); }
        dock.have = true;
        dock.seen = now;
        if (wpIdx >= wpCount) { wpIdx = wpCount - 1; segSX = pose.x; segSY = pose.y; followPivot = false; }
        if (!wps[wpCount - 1].detour) { wps[wpCount - 1].x = dock.tx; wps[wpCount - 1].y = dock.ty; }
      }
    }
  } else if (!strcmp(c, "N") && n >= 3) {
    if (isTravel(ms.st) && !estop) {
      nudge.v = clampd(atof(tok[1]), -cfg::V_NUDGE_MAX, cfg::V_NUDGE_MAX);
      nudge.w = clampd(atof(tok[2]) / 1000.0, -cfg::W_NUDGE_MAX, cfg::W_NUDGE_MAX);
      nudge.last = now;
      if (!nudge.active) { nudge.active = true; logEvent("NUDGE_ON"); }
    }
  } else if (!strcmp(c, "B") && n >= 3) {
    int nd = atoi(tok[1]);
    zone.node = (nd >= 0 && nd < N_POINTS) ? nd : -1;
    zone.rssi = atoi(tok[2]);
    zone.near = n >= 4 && atoi(tok[3]) != 0;
    zone.last = now;
  } else if (!strcmp(c, "O") && n >= 2) {
    obst.dist = atoi(tok[1]);
    obst.side = n >= 3 ? (char)toupper(tok[2][0]) : '?';
    obst.depth = n >= 4 ? atoi(tok[3]) : 0;
    obst.offset = n >= 5 ? atoi(tok[4]) : 0;
    obst.last = now;
    if (obst.side == 'L' || obst.side == 'R') lastFreeSide = obst.side;
  } else if (!strcmp(c, "?")) {
    sendf("I ATSD-1M %s\n", FW_VERSION);
    sendStatus();
  } else if (local && c[0] == '!') {
    if (!strcmp(c, "!PI")) ms.needsPi = !ms.needsPi;
    else if (!strcmp(c, "!RET")) ms.returnAfter = !ms.returnAfter;
    else if (!strcmp(c, "!TM") && n >= 2) { int i = atoi(tok[1]); if (i >= 0 && i < 6) startTest(T_MOTOR, i); }
    else if (!strcmp(c, "!TD")) startTest(T_DRIVE, -1);
    else if (!strcmp(c, "!TT")) startTest(T_TURN, -1);
    else if (!strcmp(c, "!TS")) test.kind = T_NONE;
    else if (!strcmp(c, "!GC")) {
      if (!isBusy(ms.st) && fabs(vMeas) < 5 && !gyroCal) startGyroCalibration();
    } else ok = false;
  } else {
    ok = false;
  }

  if (ok) {
    stats.rxLines++;
    if (!local && !strcmp(c, "H")) { piLink.lastRx = now; piLink.everSeen = true; }
  } else {
    stats.rxBad++;
  }
}

static uint32_t statusFlags() {
  uint32_t f = 0;
  if (velCmd.ever && !isBusy(ms.st) && nowMs() - velCmd.last > cfg::V_WATCHDOG_MS) f |= 1u << 0;
  if (estop)          f |= 1u << 1;
  if (gyroCal)        f |= 1u << 2;
  if (gyroFault)      f |= 1u << 3;
  if (hl.hotLevel)    f |= 1u << 4;
  if (hl.missing)     f |= 1u << 5;
  if (hl.stall)       f |= 1u << 6;
  if (hl.bump)        f |= 1u << 7;
  if (hl.battPct < cfg::BATT_LOW_PCT) f |= 1u << 8;
  if (isBusy(ms.st))  f |= 1u << 9;
  if (pauseMask)      f |= 1u << 10;
  if (wpIdx < wpCount && wps[wpIdx].detour) f |= 1u << 11;
  if (rec.active)     f |= 1u << 12;
  if (ms.st == MS_FAULT) f |= 1u << 13;
  if (Ctl.ButtonL1.pressing() && !isTravel(ms.st)) f |= 1u << 14;
  if (!piLink.ok)       f |= 1u << 15;
  if (isTravel(ms.st) && dock.search && !dock.done) f |= 1u << 16;
  if (zoneFresh() && zone.near) f |= 1u << 17;
  if (nudge.active) f |= 1u << 18;
  return f;
}

static uint32_t missionDs() {
  if (ms.t0 == 0) return 0;
  uint32_t end = ms.t1 ? ms.t1 : (isBusy(ms.st) || ms.st == MS_FAULT ? nowMs() : ms.t0);
  return (end - ms.t0) / 100;
}

static void sendE() {
  sendf("E %ld %ld %ld %lu %d %lu\n", (long)lround(encDegL), (long)lround(encDegR),
        (long)lround(r2d(pose.yaw) * 1000.0), (unsigned long)nowMs(), hl.battMv, (unsigned long)statusFlags());
}
static void sendP() {
  sendf("P %ld %ld %ld %ld %ld\n", (long)lround(pose.x), (long)lround(pose.y),
        (long)lround(r2d(pose.yaw) * 1000.0), (long)lround(vMeas), (long)lround(wMeas * 1000.0));
}
static void sendStatus() {
  sendf("S %s %d %d %d/%d %ld %X %lu\n", MS_NAME[ms.st], ms.from, ms.to, wpIdx, wpCount,
        (long)lround(remainingRouteMm()), (unsigned)pauseMask, (unsigned long)missionDs());
}

static const color C_BG     = color(0x101418);
static const color C_PANEL  = color(0x263238);
static const color C_GREEN  = color(0x2E7D32);
static const color C_YELLOW = color(0xF9A825);
static const color C_BLUE   = color(0x1565C0);
static const color C_RED    = color(0xC62828);
static const color C_CYAN   = color(0x00838F);
static const color C_GRAY   = color(0x455A64);
static const color C_ORANGE = color(0xEF6C00);

static int charW(fontType f) {
  switch (f) {
    case fontType::mono15: return 9;
    case fontType::mono20: return 11;
    case fontType::mono30: return 17;
    case fontType::mono40: return 22;
    default: return 11;
  }
}
static int charH(fontType f) {
  switch (f) {
    case fontType::mono15: return 12;
    case fontType::mono20: return 15;
    case fontType::mono30: return 22;
    case fontType::mono40: return 30;
    default: return 15;
  }
}

static void uiRect(int x, int y, int w, int h, const color& c) {
  Brain.Screen.setPenColor(c); Brain.Screen.setFillColor(c);
  Brain.Screen.drawRectangle(x, y, w, h);
}
static void uiText(int x, int y, const color& bg, const color& fg, fontType f, const char* s) {
  Brain.Screen.setFont(f); Brain.Screen.setPenColor(fg); Brain.Screen.setFillColor(bg);
  Brain.Screen.printAt(x, y, true, "%s", s);
}
static void uiButton(int x, int y, int w, int h, const color& c, const char* label, fontType f = fontType::mono20) {
  uiRect(x, y, w, h, c);
  Brain.Screen.setPenColor(color::white); Brain.Screen.setFillColor(color::transparent);
  Brain.Screen.drawRectangle(x, y, w, h);
  int tw = (int)strlen(label) * charW(f);
  uiText(x + (w - tw) / 2, y + (h + charH(f)) / 2, c, color::white, f, label);
}
static bool inR(int px, int py, int x, int y, int w, int h) { return px >= x && px < x + w && py >= y && py < y + h; }

static const char* pauseText() {
  if (estop) return "EMERGENCY STOP";
  if (gyroCal) return "GYRO CALIBRATING - DON'T MOVE";
  if (ms.st == MS_FAULT) return ms.faultMsg;
  if (ms.st == MS_WAIT_LOAD) return "Put cargo, lock, press CARGO OK";
  if (ms.st == MS_WAIT_UNLOAD) return "Take cargo, press CARGO OK";
  if (pauseMask & P_REC) return "RECOVERY: backing up";
  if (pauseMask & P_LINK) return "NO LINK TO RASPBERRY PI";
  if (pauseMask & P_RED) return "RED LIGHT - WAITING";
  if (pauseMask & P_STOPSIGN) return "STOP SIGN - FULL STOP";
  if (pauseMask & P_OBST) return "OBSTACLE AHEAD";
  if (pauseMask & P_EXT) return "SPEED LIMIT 0 (PI)";
  if (isTravel(ms.st)) {
    if (nudge.active) return "OPERATOR NUDGE";
    if (dock.search && !dock.done) return dock.have ? "DOCKING ON RED MARK" : "LOOKING FOR RED MARK";
    if (wpIdx < wpCount && wps[wpIdx].detour) return "DETOUR AROUND OBSTACLE";
    if (tripMm < zoneBumpEnd) return "SPEED BUMP - SLOW";
    if (tripMm < zoneCrossEnd) return "CROSSWALK - SLOW";
    return "DRIVING";
  }
  if (ms.st == MS_DONE) return "Delivery complete";
  return "Select route on MISSION tab";
}

static void drawTabs() {
  const char* names[4] = {"MAIN", "MISSION", "DIAG", "TEST"};
  for (int i = 0; i < 4; i++) uiButton(i * 120, 212, 120, 28, i == page ? C_BLUE : C_PANEL, names[i], fontType::mono15);
}

static void drawMain() {
  color pc = C_GRAY;
  if (estop || ms.st == MS_FAULT) pc = C_RED;
  else if (isTravel(ms.st)) pc = pauseMask ? C_YELLOW : C_GREEN;
  else if (ms.st == MS_WAIT_LOAD || ms.st == MS_WAIT_UNLOAD) pc = C_BLUE;
  else if (ms.st == MS_DONE) pc = C_CYAN;
  uiRect(0, 0, 340, 108, pc);
  color fg = (pc == C_YELLOW) ? color::black : color::white;
  uiText(10, 45, pc, fg, fontType::mono40, estop ? "E-STOP" : MS_HEAD[ms.st]);
  uiText(10, 90, pc, fg, fontType::mono15, pauseText());

  uiButton(345, 0, 135, 108, estop ? C_ORANGE : C_RED, estop ? "RELEASE" : "STOP", fontType::mono30);

  char b[48];
  uint32_t ds = missionDs();
  snprintf(b, sizeof b, "%02lu:%02lu.%lu", (unsigned long)(ds / 600), (unsigned long)((ds / 10) % 60), (unsigned long)(ds % 10));
  uiText(8, 150, C_BG, color::white, fontType::mono40, b);

  if (ms.from >= 0) snprintf(b, sizeof b, "%s > %s", NODES[ms.from].name, NODES[ms.to].name);
  else snprintf(b, sizeof b, "no order");
  uiText(250, 128, C_BG, color::white, fontType::mono20, b);
  if (zoneFresh()) snprintf(b, sizeof b, "BAT %d%% BLE %s %d", hl.battPct, NODES[zone.node].name, zone.rssi);
  else snprintf(b, sizeof b, "BAT %d%%  PI %s", hl.battPct, piLink.ok ? "OK" : "--");
  uiText(250, 148, C_BG, hl.battPct < cfg::BATT_LOW_PCT ? C_YELLOW : color::white, fontType::mono15, b);
  snprintf(b, sizeof b, "%.2fm/s  left %.1fm", vMeas / 1000.0, remainingRouteMm() / 1000.0);
  uiText(250, 164, C_BG, color::white, fontType::mono15, b);

  if (ms.st == MS_WAIT_LOAD || ms.st == MS_WAIT_UNLOAD) uiButton(0, 168, 340, 40, C_GREEN, "CARGO OK / GO", fontType::mono30);
  else if (ms.st == MS_FAULT && isTravel(ms.faultFrom)) uiButton(0, 168, 340, 40, C_ORANGE, "RESUME", fontType::mono30);
  else uiText(8, 195, C_BG, C_GRAY, fontType::mono15, evLog[(evIdx + 3) % 4]);
}

static void drawMission() {
  char b[40];
  uiText(4, 34, C_BG, color::white, fontType::mono20, "FROM");
  uiText(4, 89, C_BG, color::white, fontType::mono20, "TO");
  for (int i = 0; i < N_POINTS; i++) {
    uiButton(62 + i * 104, 5, 100, 48, i == selFrom ? C_BLUE : C_PANEL, NODES[i].name);
    uiButton(62 + i * 104, 60, 100, 48, i == selTo ? C_GREEN : C_PANEL, NODES[i].name);
  }
  uiButton(0, 114, 155, 45, isBusy(ms.st) ? C_GRAY : C_GREEN, "START", fontType::mono30);
  uiButton(160, 114, 155, 45, C_RED, "ABORT", fontType::mono30);
  snprintf(b, sizeof b, "PI %s", ms.needsPi ? "REQUIRED" : "OPTIONAL");
  uiButton(320, 114, 160, 45, ms.needsPi ? C_PANEL : C_ORANGE, b, fontType::mono15);
  uiButton(0, 164, 235, 44, C_PANEL, "POSE = START", fontType::mono20);
  snprintf(b, sizeof b, "RETURN: %s", ms.returnAfter ? "YES" : "NO");
  uiButton(245, 164, 235, 44, C_PANEL, b, fontType::mono20);
}

static void drawDiag() {
  char b[80]; int y = 14;
  Brain.Screen.setFont(fontType::mono15);
  uiText(4, y, C_BG, C_CYAN, fontType::mono15, "MOTOR   T,C  I,A   RPM   CMD"); y += 14;
  for (int i = 0; i < 6; i++) {
    if (hl.inst[i]) snprintf(b, sizeof b, "%-6s %4.0f %5.2f %5.0f %5.0f", MOT[i].name, hl.temp[i], hl.cur[i], hl.rpm[i], rpmCmd[i]);
    else snprintf(b, sizeof b, "%-6s  --- NOT CONNECTED ---", MOT[i].name);
    uiText(4, y, C_BG, hl.inst[i] ? (hl.temp[i] >= cfg::TEMP_HOT_C ? C_YELLOW : color::white) : C_RED, fontType::mono15, b);
    y += 14;
  }
  snprintf(b, sizeof b, "YAW %+8.1f gyro %s w %+.2f sp %lu", r2d(pose.yaw), gyroCal ? "CAL" : (gyroFault ? "FAULT>ENC" : (gyroSignFix < 0 ? "SIGN FLIP" : "OK")), wMeas, (unsigned long)gyroSpikes);
  uiText(4, y, C_BG, gyroFault ? C_RED : color::white, fontType::mono15, b); y += 14;
  snprintf(b, sizeof b, "X %.0f Y %.0f  trip %.1fm  v %.0f", pose.x, pose.y, tripMm / 1000.0, vMeas);
  uiText(4, y, C_BG, color::white, fontType::mono15, b); y += 14;
  snprintf(b, sizeof b, "BAT %d%% %dmV  bump %d stall %d", hl.battPct, hl.battMv, hl.bump ? 1 : 0, hl.stall ? 1 : 0);
  uiText(4, y, C_BG, color::white, fontType::mono15, b); y += 14;
  snprintf(b, sizeof b, "PI %s rx %lu bad %lu drop %lu vIgn %lu", piLink.ok ? "OK" : "--", (unsigned long)stats.rxLines,
           (unsigned long)stats.rxBad, (unsigned long)stats.txDrop, (unsigned long)stats.vIgnored);
  uiText(4, y, C_BG, color::white, fontType::mono15, b); y += 14;
  snprintf(b, sizeof b, "FLAGS %04lX  wp %d/%d  light %c  obst %d%c",
           (unsigned long)statusFlags(), wpIdx, wpCount, tLight.last, obstPresent() ? obst.dist : 0, obst.side);
  uiText(4, y, C_BG, color::white, fontType::mono15, b); y += 14;
  for (int k = 0; k < 3; k++) {
    uiText(4, y, C_BG, C_GRAY, fontType::mono15, evLog[(evIdx + 1 + k) % 4]);
    y += 14;
  }
}

static void drawTest() {
  for (int i = 0; i < 6; i++)
    uiButton((i % 3) * 160, 4 + (i / 3) * 50, 155, 45, test.kind == T_MOTOR && test.motorIdx == i ? C_ORANGE : C_PANEL, MOT[i].name);
  uiButton(0, 106, 116, 45, C_PANEL, "GYRO CAL", fontType::mono15);
  uiButton(121, 106, 116, 45, test.kind == T_DRIVE ? C_ORANGE : C_PANEL, "DRIVE 1M", fontType::mono15);
  uiButton(242, 106, 116, 45, test.kind == T_TURN ? C_ORANGE : C_PANEL, "TURN 90", fontType::mono15);
  uiButton(363, 106, 116, 45, C_RED, "STOP", fontType::mono15);
  uiText(4, 172, C_BG, C_YELLOW, fontType::mono15, test.result);
  char b[64];
  snprintf(b, sizeof b, "yaw %+.1f  encL %.0f encR %.0f mm", r2d(pose.yaw), encMmL, encMmR);
  uiText(4, 192, C_BG, color::white, fontType::mono15, b);
}

static void uiTouch(int x, int y) {
  if (y >= 212) { page = x / 120; if (page > 3) page = 3; return; }
  switch (page) {
    case PG_MAIN:
      if (inR(x, y, 345, 0, 135, 108)) localCmd(estop ? "R" : "X");
      else if (inR(x, y, 0, 168, 340, 40)) {
        if (ms.st == MS_WAIT_LOAD || ms.st == MS_WAIT_UNLOAD) localCmd("C");
        else if (ms.st == MS_FAULT) localCmd("U");
      }
      break;
    case PG_MISSION:
      for (int i = 0; i < N_POINTS; i++) {
        if (inR(x, y, 62 + i * 104, 5, 100, 48)) selFrom = i;
        if (inR(x, y, 62 + i * 104, 60, 100, 48)) selTo = i;
      }
      if (inR(x, y, 0, 114, 155, 45)) { localCmd("M %d %d", selFrom, selTo); page = PG_MAIN; }
      if (inR(x, y, 160, 114, 155, 45)) localCmd("A");
      if (inR(x, y, 320, 114, 160, 45)) localCmd("!PI");
      if (inR(x, y, 0, 164, 235, 44)) localCmd("Z");
      if (inR(x, y, 245, 164, 235, 44)) localCmd("!RET");
      break;
    case PG_TEST:
      for (int i = 0; i < 6; i++) if (inR(x, y, (i % 3) * 160, 4 + (i / 3) * 50, 155, 45)) localCmd("!TM %d", i);
      if (inR(x, y, 0, 106, 116, 45)) localCmd("!GC");
      if (inR(x, y, 121, 106, 116, 45)) localCmd("!TD");
      if (inR(x, y, 242, 106, 116, 45)) localCmd("!TT");
      if (inR(x, y, 363, 106, 116, 45)) localCmd("!TS");
      break;
    default: break;
  }
}

static void ctlRow(int row) {
  char b[24];
  switch (row) {
    case 0: snprintf(b, sizeof b, "%-11s B%3d%%", estop ? "E-STOP" : MS_HEAD[ms.st], hl.battPct); break;
    case 1: snprintf(b, sizeof b, "%-19.19s", pauseText()); break;
    default: {
      uint32_t ds = missionDs();
      snprintf(b, sizeof b, "T %02lu:%02lu  PI:%s", (unsigned long)(ds / 600), (unsigned long)((ds / 10) % 60), piLink.ok ? "OK" : "--");
    }
  }
  Ctl.Screen.setCursor(row + 1, 1);
  Ctl.Screen.print("%-19s", b);
}

static int uiThread() {
  bool prevTouch = false, prevA = false, prevB = false;
  uint32_t lastDraw = 0, lastCtl = 0; int row = 0;
  while (true) {
    bool t = Brain.Screen.pressing();
    if (t && !prevTouch) uiTouch(Brain.Screen.xPosition(), Brain.Screen.yPosition());
    prevTouch = t;

    if (Ctl.installed()) {
      bool a = Ctl.ButtonA.pressing(), b = Ctl.ButtonB.pressing();
      if (a && !prevA) localCmd("C");
      if (b && !prevB) localCmd(estop ? "R" : "X");
      prevA = a; prevB = b;
    }

    uint32_t now = nowMs();
    if (now - lastDraw >= 100) {
      lastDraw = now;
      Brain.Screen.clearScreen(C_BG);
      switch (page) {
        case PG_MAIN: drawMain(); break;
        case PG_MISSION: drawMission(); break;
        case PG_DIAG: drawDiag(); break;
        case PG_TEST: drawTest(); break;
      }
      drawTabs();
      Brain.Screen.render();
    }
    if (Ctl.installed() && now - lastCtl >= 150) {
      lastCtl = now;
      ctlRow(row);
      row = (row + 1) % 3;
    }
    this_thread::sleep_for(20);
  }
  return 0;
}

int run() {
  for (int i = 0; i < 6; i++) {
    MOT[i].m->setStopping(brakeType::brake);
    MOT[i].m->resetPosition();
    hl.inst[i] = false; hl.missReported[i] = false;
  }
  for (int i = 0; i < 4; i++) evLog[i][0] = 0;

  Brain.Screen.clearScreen(C_BG);
  uiText(20, 60, C_BG, color::white, fontType::mono30, "ATSD-1M  fw " FW_VERSION);
  uiText(20, 120, C_BG, C_YELLOW, fontType::mono30, "GYRO CALIBRATION");
  uiText(20, 160, C_BG, C_YELLOW, fontType::mono20, "do not move the robot");
  this_thread::sleep_for(500);
  startGyroCalibration();

  thread rx(rxThread);
  thread ui(uiThread);

  sendf("I ATSD-1M %s\n", FW_VERSION);

  uint32_t last = nowMs(), next = last;
  uint32_t tick = 0;
  bool linkPrev = false;

  while (true) {
    uint32_t now = nowMs();
    double dt = (now - last) / 1000.0;
    if (dt <= 0) dt = 0.001;
    if (dt > 0.05) dt = 0.05;
    last = now;

    LineMsg m;
    for (int k = 0; k < 16 && qPop(m); k++) handleLine(m.s, m.local);

    odomUpdate(dt);
    if (tick % 10 == 0) healthUpdate();
    bumperUpdate();
    piLink.ok = piLink.everSeen && (now - piLink.lastRx) < cfg::LINK_TIMEOUT_MS;
    if (piLink.ok != linkPrev) { logEvent(piLink.ok ? "LINK_OK" : "LINK_LOST"); linkPrev = piLink.ok; }

    double v = 0, w = 0;
    bool motorTest = false;
    bool manual = Ctl.installed() && Ctl.ButtonL1.pressing() && !isTravel(ms.st);

    if (estop || gyroCal) {
      pauseMask = estop ? P_ESTOP : 0;
      if (isTravel(ms.st)) prog.t = now;
    } else if (manual) {
      v = Ctl.Axis3.position() / 100.0 * cfg::V_MANUAL;
      w = -Ctl.Axis1.position() / 100.0 * cfg::W_MANUAL;
      if (fabs(v) < 30) v = 0;
      if (fabs(w) < 0.1) w = 0;
    } else if (test.kind != T_NONE) {
      motorTest = testTick(v, w);
    } else if (isTravel(ms.st)) {
      travelTick(v, w);
      bool nf = nudge.active && (now - nudge.last) < cfg::NUDGE_TIMEOUT_MS;
      if (nudge.active && !nf) { nudge.active = false; nudge.v = nudge.w = 0; logEvent("NUDGE_OFF"); }
      if (nf) {
        v = clampd(v + nudge.v, -cfg::V_NUDGE_MAX, cfg::V_CRUISE);
        w = clampd(w + nudge.w, -cfg::W_MAX, cfg::W_MAX);
        prog.t = now;
      }
    } else if (isBusy(ms.st)) {
      pauseMask = 0;
    } else if (now - velCmd.last < cfg::V_WATCHDOG_MS && velCmd.ever) {
      pauseMask = 0;
      velTick(dt, v, w);
    } else {
      pauseMask = 0;
      holdActive = false;
    }

    if (estop || gyroCal) {
      vOut = wOut = 0;
      stopAll();
    } else if (motorTest) {
      vOut = wOut = 0;
      for (int i = 0; i < 6; i++) {
        if (i == test.motorIdx) { rpmCmd[i] = 40; MOT[i].m->spin(directionType::fwd, 40, velocityUnits::rpm); }
        else { rpmCmd[i] = 0; MOT[i].m->stop(brakeType::brake); }
      }
    } else {
      applyDrive(v, w, dt);
    }

    if (tick % 2 == 0) sendE();
    if (tick % 5 == 0) sendP();
    if (tick % 20 == 0) sendStatus();

    tick++;
    next += 10;
    int32_t d = (int32_t)(next - nowMs());
    if (d > 0) this_thread::sleep_for(d);
    else next = nowMs();
  }
  return 0;
}
}

int main() { return atsd::run(); }
