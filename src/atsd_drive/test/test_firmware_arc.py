"""Compile the actual pure command limiter from main.cpp, without the hardware SDK."""
from pathlib import Path
import subprocess


def test_brain_preserves_arc_when_slowing_and_stops_rotation_at_obstacle(tmp_path):
    source = (Path(__file__).resolve().parents[3] / 'firmware/vex_v5/src/main.cpp').read_text()
    function = 'static void constrainTravelArc' + source.split('static void constrainTravelArc', 1)[1].split('\n}', 1)[0] + '\n}'
    test = tmp_path / 'arc.cpp'
    test.write_text('#include <cassert>\n#include <cmath>\n' + function + '''
int main() {
  double v = .1, w = .25;
  constrainTravelArc(.25, v, w, false);
  assert(fabs(w - .1) < 1e-9);
  v = .25; w = -.25;
  constrainTravelArc(.25, v, w, true);
  assert(v == 0 && w == 0);
  v = -.1; w = -.25;
  constrainTravelArc(-.25, v, w, true);
  assert(fabs(w + .1) < 1e-9 && v == -.1);
  v = 0; w = .3;
  constrainTravelArc(0, v, w, true);
  assert(v == 0 && w == 0);
}
''')
    executable = tmp_path / 'arc'
    subprocess.run(['g++', '-std=c++11', str(test), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


def test_brain_motor_ramp_keeps_checked_arc_and_curved_stop(tmp_path):
    source = (Path(__file__).resolve().parents[3] / 'firmware/vex_v5/src/main.cpp').read_text()
    function = 'static double travelArcOutput' + source.split('static double travelArcOutput', 1)[1].split('\n}', 1)[0] + '\n}'
    test = tmp_path / 'ramp.cpp'
    test.write_text('#include <cassert>\n#include <cmath>\n' + function + r'''
int main() {
  // mm/s -> rad/s; acceleration preserves radius 1 metre.
  assert(fabs(travelArcOutput(250, .25, 0, 0, 45) - .045) < 1e-9);
  assert(fabs(travelArcOutput(-150, .15, 0, 0, -45) - .045) < 1e-9);
  // A stop continues the current curved braking sweep.
  assert(fabs(travelArcOutput(0, 0, 250, -.25, 170) + .17) < 1e-9);
  assert(fabs(travelArcOutput(0, 0, -150, .15, -70) - .07) < 1e-9);
  assert(travelArcOutput(0, 0, 0, 0, 0) == 0);
}
''')
    executable = tmp_path / 'ramp'
    subprocess.run(['g++', '-std=c++11', str(test), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


def test_actual_travel_tick_honors_pauses_and_faults_before_reverse(tmp_path):
    source = (Path(__file__).resolve().parents[3] / 'firmware/vex_v5/src/main.cpp').read_text()
    functions = ''
    for prefix in ('static void constrainTravelArc', 'static void travelTick'):
        functions += prefix + source.split(prefix, 1)[1].split('\n}', 1)[0] + '\n}\n'
    test = tmp_path / 'travel.cpp'
    test.write_text(r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
namespace cfg {
const double V_CRUISE=800, V_CREEP=100, DECEL_PLAN=450, OBST_STOP_MM=450, OBST_SLOW_MM=1800;
const double V_BACKUP=180, W_MAX=2, STOP_IGNORE_MM=1000;
const uint32_t STOP_HOLD_MS=3000, NAV_CMD_TIMEOUT_MS=300, NAV_LOST_MS=20000;
}
struct {bool sideDead=false,stall=false,bump=false;} hl;
struct {bool ok=true;} piLink;
struct {bool holding=false,pending=false;uint32_t holdUntil=0;double ignoreUntil=0,stopAt=0;} stopSign;
struct {bool red=false;double stopAt=0;} tLight;
struct {double dist=0;} obst;
struct {bool active=false;} obstWait;
struct {double v=-150,w=.15;uint32_t last=999,idleSince=0;} nav;
int pauseMask=0,extLimitPct=100;
enum {P_LINK=2,P_EXT=32,P_STOPSIGN=4,P_RED=8,P_OBST=16};
double tripMm=0,vMeas=0;
const char* fault=nullptr;
uint32_t nowMs(){return 1000;}
double speedFactor(bool){return 1;}
double clampd(double x,double lo,double hi){return fmax(lo,fmin(hi,x));}
void missionFault(const char* why){fault=why;}
template<class... A>void logEvent(const char*,A...){}
bool obstPresent(){return obst.dist>0;}
''' + functions + r'''
int main(){
  double v=0,w=0;
  travelTick(v,w);assert(v==-150&&w==.15);
  piLink.ok=false;travelTick(v,w);assert(v==0&&w==0);
  piLink.ok=true;extLimitPct=0;travelTick(v,w);assert(v==0&&w==0);
  extLimitPct=100;tLight.red=true;travelTick(v,w);assert(v==0&&w==0);
  tLight.red=false;stopSign.holding=true;stopSign.holdUntil=5000;
  travelTick(v,w);assert(v==0&&w==0);
  stopSign.holding=false;hl.bump=true;travelTick(v,w);
  assert(v==0&&w==0&&strcmp(fault,"BUMPER")==0);
  hl.bump=false;hl.stall=true;travelTick(v,w);
  assert(v==0&&w==0&&strcmp(fault,"STALL")==0);
  hl.stall=false;nav.last=500;travelTick(v,w);assert(v==0&&w==0);
  nav.last=999;obst.dist=400;nav.v=250;nav.w=.25;
  travelTick(v,w);assert(v==0&&w==0);
  nav.v=-150;nav.w=.15;travelTick(v,w);assert(v==-150&&w==.15);
}
''')
    executable = tmp_path / 'travel'
    subprocess.run(['g++', '-std=c++11', str(test), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


def test_motor_only_configuration_never_reads_missing_adi_sensors(tmp_path):
    source = (Path(__file__).resolve().parents[3] / 'firmware/vex_v5/src/main.cpp').read_text()
    import re
    declarations = '\n'.join(re.findall(r'const bool\s+(?:USE_GYRO|ENABLE_BUMPERS)\s*=.*?;', source))
    functions = ''
    for prefix in ('static void startGyroCalibration', 'static void bumperUpdate'):
        functions += prefix + source.split(prefix, 1)[1].split('\n}', 1)[0] + '\n}\n'
    test = tmp_path / 'sensors.cpp'
    test.write_text('#include <cassert>\nnamespace cfg {' + declarations + '}\n' + r'''
int reads=0, calibrations=0;
struct {void calibrate(){calibrations++;}} Gyro;
struct Bumper {bool pressing(){reads++;return true;}} BumpL,BumpR;
bool gyroCal=true,gyroFault=true;
double gyroSuspectMs=0;
struct {bool bump=false,bumpPrev=false;} hl;
void logEvent(const char*){}
''' + functions + r'''
int main(){
  static_assert(!cfg::USE_GYRO && !cfg::ENABLE_BUMPERS,"Motor-only configuration");
  startGyroCalibration();bumperUpdate();
  assert(!gyroCal && !gyroFault && !hl.bump);
  assert(reads==0 && calibrations==0);
}
''')
    executable = tmp_path / 'sensors'
    subprocess.run(['g++', '-std=c++11', str(test), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
