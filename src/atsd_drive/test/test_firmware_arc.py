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
struct {bool sideDead=false,stall=false;} hl;
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
  stopSign.holding=false;hl.stall=true;travelTick(v,w);
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

def test_encoder_odometry_without_external_sensors(tmp_path):
    source = (Path(__file__).resolve().parents[3] / 'firmware/vex_v5/src/main.cpp').read_text()
    import re
    assert not re.search(r'gyro|bumper|ThreeWirePort|!GC', source, re.I)
    function = 'static void odomUpdate' + source.split('static void odomUpdate',1)[1].split('\n}',1)[0]+'\n}'
    test=tmp_path/'odom.cpp'
    test.write_text(r'''
#include <cassert>
#include <cmath>
namespace cfg {const double TRACK_EFF_MM=430;}
namespace rotationUnits {const int deg=0;}
struct Motor {double pos=0;bool ok=true;bool installed(){return ok;}double position(int){return pos;}} motors[6];
struct Slot {Motor* m;bool left;} MOT[6]={{&motors[0],true},{&motors[1],true},{&motors[2],true},{&motors[3],false},{&motors[4],false},{&motors[5],false}};
double prevPos[6]={};bool prevOk[6]={};
const double MM_PER_MOTOR_DEG=1.;
double encDegL=0,encDegR=0,encMmL=0,encMmR=0,tripMm=0,vMeas=0,wMeas=0;
struct {double x=0,y=0,yaw=0;} pose;
''' + function + r'''
int main(){
  odomUpdate(.1);
  for(auto& m:motors)m.pos=100;
  odomUpdate(.1);assert(fabs(pose.x-100)<1e-9&&pose.y==0&&pose.yaw==0);
  for(int i=3;i<6;i++)motors[i].pos+=43;
  odomUpdate(.1);assert(fabs(pose.yaw-.1)<1e-9);
  const double yaw=pose.yaw,x=pose.x;
  for(auto& m:motors)m.pos-=20;
  odomUpdate(.1);assert(fabs(pose.yaw-yaw)<1e-9&&pose.x<x);
  // Reconnecting a motor cannot inject its absolute encoder reading as travel.
  motors[0].ok=false;odomUpdate(.1);motors[0].pos=10000;motors[0].ok=true;
  const double before=pose.x;odomUpdate(.1);assert(fabs(pose.x-before)<1e-9);
}
''')
    exe=tmp_path/'odom'
    subprocess.run(['g++','-std=c++11',str(test),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
