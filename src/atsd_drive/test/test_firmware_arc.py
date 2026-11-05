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
  v = 0; w = .3;
  constrainTravelArc(0, v, w, true);
  assert(v == 0 && w == 0);
}
''')
    executable = tmp_path / 'arc'
    subprocess.run(['g++', '-std=c++11', str(test), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
