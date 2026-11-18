"""Compile and execute the actual portable event queue embedded in VEX firmware."""
from pathlib import Path
import shutil
import subprocess
import pytest


def test_vex_passed_sign_distance_and_independent_queue(tmp_path):
    compiler=shutil.which('g++')
    if not compiler: pytest.skip('g++ unavailable')
    text=(Path(__file__).resolve().parents[3]/'firmware/vex_v5/src/main.cpp').read_text()
    queue=text.split('// BEGIN PASSAGE_QUEUE:')[1].split('\n',1)[1].split('// END PASSAGE_QUEUE')[0]
    source=tmp_path/'check.cpp'
    source.write_text('#include <cassert>\n'+queue+'''
int main() {
  PassageQueue q;
  assert(q.add(1,100)); assert(q.add(2,500));
  assert(q.take(1599)==0); assert(q.take(1600)==1);
  assert(q.take(1600)==0); assert(q.take(1999)==0);
  assert(q.take(2000)==2); assert(q.take(5000)==0);
  for(int i=0;i<16;i++) assert(q.add(2,5000));
  assert(!q.add(1,5000));
  q=PassageQueue(); assert(q.take(10000)==0);
}
''')
    binary=tmp_path/'check'
    subprocess.run([compiler,'-std=c++11','-Wall','-Wextra','-Werror',str(source),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
