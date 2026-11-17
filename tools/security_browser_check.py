#!/usr/bin/env python3
"""Isolated TLS/browser check, without ROS or a robot. Run as a regular user.

Requires nginx, Chromium and Python packages playwright, cryptography, fastapi,
uvicorn, numpy, pydantic. ATSD_TEST_NGINX/ATSD_TEST_CHROMIUM override executables.
Uses a temporary CA and mock robot; never touches the production configuration.
"""
import importlib.util, os, pwd, json, hashlib, subprocess, time, sys, tempfile, shutil
from pathlib import Path
from types import SimpleNamespace
from cryptography import x509
from cryptography.hazmat.primitives.serialization import pkcs12, Encoding, PrivateFormat, NoEncryption
from cryptography.hazmat.primitives import hashes
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[1]
if os.geteuid() == 0:
 raise SystemExit('Run as a regular user, not root')
NGINX=os.environ.get('ATSD_TEST_NGINX') or shutil.which('nginx')
CHROMIUM=os.environ.get('ATSD_TEST_CHROMIUM') or shutil.which('chromium') or shutil.which('chromium-browser')
if not NGINX or not CHROMIUM:
 raise SystemExit('Install nginx and Chromium or set ATSD_TEST_NGINX/ATSD_TEST_CHROMIUM')
TMP=Path(tempfile.mkdtemp(prefix='atsd-security-e2e-'))
def module(name,path):
 s=importlib.util.spec_from_file_location(name,path); m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
m=module('provision',ROOT/'src/atsd_bringup/security/manage.py')
m.CONF=TMP/'conf';m.STATE=TMP/'state';m.EXPORT=m.STATE/'access'
m.DATA=TMP/'data';m.RECORDINGS=TMP/'recordings'
real_chown=os.chown;real_pwd=pwd.getpwnam
m.os.chown=lambda *a:None
m.pwd.getpwnam=lambda _:pwd.getpwuid(os.getuid())
m.provision()
m.os.chown=real_chown;m.pwd.getpwnam=real_pwd
for device in ('android','ios','ubuntu'):
 key,cert,_=pkcs12.load_key_and_certificates((m.EXPORT/(device+'.p12')).read_bytes(),
                                          (m.EXPORT/(device+'-password.txt')).read_text().strip().encode())
 (TMP/(device+'.key')).write_bytes(key.private_bytes(Encoding.PEM,PrivateFormat.PKCS8,NoEncryption()))
 (TMP/(device+'.crt')).write_bytes(cert.public_bytes(Encoding.PEM))
(TMP/'data').mkdir(exist_ok=True)
backend=TMP/'backend.py'
backend.write_text("""
import json,sys
from pathlib import Path
import uvicorn
from atsd_web import server,debug
from atsd_web.secure_runtime import Security
from atsd_web.ros_link import RosLink
root=Path(sys.argv[1])
server.DATA_DIR=root/'data'; server.DATA_DIR.mkdir(exist_ok=True)
server.POINTS_FILE=server.DATA_DIR/'points.json'
server.ROADS_FILE=server.DATA_DIR/'roads.json'
server.ORDERS_FILE=server.DATA_DIR/'orders.json'
def audit(*args):
 with (root/'audit-test.jsonl').open('a') as f: f.write(json.dumps(args)+'\\n')
server.security=Security(json.loads((root/'conf/web.json').read_text()),root/'tickets.db',audit=audit)
server.link=RosLink(mock=True)
server._save(server.ROADS_FILE,{'nodes':{'kpp':[2,0],'admin':[18,0]},'edges':[['kpp','admin']]})
server._save(server.POINTS_FILE,server.DEFAULT_POINTS)
server.link._set(['road'],{'ready':True})
server.link._set(['brain_link'],True)
debug.attach(server.link,server.STATIC,server.api_config,server.api_map_image,lambda:server.security)
server.app.mount('/debug',debug.debug_app)
uvicorn.run(server.app,uds=str(root/'http.sock'),access_log=False)
""")
config=(m.CONF/'nginx.conf').read_text()
config=config.replace('user www-data;','')
config=config.replace('/run/atsd-gateway',str(TMP/'run')).replace('/var/log/nginx/atsd-error.log',str(TMP/'nginx-error.log'))
config=config.replace('include /etc/nginx/mime.types;', 'types { text/html html; application/javascript js; text/css css; image/jpeg jpg; image/png png; }')
config=config.replace('/etc/atsd/security',str(m.CONF)).replace('/run/atsd-web/http.sock',str(TMP/'http.sock'))
config=config.replace('listen 443 ssl','listen 127.0.0.1:18443 ssl')
(TMP/'run').mkdir(exist_ok=True)
(TMP/'logs').mkdir(exist_ok=True)
(TMP/'nginx.conf').write_text(config)
env=dict(os.environ,PYTHONPATH=str(ROOT/'src/atsd_web')+':'+str(ROOT/'src/atsd_drive'))
(TMP/'http.sock').unlink(missing_ok=True)
be=subprocess.Popen([sys.executable,str(backend),str(TMP)],env=env,stdout=(TMP/'backend.log').open('w'),stderr=subprocess.STDOUT)
ng=None
try:
 for _ in range(100):
  if (TMP/'http.sock').exists():break
  time.sleep(.05)
 nginx=NGINX
 subprocess.run([nginx,'-t','-p',str(TMP),'-c',str(TMP/'nginx.conf')],check=True)
 ng=subprocess.Popen([nginx,'-p',str(TMP),'-c',str(TMP/'nginx.conf'),'-g','daemon off;'])
 time.sleep(.4)
 url='https://127.0.0.1:18443'
 def curl(extra):
  r=subprocess.run(['curl','--silent','--show-error','--cacert',str(m.CONF/'ca.crt'),
                    '-o',str(TMP/'response.txt'),'-w','%{http_code}',*extra,url+'/login'],
                   text=True,capture_output=True)
  return r.stdout
 assert curl([]) in ('400','000'), 'client cert required'
 assert curl(['--cert',str(TMP/'android.crt'),'--key',str(TMP/'android.key')])=='200'
 with sync_playwright() as pw:
  browser=pw.chromium.launch(executable_path=CHROMIUM,headless=True,args=['--no-sandbox'])
  # Browser server trust bypass is limited to this isolated test. curl above verifies real CA/SAN.
  context=browser.new_context(ignore_https_errors=True,client_certificates=[
     {'origin':url,'certPath':str(TMP/'android.crt'),'keyPath':str(TMP/'android.key')}])
  page=context.new_page();errors=[]
  page.on('pageerror',lambda e:errors.append(str(e)))
  page.goto(url+'/login')
  page.fill('#secret',(m.EXPORT/'admin-password.txt').read_text().strip())
  page.click('button[type=submit]')
  page.wait_for_url(url+'/',timeout=10000)
  page.locator('#selFrom option').first.wait_for(state='attached')
  page.screenshot(path=str(TMP/'admin.png'))
  page.click('#btnCreate')
  page.locator('#orderLinks:not(.hidden)').wait_for()
  sender_link=page.locator('#lkSender').input_value()
  receiver_link=page.locator('#lkReceiver').input_value()
  assert '/login#role=sender' in sender_link
  assert not errors,errors
  assert context.request.get(url+'/api/security/me').json()['role']=='admin'
  page.goto(url+'/debug/')
  page.wait_for_timeout(600)
  assert not errors,errors
  page.screenshot(path=str(TMP/'debug.png'))
  for device,link,role in [('ios',sender_link,'sender'),('ubuntu',receiver_link,'receiver')]:
   guest=browser.new_context(ignore_https_errors=True,client_certificates=[
     {'origin':url,'certPath':str(TMP/(device+'.crt')),'keyPath':str(TMP/(device+'.key'))}])
   client=guest.new_page();guest_errors=[]
   client.on('pageerror',lambda e:guest_errors.append(str(e)))
   client.goto(link)
   client.click('button[type=submit]')
   client.wait_for_url(url+'/',timeout=10000)
   client.locator('#roleLabel').filter(has_text=('вы отправитель' if role=='sender' else 'вы получатель')).wait_for()
   client.wait_for_timeout(400)
   assert not guest_errors,guest_errors
   assert guest.request.get(url+'/api/security/me').json()['role']==role
   assert guest.request.get(url+'/api/orders').status==403
   assert guest.request.get(url+'/debug/frame.jpg').status==403
   client.screenshot(path=str(TMP/(role+'.png')))
   guest.close()
  browser.close()
 print('PASS: mTLS rejects no certificate; valid CA/server/client works; Chromium admin/dashboard/debug plus sender and receiver with separate certificates')
finally:
 if ng:ng.terminate();ng.wait(timeout=5)
 be.terminate();be.wait(timeout=5)
 shutil.rmtree(TMP)
