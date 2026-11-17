#!/usr/bin/python3
"""Provision the robot locally. Never sends credentials to external services."""
import argparse
import base64
from datetime import datetime, timedelta, timezone
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import pwd
import secrets
import shutil
import socket
import subprocess

from cryptography import x509
from cryptography.fernet import Fernet
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa, ed25519
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives.serialization import pkcs12
from cryptography.x509.oid import NameOID, ExtendedKeyUsageOID

CONF = Path('/etc/atsd/security')
STATE = Path('/var/lib/atsd-security')
LIB = Path('/usr/local/libexec')
WS = Path('/opt/atsd/ws')
DATA = Path('/opt/atsd/data')
RECORDINGS = Path('/opt/atsd')
EXPORT = STATE/'access'
RECORDING_MAGIC = b'ATSDREC1'
RECORDING_OUTPUT = Path('/run/atsd-recordings')


def encrypt_recording(source, key):
    """Bounded-memory authenticated archive; remove plaintext only after durable success."""
    target = source.with_suffix(source.suffix+'.encrypted')
    temp = target.with_suffix('.partial')
    if target.exists():
        raise RuntimeError('Archive already exists: '+str(target))
    before = source.stat()
    nonce = secrets.token_bytes(12)
    encryptor = Cipher(algorithms.AES(key), modes.GCM(nonce)).encryptor()
    encryptor.authenticate_additional_data(RECORDING_MAGIC)
    try:
        with source.open('rb') as src, temp.open('xb') as dst:
            temp.chmod(0o600)
            dst.write(RECORDING_MAGIC+nonce)
            while block := src.read(1024*1024):
                dst.write(encryptor.update(block))
            dst.write(encryptor.finalize()+encryptor.tag)
            dst.flush()
            os.fsync(dst.fileno())
        after = source.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise RuntimeError('Recording is still being written: '+str(source))
        temp.replace(target)
        directory = os.open(source.parent, os.O_DIRECTORY)
        try:
            os.fsync(directory)
            source.unlink()
            os.fsync(directory)
        finally:
            os.close(directory)
    except Exception:
        temp.unlink(missing_ok=True)
        raise


def decrypt_recording(source):
    """Restore only to RAM, verifying the tag before exposing the finished file."""
    source = Path(source)
    output = RECORDING_OUTPUT
    output.mkdir(mode=0o700, exist_ok=True)
    target = output/source.name.removesuffix('.encrypted')
    if target.exists():
        raise ValueError('Output already exists: '+str(target))
    temp = target.with_suffix(target.suffix+'.partial')
    key = base64.urlsafe_b64decode(web_config()['data_key'])
    try:
        with source.open('rb') as src, temp.open('xb') as dst:
            temp.chmod(0o600)
            if src.read(8) != RECORDING_MAGIC:
                raise ValueError('Unknown recording format')
            nonce = src.read(12)
            src.seek(-16, 2)
            tag = src.read(16)
            remaining = src.tell()-36
            if remaining < 0:
                raise ValueError('Truncated recording')
            decryptor = Cipher(algorithms.AES(key), modes.GCM(nonce, tag)).decryptor()
            decryptor.authenticate_additional_data(RECORDING_MAGIC)
            src.seek(20)
            while remaining:
                block = src.read(min(remaining, 1024*1024))
                if not block:
                    raise ValueError('Truncated recording')
                dst.write(decryptor.update(block))
                remaining -= len(block)
            dst.write(decryptor.finalize())
        temp.replace(target)
    except Exception:
        temp.unlink(missing_ok=True)
        raise
    print('Decrypted to volatile storage (removed on reboot):', target)


def write(path, data, mode=0o600):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name+'.'+secrets.token_hex(6)+'.tmp')
    try:
        descriptor = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_EXCL, mode)
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(data if isinstance(data, bytes) else data.encode())
            stream.flush()
            os.fsync(stream.fileno())
        temp.chmod(mode)
        temp.replace(path)
        descriptor = os.open(path.parent, os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        temp.unlink(missing_ok=True)


def save_key(path, key):
    write(path, key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                 serialization.NoEncryption()))


def ca_pair():
    if not (STATE/'ca.key').exists():
        key = rsa.generate_private_key(public_exponent=65537, key_size=3072)
        name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, 'ATSD private device CA')])
        now = datetime.now(timezone.utc)
        cert = (x509.CertificateBuilder().subject_name(name).issuer_name(name)
                .public_key(key.public_key()).serial_number(x509.random_serial_number())
                .not_valid_before(now-timedelta(days=1)).not_valid_after(now+timedelta(days=3650))
                .add_extension(x509.BasicConstraints(ca=True, path_length=0), critical=True)
                .add_extension(x509.KeyUsage(False,False,False,False,False,True,True,False,False), critical=True)
                .sign(key, hashes.SHA256()))
        save_key(STATE/'ca.key', key)
        write(CONF/'ca.crt', cert.public_bytes(serialization.Encoding.PEM), 0o644)
    return (serialization.load_pem_private_key((STATE/'ca.key').read_bytes(), None),
            x509.load_pem_x509_certificate((CONF/'ca.crt').read_bytes()))


def certificate(name, client=False, sans=()):
    ca_key, ca = ca_pair()
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    now = datetime.now(timezone.utc)
    builder = (x509.CertificateBuilder()
               .subject_name(x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, name)]))
               .issuer_name(ca.subject).public_key(key.public_key())
               .serial_number(x509.random_serial_number())
               .not_valid_before(now-timedelta(days=1)).not_valid_after(now+timedelta(days=365))
               .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
               .add_extension(x509.ExtendedKeyUsage([ExtendedKeyUsageOID.CLIENT_AUTH if client
                                                     else ExtendedKeyUsageOID.SERVER_AUTH]), critical=False))
    if sans:
        builder = builder.add_extension(x509.SubjectAlternativeName(list(sans)), critical=False)
    return key, builder.sign(ca_key, hashes.SHA256()), ca


def web_config():
    return json.loads((CONF/'web.json').read_text())


def save_config(config):
    write(CONF/'web.json', json.dumps(config), 0o640)
    os.chown(CONF/'web.json', 0, pwd.getpwnam('atsd').pw_gid)


def add_client(name):
    if not name or len(name)>32 or any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789-' for c in name):
        raise ValueError('Client name: lowercase latin letters, numbers, hyphen')
    config = web_config()
    if name in config['certificates'].values():
        return
    key, cert, ca = certificate('ATSD '+name, client=True)
    password = secrets.token_urlsafe(18)
    # Modern PBES2 PKCS#12; Android, iOS and current desktop browsers support this.
    p12 = pkcs12.serialize_key_and_certificates(
        name.encode(), key, cert, [ca], serialization.BestAvailableEncryption(password.encode()))
    write(EXPORT/(name+'.p12'), p12)
    write(EXPORT/(name+'-password.txt'), password+'\n')
    config['certificates'][cert.fingerprint(hashes.SHA1()).hex().upper()] = name
    save_config(config)


def provision():
    os.umask(0o077)
    for path in (CONF, STATE, EXPORT):
        path.mkdir(parents=True, exist_ok=True)
        path.chmod(0o750 if path == CONF else 0o700)
    os.chown(CONF, 0, pwd.getpwnam('atsd').pw_gid)
    ca_pair()
    if not (CONF/'web.json').exists():
        password, salt = secrets.token_urlsafe(24), secrets.token_hex(16)
        config = {'proxy_secret':secrets.token_hex(32), 'certificates':{},
                  'data_key':Fernet.generate_key().decode(), 'salt':salt,
                  'admin_hash':hashlib.scrypt(password.encode(), salt=bytes.fromhex(salt),
                                              n=16384,r=8,p=1).hex()}
        save_config(config)
        write(EXPORT/'admin-password.txt', password+'\n')
    if not (STATE/'audit.key').exists():
        write(STATE/'audit.key', secrets.token_bytes(32))
    for name in ('android', 'ios', 'ubuntu'):
        add_client(name)
    names = {socket.gethostname(), socket.gethostname()+'.local', 'rasber.local', 'atsd.local', 'localhost'}
    sans = [x509.DNSName(n) for n in sorted(names)]
    ips = {'10.42.0.1', '127.0.0.1'}
    try:
        for value in subprocess.check_output(['hostname','-I'], text=True).split():
            if ipaddress.ip_address(value).version == 4:
                ips.add(value)
    except subprocess.CalledProcessError:
        pass
    sans.extend(x509.IPAddress(ipaddress.ip_address(ip)) for ip in sorted(ips))
    key, cert, _ = certificate(socket.gethostname()+'.local', sans=sans)
    save_key(CONF/'server.key', key)
    write(CONF/'server.crt', cert.public_bytes(serialization.Encoding.PEM))
    shutil.copyfile(CONF/'ca.crt', EXPORT/'atsd-ca.crt')
    config = web_config()
    orders = DATA/'orders.json'
    cipher = Fernet(config['data_key'].encode())
    if orders.exists():
        raw = orders.read_bytes()
        if raw.lstrip().startswith(b'['):
            parsed = json.loads(raw)
            for order in parsed:
                for role in ('sender','receiver'):
                    token = order.pop(role+'_token', None)
                    if token:
                        order[role+'_token_hash'] = hashlib.sha256(token.encode()).hexdigest()
            temp = orders.with_suffix('.secure')
            write(temp, cipher.encrypt(json.dumps(parsed,ensure_ascii=False).encode()))
            os.chown(temp, pwd.getpwnam('atsd').pw_uid, pwd.getpwnam('atsd').pw_gid)
            temp.replace(orders)
        else:
            cipher.decrypt(raw)  # never silently discard a corrupt or wrong-key order store
    for folder in RECORDINGS.glob('lidar-check-*'):
        if folder.is_dir():
            for raw_file in folder.rglob('*'):
                if raw_file.is_file() and raw_file.suffix in ('.mcap', '.db3', '.bag'):
                    encrypt_recording(raw_file, base64.urlsafe_b64decode(config['data_key']))
    write(EXPORT/'README.txt',
          'АТСД: https://rasber.local/ или https://10.42.0.1/\n'
          'Установите atsd-ca.crt как доверенный CA и только P12 своего устройства.\n'
          'Пароль импорта находится в соответствующем *-password.txt.\n'
          'Пароль администратора — admin-password.txt; не выдавайте его получателям.\n'
          'Этот каталог закрытый: не загружайте его в Git или открытое облако конкурса.\n'
          'Камера: https://rasber.local/debug/ (только администратор).\n'
          'Инструкция: https://github.com/IlyaGitH/deliviry_v1/blob/main/docs/security.md\n'
          'CA SHA256: '+ca_pair()[1].fingerprint(hashes.SHA256()).hex()+'\n')
    proxy = config['proxy_secret']
    nginx = '''user www-data;
worker_processes 1;
pid /run/atsd-gateway/nginx.pid;
error_log /var/log/nginx/atsd-error.log warn;
events { worker_connections 256; }
http {
    include /etc/nginx/mime.types;
    access_log off;
    server_tokens off;
    client_max_body_size 1m;
    client_body_buffer_size 1m;
    client_body_temp_path /run/atsd-gateway/body;
    proxy_temp_path /run/atsd-gateway/proxy;
    map $http_upgrade $atsd_connection { default upgrade; '' close; }
    server {
        listen 443 ssl;
        ssl_certificate /etc/atsd/security/server.crt;
        ssl_certificate_key /etc/atsd/security/server.key;
        ssl_client_certificate /etc/atsd/security/ca.crt;
        ssl_verify_client on;
        ssl_verify_depth 1;
        ssl_protocols TLSv1.2 TLSv1.3;
        ssl_session_tickets off;
        ssl_early_data off;
        add_header Strict-Transport-Security "max-age=86400" always;
        add_header Content-Security-Policy "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' blob: data:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'self'" always;
        location / {
            proxy_pass http://unix:/run/atsd-web/http.sock:;
            proxy_http_version 1.1;
            proxy_set_header Host $http_host;
            proxy_set_header X-ATSD-Certificate $ssl_client_fingerprint;
            proxy_set_header X-ATSD-Verified $ssl_client_verify;
            proxy_set_header X-ATSD-Proxy PROXY_SECRET;
            proxy_set_header Upgrade $http_upgrade;
            proxy_set_header Connection $atsd_connection;
            proxy_buffering off;
            proxy_request_buffering off;
            proxy_read_timeout 60s;
        }
    }
}
'''.replace('PROXY_SECRET', proxy)
    write(CONF/'nginx.conf', nginx)
    print('Security provisioned. Private access package:', EXPORT)


def monitored():
    roots = [WS/'src', WS/'install', Path('/usr/local/libexec'),
             Path('/etc/systemd/system'), Path('/etc/udev/rules.d'), Path('/etc/sudoers.d')]
    files = []
    for root in roots:
        if not root.exists():
            continue
        for p in root.rglob('*') if root in roots[:2] else root.glob('*atsd*'):
            if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc':
                files.append(p)
    for directory in Path('/etc/systemd/system').glob('atsd*.service.d'):
        files += [p for p in directory.glob('*.conf') if p.is_file()]
    files += [Path('/opt/atsd/run.sh'), CONF/'web.json', CONF/'nginx.conf', CONF/'server.crt',
              CONF/'ca.crt', CONF/'ros-network.nft', Path('/etc/atsd/cyclonedds.xml'),
              Path('/etc/profile.d/atsd-ros.sh'), Path('/etc/atsd/security/verify.pub')]
    return sorted(set(files))


def seal():
    if not (STATE/'signing.key').exists():
        save_key(STATE/'signing.key', ed25519.Ed25519PrivateKey.generate())
    key = serialization.load_pem_private_key((STATE/'signing.key').read_bytes(), None)
    write(CONF/'verify.pub', key.public_key().public_bytes(
        serialization.Encoding.PEM, serialization.PublicFormat.SubjectPublicKeyInfo), 0o644)
    entries = {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in monitored()}
    raw = json.dumps(entries, sort_keys=True).encode()
    write(CONF/'manifest.json', raw, 0o644)
    write(CONF/'manifest.sig', key.sign(raw), 0o644)
    print('Release sealed:', len(entries), 'files')


def verify():
    raw = (CONF/'manifest.json').read_bytes()
    pub = serialization.load_pem_public_key((CONF/'verify.pub').read_bytes())
    pub.verify((CONF/'manifest.sig').read_bytes(), raw)
    entries = json.loads(raw)
    for name, expected in entries.items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != expected:
            raise RuntimeError('Integrity failure: '+name)
    if set(entries) != {str(p) for p in monitored()}:
        raise RuntimeError('Release file set changed')
    print('Release integrity verified')


def export(owner):
    account = pwd.getpwnam(owner)
    if account.pw_uid == 0 or account.pw_dir in ('/', '/opt/atsd'):
        raise ValueError('Export to the SSH operator account, not the service/root account')
    destination = Path(account.pw_dir)/'atsd-access'
    if destination.is_symlink() or destination.exists():
        destination = Path(account.pw_dir)/('atsd-access-'+secrets.token_hex(4))
    shutil.copytree(EXPORT, destination)
    for path in [destination, *destination.rglob('*')]:
        os.chown(path, account.pw_uid, account.pw_gid)
        path.chmod(0o700 if path.is_dir() else 0o600)
    print('Private access package:', destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['provision','seal','verify','export','add-client','revoke','decrypt-recording'])
    parser.add_argument('value', nargs='?')
    args = parser.parse_args()
    if args.action == 'verify':
        verify()
    else:
        if os.geteuid() != 0:
            raise SystemExit('Run via sudo')
        if args.action == 'provision': provision()
        elif args.action == 'seal': seal()
        elif args.action == 'export': export(args.value)
        elif args.action == 'decrypt-recording': decrypt_recording(args.value)
        elif args.action == 'add-client':
            add_client(args.value); seal()
            subprocess.run(['systemctl','restart','atsd-web'], check=True)
        elif args.action == 'revoke':
            config = web_config()
            config['certificates'] = {k:v for k,v in config['certificates'].items() if v != args.value}
            save_config(config); seal()
            subprocess.run(['systemctl','restart','atsd-web'], check=True)
