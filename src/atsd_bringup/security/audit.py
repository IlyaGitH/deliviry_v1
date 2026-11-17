#!/usr/bin/python3
"""Root-owned append-only interface. Chain/anchor detect edits by unprivileged users."""
import hashlib
import fcntl
from contextlib import contextmanager
import hmac
import json
import os
from pathlib import Path
import pwd
import socket
import struct
import sys
import time

ROOT = Path('/var/lib/atsd-security')
SOCKET = Path('/run/atsd-audit/socket')


def canonical(data):
    return json.dumps(data, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode()


class Audit:
    def __init__(self, root=ROOT, read_only=False):
        self.root = root
        root.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.log, self.anchor = root/'audit.jsonl', root/'audit.anchor'
        self.key = (root/'audit.key').read_bytes()
        with self.locked():
            self.load(read_only)

    @contextmanager
    def locked(self):
        with (self.root/'audit.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(lock, fcntl.LOCK_UN)

    def load(self, read_only=False):
        self.seq, self.previous = 0, '0'*64
        anchor = json.loads(self.anchor.read_text()) if self.anchor.exists() else [0, self.previous]
        anchor_seen = anchor == [0, self.previous]
        if self.log.exists():
            with self.log.open('rb') as stream:
                for line in stream:
                    row = json.loads(line)
                    signature = row.pop('mac')
                    if row['seq'] != self.seq+1 or row['previous'] != self.previous:
                        raise RuntimeError('Audit sequence changed')
                    actual = hmac.new(self.key, canonical(row), hashlib.sha256).hexdigest()
                    if not hmac.compare_digest(signature, actual):
                        raise RuntimeError('Audit integrity failed')
                    self.seq, self.previous = row['seq'], signature
                    anchor_seen |= anchor == [self.seq, signature]
        if not anchor_seen:
            raise RuntimeError('Audit truncated or anchor changed')
        if not read_only:
            self.save_anchor()

    def save_anchor(self):
        temp = self.anchor.with_suffix('.tmp')
        with temp.open('w') as stream:
            json.dump([self.seq, self.previous], stream)
            stream.flush()
            os.fsync(stream.fileno())
        temp.replace(self.anchor)
        descriptor = os.open(self.root, os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    def append(self, event, identity='', detail=''):
        with self.locked():
            self._append(event, identity, detail)

    def _append(self, event, identity='', detail=''):
        if self.log.exists() and self.log.stat().st_size > 50_000_000:
            raise RuntimeError('Audit storage full: archive via administrator')
        row = dict(seq=self.seq+1, previous=self.previous, at=time.time(),
                   event=str(event)[:80], identity=str(identity)[:80], detail=str(detail)[:400])
        signature = hmac.new(self.key, canonical(row), hashlib.sha256).hexdigest()
        row['mac'] = signature
        with self.log.open('ab') as stream:
            stream.write(canonical(row)+b'\n')
            stream.flush()
            os.fsync(stream.fileno())
        self.seq, self.previous = row['seq'], signature
        self.save_anchor()


def main():
    os.umask(0o077)
    audit = Audit(read_only=sys.argv[1:] == ['verify'])
    if sys.argv[1:] == ['verify']:
        print(f'Audit verified: {audit.seq} records')
        return
    uid = pwd.getpwnam('atsd').pw_uid
    SOCKET.parent.mkdir(exist_ok=True)
    SOCKET.unlink(missing_ok=True)
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
        server.bind(str(SOCKET))
        os.chown(SOCKET, 0, pwd.getpwnam('atsd').pw_gid)
        SOCKET.chmod(0o660)
        server.listen(16)
        while True:
            conn, _ = server.accept()
            with conn:
                conn.settimeout(1)
                try:
                    _, peer_uid, _ = struct.unpack('3i', conn.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
                    if peer_uid not in (0, uid):
                        continue
                    data = bytearray()
                    while b'\n' not in data and len(data) < 4096:
                        chunk = conn.recv(1024)
                        if not chunk:
                            break
                        data.extend(chunk)
                    event = json.loads(data.split(b'\n',1)[0])
                    audit.append(event['event'], event.get('identity',''), event.get('detail',''))
                    conn.sendall(b'OK\n')
                except (ValueError, KeyError, OSError, RuntimeError):
                    try:
                        conn.sendall(b'ERROR\n')
                    except OSError:
                        pass


if __name__ == '__main__':
    main()
