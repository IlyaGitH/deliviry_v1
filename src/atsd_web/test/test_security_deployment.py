import importlib.util
import json
from pathlib import Path
import pytest


def load(name, filename):
    path=Path(__file__).parents[2]/'atsd_bringup/security'/filename
    spec=importlib.util.spec_from_file_location(name,path)
    mod=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_audit_detects_edit_and_truncation(tmp_path):
    module=load('audit_test','audit.py')
    (tmp_path/'audit.key').write_bytes(b'k'*32)
    log=module.Audit(tmp_path)
    log.append('OPEN','device','order')
    log.append('DENIED','device','role')
    assert module.Audit(tmp_path).seq==2
    original=(tmp_path/'audit.jsonl').read_bytes()
    (tmp_path/'audit.jsonl').write_bytes(original.replace(b'OPEN',b'XXXX'))
    with pytest.raises(RuntimeError):
        module.Audit(tmp_path)
    (tmp_path/'audit.jsonl').write_bytes(original.splitlines(keepends=True)[0])
    with pytest.raises(RuntimeError):
        module.Audit(tmp_path)


def test_signature_rejects_changed_code(tmp_path,monkeypatch):
    module=load('manifest_test','manage.py')
    module.STATE=tmp_path/'state';module.STATE.mkdir()
    module.CONF=tmp_path/'conf';module.CONF.mkdir()
    source=tmp_path/'code.py';source.write_text('print("test")')
    monkeypatch.setattr(module,'monitored',lambda:[source])
    module.seal()
    module.verify()
    source.write_text('print("changed")')
    with pytest.raises(RuntimeError):
        module.verify()


def test_certificates_have_correct_usage_and_distinct_keys(tmp_path):
    from cryptography import x509
    from cryptography.x509.oid import ExtendedKeyUsageOID
    module=load('pki_test','manage.py')
    module.STATE=tmp_path/'state';module.STATE.mkdir()
    module.CONF=tmp_path/'conf';module.CONF.mkdir()
    key,cert,ca=module.certificate('phone',client=True)
    assert ExtendedKeyUsageOID.CLIENT_AUTH in cert.extensions.get_extension_for_class(x509.ExtendedKeyUsage).value
    _,server,_=module.certificate('robot',sans=[x509.DNSName('rasber.local')])
    assert ExtendedKeyUsageOID.SERVER_AUTH in server.extensions.get_extension_for_class(x509.ExtendedKeyUsage).value
    assert cert.public_key().public_numbers()!=server.public_key().public_numbers()


def test_sensor_archive_roundtrip_and_corruption(tmp_path, monkeypatch):
    import base64
    from cryptography.exceptions import InvalidTag
    module = load('archive_test', 'manage.py')
    module.RECORDING_OUTPUT = tmp_path/'ram'
    key = b'x'*32
    monkeypatch.setattr(module, 'web_config', lambda: {'data_key':base64.urlsafe_b64encode(key).decode()})
    source = tmp_path/'scan.mcap'
    raw = b'secret sensor recording'*100000
    source.write_bytes(raw)
    module.encrypt_recording(source, key)
    encrypted = tmp_path/'scan.mcap.encrypted'
    assert not source.exists() and b'secret sensor' not in encrypted.read_bytes()
    module.decrypt_recording(encrypted)
    output = module.RECORDING_OUTPUT/'scan.mcap'
    assert output.read_bytes() == raw
    output.unlink()
    changed = bytearray(encrypted.read_bytes())
    changed[40] ^= 1
    encrypted.write_bytes(changed)
    with pytest.raises(InvalidTag):
        module.decrypt_recording(encrypted)
    assert not output.exists() and not list(module.RECORDING_OUTPUT.glob('*.partial'))
