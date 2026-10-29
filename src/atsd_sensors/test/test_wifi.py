import json

from atsd_sensors.wifi_logic import parse_iw, parse_nmcli, parse_response, request_body, usable

NMCLI = r"""74\:4D\:28\:CD\:6A\:10:72:physics
86\:D9\:E7\:C7\:8B\:20:40:Kafedra\:2
86\:D9\:E7\:C7\:8B\:20:55:Kafedra\:2
1A\:2B\:3C\:4D\:5E\:6F:90:redmi_13
B8\:69\:F4\:72\:55\:D9:30:
C0\:FF\:EE\:00\:00\:01:50:home_nomap
"""


def test_parse_and_filter():
    aps = parse_nmcli(NMCLI)
    assert aps[1] == ('86:d9:e7:c7:8b:20', 40, 'Kafedra:2')
    # скрытая и _nomap выброшены; дубль — по сильнейшему; сильные первыми
    assert usable(aps) == [('1a:2b:3c:4d:5e:6f', -55.0), ('74:4d:28:cd:6a:10', -64.0),
                           ('86:d9:e7:c7:8b:20', -72.5)]


def test_requests():
    aps = [('74:4d:28:cd:6a:10', -64.0)]
    assert json.loads(request_body('yandex', aps)) == {'wifi': [{'bssid': '74:4d:28:cd:6a:10', 'signal_strength': -64}]}
    assert json.loads(request_body('beacondb', aps))['wifiAccessPoints'][0]['macAddress'] == '74:4d:28:cd:6a:10'


def test_responses():
    assert parse_response('yandex', {'location': {'point': {'lat': 53.5, 'lon': 49.4}, 'accuracy': 40}}) == (53.5, 49.4, 40.0)
    assert parse_response('yandex', {}) is None
    assert parse_response('beacondb', {'location': {'lat': 53.5, 'lng': 49.4}, 'accuracy': 25.0}) == (53.5, 49.4, 25.0)
    assert parse_response('beacondb', {'location': {'lat': 53.5, 'lng': 49.4}, 'accuracy': 5000.0, 'fallback': 'ipf'}) is None


IW = """BSS 50:ff:20:d4:da:9d(on wlan0) -- associated
\tfreq: 2442
\tsignal: -58.00 dBm
\tSSID: physics
BSS 50:ff:20:d4:da:9e(on wlan0)
\tsignal: -81.00 dBm
\tSSID:
BSS 11:22:33:44:55:66(on wlan0)
\tsignal: -70.00 dBm
\tSSID: Kafedra 2
"""


def test_parse_iw():
    aps = parse_iw(IW)
    assert aps[0] == ('50:ff:20:d4:da:9d', 84, 'physics')
    assert usable(aps) == [('50:ff:20:d4:da:9d', -58.0), ('11:22:33:44:55:66', -70.0)]  # скрытая выброшена
