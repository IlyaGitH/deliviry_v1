from atsd_drive import protocol as p


def test_parse_encoders_and_flags():
    e = p.parse_line('E 1200 -300 90500 123456 12840 514')
    assert isinstance(e, p.Encoders)
    assert (e.deg_l, e.deg_r, e.heading_mdeg, e.millivolts) == (1200, -300, 90500, 12840)
    assert p.decode_flags(514) == ['estop', 'mission_active']
    assert e.flags & p.FLAG_ESTOP


def test_parse_status_and_pause():
    s = p.parse_line('S TO_DROP 3 1 2/5 12000 12 768')
    assert s.state == 'TO_DROP' and s.from_idx == 3 and s.to_idx == 1
    assert (s.wp, s.wp_count, s.remain_mm, s.mission_ds) == (2, 5, 12000, 768)
    assert p.decode_pause(s.pause) == ['no_pi_link', 'obstacle']


def test_parse_pose_event_info():
    assert p.parse_line('P 1500 -20 -3000 540 -12') == p.Pose(1500, -20, -3000, 540, -12)
    k = p.parse_line('K DETOUR_START L')
    assert k.name == 'DETOUR_START' and k.args == ['L'] and k.raw == 'DETOUR_START L'
    assert p.parse_line('I ATSD-1M 2.0.2').text == 'ATSD-1M 2.0.2'
    assert p.parse_line('garbage') is None
    assert p.parse_line('E 1 2 x') is None


def test_commands_match_firmware_format():
    assert p.cmd_velocity(0.3, -0.5) == 'V 300 -500\n'
    assert p.cmd_mission(3, 1) == 'M 3 1\n'
    assert p.cmd_sign('stop', 2.4) == 'F STOP 2400\n'
    assert p.cmd_light('red', 1.8) == 'T R 1800\n'
    assert p.cmd_obstacle(0.95, 'l') == 'O 950 L 0 0\n'
    assert p.cmd_obstacle(0) == 'O 0\n'
    assert p.cmd_speed_limit(1.7) == 'L 100\n'
    assert p.cmd_pose(1.0, 2.0, 0.0) == 'P 1000 2000 0\n'


def test_topic_parsers():
    assert p.parse_sign_msg('crosswalk 3.1') == ('CROSS', 3.1)
    assert p.parse_sign_msg('bump_warn') == ('BUMP', 0.0)
    assert p.parse_sign_msg('parking') is None
    assert p.parse_light_msg('G') == ('G', 0.0)
    assert p.parse_obstacle_msg('0') == (0.0, '?', 0.0, 0.0)
    assert p.parse_obstacle_msg('0.9 R 0.4 0.65') == (0.9, 'R', 0.4, 0.65)


def test_dock_and_zone():
    assert p.cmd_dock(1.25, -0.04) == 'D 1250 -40\n'
    assert p.cmd_zone(2, -61, True) == 'B 2 -61 1\n'
    assert p.cmd_zone(-1) == 'B -1 -127 0\n'
    assert p.parse_spot_msg('none') is None
    assert p.parse_spot_msg('1.2 -0.1 0.09') == (1.2, -0.1)
    assert p.parse_zone_msg('kpp -58 1') == ('kpp', -58, True)
    assert p.parse_zone_msg('none') is None
    assert 'docking' in p.decode_flags(1 << 16)
