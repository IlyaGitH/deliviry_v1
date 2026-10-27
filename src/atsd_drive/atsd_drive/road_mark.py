#!/usr/bin/env python3
# Снять узел графа дорог по GPS: ros2 run atsd_drive road_mark kpp --link n3
# Стоять на месте: 10 с фиксы усредняются. Первый узел задаёт начало координат (origin).
import argparse
import json
import os
import time

import rclpy
from sensor_msgs.msg import NavSatFix, NavSatStatus

from atsd_drive.road_logic import to_local


def save(path, data):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    os.replace(tmp, path)


def main():
    ap = argparse.ArgumentParser(description='Записать узел графа дорог по GPS')
    ap.add_argument('id', help='имя узла: перекрёсток (n1, n2…) или точка заказа (kpp, admin…)')
    ap.add_argument('--link', action='append', default=[], help='соединить дорогой с узлом (можно несколько)')
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--roads', default='/opt/atsd/data/roads.json')
    ap.add_argument('--points', default='/opt/atsd/data/points.json')
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('road_mark')
    fixes = []
    node.create_subscription(NavSatFix, 'gnss/fix', lambda m: fixes.append(m)
                             if m.status.status >= NavSatStatus.STATUS_FIX else None, 10)
    print(f'Стойте на месте, {args.seconds:.0f} с собираю GPS…')
    t_end = time.monotonic() + args.seconds
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if len(fixes) < 5:
        raise SystemExit(f'Мало фиксов GPS ({len(fixes)}) — нет спутников или GPS заглушён, узел не записан')

    lat = sum(m.latitude for m in fixes) / len(fixes)
    lon = sum(m.longitude for m in fixes) / len(fixes)
    roads = {'origin': None, 'nodes': {}, 'edges': []}
    if os.path.exists(args.roads):
        with open(args.roads, encoding='utf-8') as f:
            roads = json.load(f)
    if not roads.get('origin'):
        roads['origin'] = [lat, lon]
        print(f'Начало координат карты: {lat:.7f}, {lon:.7f}')
    x, y = to_local(lat, lon, roads['origin'])
    roads['nodes'][args.id] = [round(x, 2), round(y, 2)]
    for other in args.link:
        if other not in roads['nodes']:
            print(f'Узла «{other}» нет — дорога к нему не записана')
        elif [other, args.id] not in roads['edges'] and [args.id, other] not in roads['edges']:
            roads['edges'].append([other, args.id])
    save(args.roads, roads)
    print(f'Узел «{args.id}»: x {x:.1f} м, y {y:.1f} м ({len(fixes)} фиксов)')

    if os.path.exists(args.points):  # точка заказа с тем же id — та же позиция на карте веба
        with open(args.points, encoding='utf-8') as f:
            points = json.load(f)
        for pt in points:
            if pt.get('id') == args.id:
                pt['x'], pt['y'] = round(x, 2), round(y, 2)
                save(args.points, points)
                print(f'Точка заказа «{pt.get("name", args.id)}» перенесена сюда')


if __name__ == '__main__':
    main()
