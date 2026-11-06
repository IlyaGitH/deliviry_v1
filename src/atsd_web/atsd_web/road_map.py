"""Map data validation and OpenStreetMap import; route math is shared with nav_node."""
import math
from atsd_drive.road_logic import Roads, to_local

HIGHWAYS = {'residential', 'living_street', 'service', 'unclassified', 'tertiary', 'tertiary_link',
            'secondary', 'secondary_link', 'pedestrian', 'footway', 'cycleway'}


def validate(data):
    nodes, edges, directed = data.get('nodes', {}), data.get('edges', []), data.get('directed_edges', [])
    if not isinstance(nodes, dict) or not isinstance(edges, list) or not isinstance(directed, list):
        raise ValueError('Нужны узлы и участки дорог')
    if len(nodes) > 12000 or len(edges)+len(directed) > 24000:
        raise ValueError('Слишком большая карта: загрузите меньшую область')
    for key, point in nodes.items():
        if not isinstance(key, str) or not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError('У каждого узла должны быть две координаты')
        if not all(isinstance(v, (int,float)) and math.isfinite(v) and abs(v)<=100000 for v in point):
            raise ValueError('Недопустимые координаты дороги')
    for edge in edges+directed:
        if not isinstance(edge, (list,tuple)) or len(edge)!=2 or any(k not in nodes for k in edge):
            raise ValueError('Участок должен соединять два существующих узла')
    origin=data.get('origin')
    if origin is not None:
        if not isinstance(origin,(list,tuple)) or len(origin)!=2 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in origin):
            raise ValueError('Укажите широту и долготу центра карты')
        if not (-85<=origin[0]<=85 and -180<=origin[1]<=180):
            raise ValueError('Широта должна быть от −85 до 85°, долгота от −180 до 180°')
    snap=float(data.get('snap_m',3.))
    if not math.isfinite(snap) or not 0.1<=snap<=5.:
        raise ValueError('Допустимая привязка к дороге: 0,1–5 м')
    clean={'nodes':nodes,'edges':edges,'directed_edges':directed,'snap_m':snap,'origin':origin,
           'source':str(data.get('source','manual'))[:100]}
    Roads.from_data(clean)
    return clean


def from_osm(data, origin):
    raw={str(e['id']):e for e in data.get('elements',[]) if e.get('type')=='node'}
    nodes={}; edges=[]; directed=[]
    for way in data.get('elements',[]):
        tags=way.get('tags',{})
        if way.get('type')!='way' or tags.get('highway') not in HIGHWAYS:
            continue
        if tags.get('access') in ('no','private') or tags.get('area')=='yes':
            continue
        ids=[str(n) for n in way.get('nodes',[])]
        oneway=tags.get('oneway', 'yes' if tags.get('junction')=='roundabout' else 'no')
        for a,b in zip(ids,ids[1:]):
            if a not in raw or b not in raw: continue  # missing node never joins across a gap
            for key in (a,b):
                n=raw[key]
                nodes[key]=list(to_local(float(n['lat']),float(n['lon']),origin))
            if oneway=='-1': directed.append([b,a])
            elif oneway in ('yes','1','true'): directed.append([a,b])
            else: edges.append([a,b])
    if not edges and not directed:
        raise ValueError('Дорог в этой области не найдено. Отметьте их поверх карты вручную.')
    return validate({'origin':list(origin),'nodes':nodes,'edges':edges,'directed_edges':directed,
                     'source':'OpenStreetMap contributors (ODbL)'})
