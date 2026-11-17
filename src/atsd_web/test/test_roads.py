import json
from pathlib import Path
import subprocess
from unittest.mock import Mock

import pytest
from fastapi.testclient import TestClient
from atsd_drive.road_logic import Roads
from atsd_web import road_map, server
from atsd_web.ros_link import RosLink

GRAPH={'nodes':{'a':[0,0],'b':[10,0],'c':[10,10]},'edges':[['a','b'],['b','c']]}


@pytest.fixture
def web(monkeypatch,tmp_path):
    monkeypatch.setattr(server,'DATA_DIR',tmp_path)
    for attr,name in [('ROADS_FILE','roads.json'),('POINTS_FILE','points.json'),('ORDERS_FILE','orders.json')]:
        monkeypatch.setattr(server,attr,tmp_path/name)
    monkeypatch.setattr(server,'link',RosLink(mock=True))
    return TestClient(server.app)


def save(client,data=GRAPH):
    revision=client.get('/api/config').json()['roads_revision']
    return client.post('/api/roads',json={'roads':data,'revision':revision})


def test_save_preview_restore_and_stale_edit(web):
    revision=web.get('/api/config').json()['roads_revision']
    assert save(web).status_code==200
    r=web.post('/api/route',json={'start':[5,0],'goal':[10,5]})
    assert r.status_code==200 and r.json()['path']==[[5,0],[10,0],[10,5]]
    assert r.json()['distance_m']==10
    assert web.post('/api/roads',json={'roads':GRAPH,'revision':revision}).status_code==409
    assert save(web).status_code==200
    assert list(server.DATA_DIR.glob('roads-backup-*.json'))
    assert Roads.from_data(json.loads(server.ROADS_FILE.read_text())).route((5,0),(10,5))


def test_no_map_never_starts_mission_or_draws_fake_route(web):
    server.link.start_mission=Mock(return_value=True)
    assert web.post('/api/route',json={'start':[0,0],'goal':[10,5]}).status_code==409
    assert web.post('/api/orders',json={'from_id':'kpp','to_id':'depot'}).status_code==409
    server.link.start_mission.assert_not_called()
    assert not server.ORDERS_FILE.exists()


def test_both_legs_validated_and_map_cannot_change_during_mission(web):
    save(web)
    server._save(server.POINTS_FILE,[{'id':'a','name':'A','x':5,'y':0},{'id':'b','name':'B','x':10,'y':5}])
    result=web.post('/api/orders',json={'from_id':'a','to_id':'b'})
    assert result.status_code==200,result.text
    payload=server.link._mock_cmd[-1][1]
    assert payload['routes'][1]==[(5,0),(10,0),(10,5)]
    assert save(web).status_code==409


def test_second_leg_disconnected_does_not_send_start(web):
    save(web,{'nodes':{'a':[0,0],'b':[10,0],'c':[0,10],'d':[10,10]},'edges':[['a','b'],['c','d']]})
    server._save(server.POINTS_FILE,[{'id':'a','name':'A','x':5,'y':0},{'id':'b','name':'B','x':5,'y':10}])
    server.link.start_mission=Mock()
    assert web.post('/api/orders',json={'from_id':'a','to_id':'b'}).status_code==409
    server.link.start_mission.assert_not_called()


def osm():
    return {'elements':[
        {'type':'node','id':1,'lat':53.,'lon':49.},
        {'type':'node','id':2,'lat':53.,'lon':49.001},
        {'type':'node','id':3,'lat':53.001,'lon':49.001},
        {'type':'way','id':10,'nodes':[1,2],'tags':{'highway':'service','oneway':'yes'}},
        {'type':'way','id':11,'nodes':[2,3],'tags':{'highway':'residential'}}]}


def test_osm_geometry_intersections_and_directions():
    graph=road_map.from_osm(osm(),[53.,49.])
    assert graph['nodes']['1']==[0.,0.]
    assert graph['directed_edges']==[['1','2']]
    r=Roads.from_data(graph)
    assert r.route(graph['nodes']['1'],graph['nodes']['3'])
    assert r.route(graph['nodes']['2'],graph['nodes']['1']) is None
    raw=osm();raw['elements'][-2]['tags']['access']='private'
    assert road_map.from_osm(raw,[53.,49.])['directed_edges']==[]


def test_grid_config_and_disabled_geographic_endpoints_preserve_saved_data(web, monkeypatch):
    import urllib.request
    fetch = Mock(side_effect=AssertionError('Network request in local mode'))
    monkeypatch.setattr(urllib.request, 'urlopen', fetch)
    save(web)
    before = server.ROADS_FILE.read_bytes()
    config = web.get('/api/config').json()
    assert config['maps'] == {} and config['grid']['size_m'] == 100
    assert config['grid']['step_m'] == 5
    # Even a cached image from the previous installation must not be served.
    (server.DATA_DIR/'yandex_map.png').write_bytes(b'old map')
    assert web.get('/api/map/map.png').status_code == 410
    assert web.post('/api/roads/import',json={'revision':config['roads_revision']}).status_code == 410
    assert server.ROADS_FILE.read_bytes() == before
    fetch.assert_not_called()


def test_editor_splits_oneway_and_undoes_and_deletes(tmp_path):
    editor=Path(__file__).resolve().parents[1]/'atsd_web/static/road_editor.js'
    js=tmp_path/'test.cjs'
    js.write_text('const RoadEditor=require('+json.dumps(str(editor))+');\n'+'''
const assert=require('node:assert/strict');
const e=new RoadEditor({nodes:{a:[0,0],b:[10,0]},edges:[],directed_edges:[['a','b']]});
e.click(5,.1,.3);
const n=e.selected;
assert.deepEqual(e.data.nodes[n],[5,0]);
assert.deepEqual(e.data.directed_edges,[['a',n],[n,'b']]);
e.click(5,5,.3);assert.equal(e.data.edges.length,1);
e.undo();assert.equal(e.data.edges.length,0);assert.equal(e.selected,n);
e.click(8,0,.3,true);assert.equal(e.data.directed_edges.length,1);
e.undo();assert.equal(e.data.directed_edges.length,2);
''')
    subprocess.run(['node',str(js)],check=True)


def test_grid_is_exactly_100m_and_preserves_metric_coordinates(tmp_path):
    grid = Path(__file__).resolve().parents[1]/'atsd_web/static/grid100.js'
    js = tmp_path/'grid.cjs'
    js.write_text('const Grid=require('+json.dumps(str(grid))+');\n'+"""
const assert=require('node:assert/strict');
for (const center of [[0,0],[-95,15],[500,-250]]) {
  const b=Grid.bounds({center});
  assert.equal(b.x1-b.x0,100); assert.equal(b.y1-b.y0,100);
  for (const [w,h] of [[400,700],[900,450]]) {
    const v=Grid.view({center},w,h);
    assert.equal(v.ox+center[0]*v.s,w/2);
    assert.equal(v.oy-center[1]*v.s,h/2);
    assert.ok(100*v.s <= Math.min(w,h));
    const px=v.ox+(center[0]+12)*v.s;
    assert.ok(Math.abs((px-v.ox)/v.s-(center[0]+12))<1e-9);
  }
}
""")
    subprocess.run(['node',str(js)],check=True)
