import math
import pytest
from atsd_drive.road_logic import Roads


def test_empty_disconnected_and_far_points_do_not_fall_back_to_direct():
    assert Roads().route((0,0),(5,0)) is None
    r=Roads({'a':(0,0),'b':(10,0),'c':(0,10),'d':(10,10)},[['a','b'],['c','d']])
    assert r.route((0,0),(0,10)) is None
    assert r.route((0,0),(0,100)) is None


def test_same_segment_both_directions_and_exact_endpoints():
    r=Roads({'a':(0,0),'b':(100,0)},[['a','b']])
    assert r.route((20,0),(70,0))==[(20,0),(70,0)]
    assert r.route((70,0),(20,0))==[(70,0),(20,0)]
    assert r.route((0,0),(100,0))==[(0,0),(100,0)]
    assert r.route((20,.5),(70,-.5))==[(20,.5),(20,0),(70,0),(70,-.5)]
    assert r.route((20,0),(20,0))==[(20,0),(20,0)]


def test_bend_is_not_skipped_when_goal_nearest_node_is_start():
    r=Roads({'a':(0,0),'b':(10,0),'c':(10,10)},[['a','b'],['b','c']])
    assert r.route((6,0),(10,4))==[(6,0),(10,0),(10,4)]


def test_oneway_projection_and_detour_in_reverse_direction():
    nodes={'a':(0,0),'b':(10,0),'c':(10,10),'d':(0,10)}
    r=Roads(nodes,[],directed_edges=[['a','b'],['b','c'],['c','d'],['d','a']])
    assert r.route((2,0),(8,0))==[(2,0),(8,0)]
    path=r.route((8,0),(2,0))
    assert path==[(8,0),(10,0),(10,10),(0,10),(0,0),(2,0)]
    assert Roads(nodes,[],directed_edges=[['a','b']]).route((8,0),(2,0)) is None


def test_crossing_geometry_does_not_connect_overpass():
    r=Roads({'a':(-10,0),'b':(10,0),'c':(0,-10),'d':(0,10)},[['a','b'],['c','d']])
    assert r.route((-8,0),(0,8)) is None


def test_saved_graph_loader_and_invalid_graph():
    r=Roads.from_data({'nodes':{'a':[0,0],'b':[10,0]},'edges':[['a','b']]})
    assert r.route((0,0),(10,0))==[(0,0),(10,0)]
    for data in ({'nodes':{'a':[math.nan,0]}},{'edges':[['a','b']]},{'snap_m':100},[]):
        with pytest.raises((ValueError,TypeError)): Roads.from_data(data)
