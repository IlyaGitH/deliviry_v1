from atsd_perception.sign_passage import SignPassage


def det(kind='STOP',x=0,size=100):
    return dict(kind=kind,box=[x,0,x+size,size])


def test_confirm_and_disappear_six_distinct_processed_frames():
    t=SignPassage()
    for i in range(6): assert t.update([det()],i*.2)==[]
    for i in range(5): assert t.update([],1.2+i*.2)==[]
    assert t.update([],2.2)==['STOP']
    assert t.update([],2.4)==[]


def test_flash_and_temporary_occlusion_do_not_trigger():
    t=SignPassage()
    for i in range(5): t.update([det()],i*.2)
    for i in range(6): assert t.update([],1+i*.2)==[]
    for i in range(6): t.update([det()],3+i*.2)
    for i in range(5): assert t.update([],4.2+i*.2)==[]
    assert t.update([det()],5.2)==[]
    for i in range(5): assert t.update([],5.4+i*.2)==[]
    assert t.update([],6.4)==['STOP']


def test_other_sign_and_traffic_light_do_not_mask_disappearance():
    t=SignPassage()
    for i in range(6): t.update([det()],i*.2)
    for i in range(5): assert t.update([det('BUMP',200),det('CROSS',400),det('LIGHT',600)],1.2+i*.2)==[]
    assert t.update([det('BUMP',200)],2.2)==['STOP']
    for i in range(5): assert t.update([],2.4+i*.2)==[]
    assert t.update([],3.4)==['BUMP']


def test_camera_gap_is_not_six_missing_frames():
    t=SignPassage()
    for i in range(6): t.update([det()],i*.2)
    for i in range(5): t.update([],1.2+i*.2)
    assert t.update([],10)==[]
