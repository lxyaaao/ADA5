from pathlib import Path
import numpy as np


def api():
    assert (Path(__file__).parents[1]/'indoor_mapping/temporal_filter.py').exists()
    from indoor_mapping import temporal_filter
    return temporal_filter


def test_closer_or_invalid_neighbor_rays_never_vote_free_and_wrap_is_correct():
    m=api();scan=np.full(360,10.);points=np.array([[5.,0.],[-5.,0.]])
    hit,free,occ=m.point_visibility(points,scan,-np.pi,2*np.pi/360)
    assert free.all() and not hit.any() and not occ.any()
    scan[181]=3.;scan[359]=3.
    hit,free,occ=m.point_visibility(points,scan,-np.pi,2*np.pi/360)
    assert not free.any() and occ.all()
    scan[:]=10.;scan[180]=np.nan;scan[0]=np.inf
    hit,free,occ=m.point_visibility(points,scan,-np.pi,2*np.pi/360)
    assert not free.any() and not occ.any()
    scan[:]=10.;scan[180]=5.;scan[0]=5.
    hit,free,occ=m.point_visibility(points,scan,-np.pi,2*np.pi/360)
    assert hit.all() and not free.any()


def test_adjacent_scans_are_epoch_votes_and_spans_use_actual_timestamps():
    m=api();ranges=np.full((5,360),10.);ranges[1:,180]=5.
    times=1_700_000_000_000_000_000+np.array([0.,1.9,2.1,4.1,10.1])*1e9
    times=times.astype(np.int64)
    evidence,stats=m.compute_temporal_evidence(np.array([[5.,0.,0.]]),ranges,times,np.arange(5),np.zeros((5,3)),-np.pi,2*np.pi/360)
    assert evidence['hit_epochs'][0]==4
    assert abs(evidence['hit_span_s'][0]-8.2)<1e-6
    assert not evidence['static_supported'][0]  # epoch indices span10s, actual hits only8.2s
    assert evidence['insufficient'][0]


def test_hit_overrides_same_epoch_free_but_later_visible_conflict_is_counted():
    m=api();ranges=np.full((5,360),10.);ranges[0,180]=5.
    times=1_700_000_000_000_000_000+np.array([0,1,4,6,8],np.int64)*1_000_000_000
    evidence,stats=m.compute_temporal_evidence(np.array([[5.,0.,0.]]),ranges,times,np.arange(5),np.zeros((5,3)),-np.pi,2*np.pi/360)
    assert evidence['hit_epochs'][0]==1 and evidence['free_epochs'][0]==3
    assert evidence['raw_free_epochs'][0]==4 and evidence['within_epoch_hit_free_conflicts'][0]==1
    assert evidence['free_span_s'][0]==4.
    assert evidence['conflicted'][0] and not evidence['static_supported'][0]


def test_static_conflict_and_insufficient_are_disjoint_and_unknown_is_kept():
    m=api()
    e={'hit_epochs':np.array([5,2,1,4]),'free_epochs':np.array([0,8,0,1]),
       'hit_span_s':np.array([12.,15.,0.,20.]),'free_span_s':np.array([0.,20.,0.,0.])}
    masks=m.classify_temporal(e)
    np.testing.assert_array_equal(masks['static_supported'],[True,False,False,True])
    np.testing.assert_array_equal(masks['conflicted'],[False,True,False,False])
    np.testing.assert_array_equal(masks['insufficient'],[False,False,True,False])
    assert (~masks['conflicted'])[2]
    assert sum(mask.astype(int) for mask in (masks['static_supported'],masks['conflicted'],masks['insufficient'])).tolist()==[1,1,1,1]
