from pathlib import Path
import numpy as np

from indoor_mapping.geometry import pose_compose, pose_inverse, pose_between


def module():
    assert (Path(__file__).parents[1]/'indoor_mapping/revisit_refinement.py').exists()
    from indoor_mapping import revisit_refinement
    return revisit_refinement


def test_earlier_neighbor_transports_reverse_measurement_to_original_direction():
    m=module()
    a_from_b=np.array([1.4,-.8,2.9]); a_from_neighbor=np.array([.2,.1,.04])
    b_from_neighbor=pose_compose(pose_inverse(a_from_b),a_from_neighbor)
    np.testing.assert_allclose(m.earlier_neighbor_constraint(b_from_neighbor,a_from_neighbor),a_from_b,atol=1e-12)


def test_submaps_use_actual_accepted_indices_without_query_interpolation():
    m=module();keys=np.array([2,9,17,30])
    assert m.submap_members(1,keys,40,'all-heading') == [7,8,9,10,11]
    assert m.submap_members(1,keys,40,'multikeyframe') == [1,2,3,8,9,10,16,17,18]


def test_transporting_a_closure_preserves_nonzero_relative_pose():
    m=module()
    old_target=[2.,3.,.5];old_source=[5.,2.,2.8]
    target=[2.1,3.2,.55];source=[5.2,2.1,2.9]
    old_measurement=pose_between(old_target,old_source)
    actual=m.transport_closure(old_measurement,pose_between(target,old_target),pose_between(old_source,source))
    np.testing.assert_allclose(actual,pose_between(target,source),atol=1e-12)
    assert np.linalg.norm(actual[:2])>1.


def test_full_rotation_proposals_still_require_elapsed_time_and_excursion(monkeypatch):
    m=module()
    def stop_before_registration(source,target,guess,parameters):
        return {'accepted':False,'reason':'proposal_observed','pose':guess}
    monkeypatch.setattr(m,'verify_revisit',stop_before_registration)
    scans=[np.array([[0.,0.],[1.,0.],[0.,1.]]) for _ in range(5)]
    poses=np.array([[0,0,0],[4,0,0],[5,0,0],[4,0,0],[.1,0,np.pi]])
    keys=np.arange(5);times=np.arange(5,dtype=np.int64)*10_000_000_000
    _,events,_,_=m.propose_refinement(scans,poses,keys,times,[])
    assert [(e['i'],e['j']) for e in events]==[(0,4)]
    assert events[0]['previous_yaw_gate_would_exclude']
    _,short_events,_,_=m.propose_refinement(scans,poses,keys,times//2,[])
    assert short_events==[]
    poses[:,:2]=0
    _,no_excursion,_,_=m.propose_refinement(scans,poses,keys,times,[])
    assert no_excursion==[]


def test_existing_nonzero_constraint_is_preserved_once_without_new_evidence():
    m=module();scans=[np.zeros((1,2)) for _ in range(6)]
    poses=np.zeros((6,3));keys=np.arange(6);times=np.arange(6,dtype=np.int64)*1_000_000_000
    existing=[{'i':0,'j':5,'pose':[1.2,-.8,2.1],'loop':True,'sensor':'lidar_revisit'}]
    selected,events,used,_=m.propose_refinement(scans,poses,keys,times,existing)
    assert selected==existing and len(selected)==1 and events==[]
