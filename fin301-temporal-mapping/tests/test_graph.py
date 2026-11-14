import numpy as np
from indoor_mapping.graph import optimize_graph, nearest_timestamps


def test_nearest_timestamps_keeps_nanosecond_precision_and_rejects_gap():
    base = 1792708234107287354
    target = np.array([base, base+100_000_000, base+200_000_000], dtype=np.int64)
    ids, delta, valid = nearest_timestamps(target, np.array([base+80_000_000,base+500_000_000]), 50_000_000)
    assert ids.tolist() == [1,2]
    assert delta.tolist() == [20_000_000,300_000_000]
    assert valid.tolist() == [True,False]


def test_loop_constraint_reduces_drift_without_moving_anchor():
    poses=np.array([[0.,0.,0.],[1.05,0,0],[2.1,0,0],[1.15,0,0],[.2,0,0]])
    edges=[{'i':i,'j':i+1,'pose':(poses[i+1]-poses[i]).tolist(),'loop':False} for i in range(4)]
    edges.append({'i':0,'j':4,'pose':[0,0,0],'loop':True})
    corrected,stats=optimize_graph(poses,edges)
    np.testing.assert_allclose(corrected[0],poses[0])
    assert abs(corrected[-1,0]) < abs(poses[-1,0])*.7
    assert stats['success']


def test_rotated_square_graph_recovers_metric_pose_constraints():
    from indoor_mapping.geometry import pose_between
    truth=np.array([[0.,0.,0.],[1,0,np.pi/2],[1,1,np.pi],[0,1,-np.pi/2],[0,0,0.]])
    initial=truth.copy();initial[1:,:2]+=[.1,-.07];initial[1:,2]+=.08
    edges=[{'i':i,'j':i+1,'pose':pose_between(truth[i],truth[i+1]),'loop':False} for i in range(4)]
    edges.append({'i':0,'j':4,'pose':[0,0,0],'loop':True})
    corrected,stats=optimize_graph(initial,edges)
    assert stats['success']
    np.testing.assert_allclose(corrected[:,:2],truth[:,:2],atol=1e-4)
    np.testing.assert_allclose(np.sin(corrected[:,2]-truth[:,2]),0,atol=1e-4)
