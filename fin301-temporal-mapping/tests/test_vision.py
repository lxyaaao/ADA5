import numpy as np
from indoor_mapping.vision import verify_images


def test_tiny_patch_in_either_image_cannot_validate_revisit():
    rng=np.random.default_rng(4)
    xy=np.array([(x,y) for x in np.linspace(50,590,8) for y in np.linspace(40,440,8)],np.float32)
    orb=rng.integers(0,256,(len(xy),32),dtype=np.uint8)
    a={'xy':xy,'orb':orb,'image_area':640*480}
    b={'xy':xy*.05+100,'orb':orb.copy(),'image_area':640*480}
    assert not verify_images(a,b)['success']


def test_spatially_distributed_same_place_features_can_pass():
    rng=np.random.default_rng(4)
    xy=np.array([(x,y) for x in np.linspace(50,550,8) for y in np.linspace(40,400,8)],np.float32)
    orb=rng.integers(0,256,(len(xy),32),dtype=np.uint8)
    a={'xy':xy,'orb':orb,'image_area':640*480}
    b={'xy':xy+10,'orb':orb.copy(),'image_area':640*480}
    result=verify_images(a,b)
    assert result['success'] and result['inliers']==64
