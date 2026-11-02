#!/usr/bin/env python3
"""Exercise the real circuit/readout and check isolation from temporal flow."""
import json
from pathlib import Path
import cv2
import numpy as np
from connectome_flow import FlyvisFlow
from connectome_road import ConnectomeRoadPerception,panorama_mask
from dangjin_sim_server import BUNDLE_SHA256


def main():
    encoder=FlyvisFlow('.cache/flyvis_encoder.pt',BUNDLE_SHA256,'cpu')
    model=ConnectomeRoadPerception(encoder,'.cache/dangjin-connectome-road','mps')
    frames=[cv2.imread(f'.cache/dangjin-connectome-training/{i:03d}-rgb.png') for i in range(4)]
    encoder.advance(frames[0],1.)
    reference=encoder.advance(frames[1],1.5)
    encoder.reset();encoder.advance(frames[0],1.)
    state=encoder.state.nodes.activity.clone();receptors=encoder.last_input.clone()
    selected,hazard,metrics,pano=model.infer_views(frames)
    assert encoder.torch.equal(state,encoder.state.nodes.activity)
    assert encoder.torch.equal(receptors,encoder.last_input) and encoder.last_stamp==1.
    after=encoder.advance(frames[1],1.5)
    difference=float(np.max(np.abs(reference.relative_motion-after.relative_motion)))
    assert difference<1e-6
    assert selected.shape==(270,480) and pano.shape==(384,768,3) and not hazard.any()
    assert metrics['views']==4 and metrics['circuit_ms']>0
    blank=model.infer_views([np.zeros_like(frames[0])]*4)
    assert not blank[0].any(), 'Blank images must not create a drivable surface'
    empty=np.zeros((270,480),np.uint8);full=np.ones_like(empty)
    p=panorama_mask([empty,full,empty,empty])
    assert tuple(p[192,192])==(140,167,94) and tuple(p[192,576])==(50,48,27)
    assert tuple(p[0,192])==(85,85,85)
    report={'schema':'flyvis.road_readout.contract_validation.v1','flow_state_preserved':True,
            'flow_after_detection_max_difference':difference,'blank_camera_road_pixels':int(blank[0].sum()),
            'panorama_azimuth_and_unknown_caps':True,'four_view_inference':metrics,
            'readout_sha256':model.details['weights_sha256'],'encoder_sha256':BUNDLE_SHA256,
            'runtime_ground_truth':False,'raw_image_skip_connection':False}
    Path('environments/dangjin/validation/connectome-road-contracts.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
