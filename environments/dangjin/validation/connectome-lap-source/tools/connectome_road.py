"""Road readout of frozen Flyvis visual-circuit activity, without an RGB shortcut.

The optical-flow-pretrained connectome is not a pretrained road classifier.
Only the convolutional road readout is trained on offline synthetic labels.
Detection receives camera pixels only. Map/pose guidance belongs to the planner.
"""
from pathlib import Path
import hashlib
import json
import time

import cv2
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F


class CircuitFeatures:
    height, width = 48, 88
    steps, dt = 10, .02

    def __init__(self, encoder):
        from scipy.spatial import Delaunay, cKDTree
        self.encoder=encoder
        size=encoder.size
        scale=min(size/480,size/270)
        ox,oy=(size-round(480*scale))//2,(size-round(270*scale))//2
        centers=encoder.eye.receptor_centers.cpu().numpy()[:,::-1].astype(float)
        pixels=(centers+size//2-[ox,oy])/scale
        yy,xx=np.mgrid[:self.height,:self.width]
        query=np.stack(((xx.ravel()+.5)*480/self.width-.5,(yy.ravel()+.5)*270/self.height-.5),axis=1)
        tri=Delaunay(pixels);simplex=tri.find_simplex(query);inside=simplex>=0
        self.indices=tri.simplices[np.maximum(simplex,0)].copy()
        transform=tri.transform[np.maximum(simplex,0)]
        bary=np.einsum('nij,nj->ni',transform[:,:2],query-transform[:,2])
        self.weights=np.column_stack((bary,1-bary.sum(axis=1))).astype(np.float32)
        # Outside the hexagonal eye's hull use its nearest measured activity;
        # this is interpolation of neurons, never an image or map shortcut.
        nearest=cKDTree(pixels).query(query[~inside])[1]
        self.indices[~inside]=nearest[:,None];self.weights[~inside]=[1,0,0]
        self.cell_types=[str(t) for t in encoder.network.connectome.output_cell_types[:].astype(str)]

    def __call__(self, frames):
        if not frames or any(f.shape!=(270,480,3) for f in frames):
            raise ValueError('480x270 RGB camera projections required')
        e=self.encoder;canvases=[]
        scale=min(e.size/480,e.size/270);w,h=round(480*scale),round(270*scale)
        ox,oy=(e.size-w)//2,(e.size-h)//2
        for frame in frames:
            gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY).astype(np.float32)/255
            canvas=np.full((e.size,e.size),.5,np.float32)
            canvas[oy:oy+h,ox:ox+w]=cv2.resize(gray,(w,h));canvases.append(canvas)
        with torch.inference_mode():
            receptors=e.eye(torch.as_tensor(np.asarray(canvases),device=e.device)[:,None])
            D=type(e.grey)
            baseline=e.network._state_api(D(nodes=D(activity=e.grey.nodes.activity.expand(len(frames),-1).clone()),edges=D()))
            e.network.stimulus.zero(len(frames),self.steps)
            e.network.stimulus.add_input(receptors.expand(-1,self.steps,-1,-1))
            states=e.network(e.network.stimulus(),dt=self.dt,state=baseline,as_states=True)
            activity=states[-1].nodes.activity[:,None,:]  # LayerActivity stores a weak reference.
            e.decoder.dvs_channels.update(activity)
            channels=F.relu(e.decoder.dvs_channels.output[:,0]).cpu().numpy()
        if channels.shape[1:]!=(34,721) or not np.isfinite(channels).all():
            raise ValueError('invalid connectome output activity')
        # Leave e.state / e.last_input / e.last_stamp untouched: flow tracking
        # and fixed-duration visual detection have separate recurrent states.
        grid=(channels[:,:,self.indices]*self.weights[None,None]).sum(axis=-1)
        return grid.reshape(len(frames),34,self.height,self.width)


def block(in_channels,out_channels):
    return nn.Sequential(nn.Conv2d(in_channels,out_channels,3,padding=1),nn.GroupNorm(8,out_channels),nn.SiLU(),
                         nn.Conv2d(out_channels,out_channels,3,padding=1),nn.GroupNorm(8,out_channels),nn.SiLU())


class RoadReadout(nn.Module):
    def __init__(self):
        super().__init__()
        self.enc1=block(34,32);self.enc2=block(32,64);self.center=block(64,96)
        self.dec2=block(160,64);self.dec1=block(96,32);self.head=nn.Conv2d(32,1,1)
        self.register_buffer('mean',torch.zeros(1,34,1,1))
        self.register_buffer('std',torch.ones(1,34,1,1))

    def forward(self,activity):
        x=(activity-self.mean)/self.std
        a=self.enc1(x);b=self.enc2(F.avg_pool2d(a,2));c=self.center(F.avg_pool2d(b,2))
        d=self.dec2(torch.cat((F.interpolate(c,size=b.shape[-2:],mode='bilinear',align_corners=False),b),1))
        d=self.dec1(torch.cat((F.interpolate(d,size=a.shape[-2:],mode='bilinear',align_corners=False),a),1))
        return self.head(d)


def panorama_mask(masks,width=768,height=384):
    """Four ±45° vertical views cover every azimuth; unseen polar caps stay unknown."""
    if len(masks)!=4 or any(m.shape!=(270,480) for m in masks):raise ValueError('four aligned masks required')
    yy,xx=np.mgrid[:height,:width]
    lon=((xx+.5)/width-.5)*2*np.pi
    lat=(.5-(yy+.5)/height)*np.pi
    # Panorama longitude increases right; positive robot heading turns left.
    heading=-lon
    sector=np.mod(np.rint(heading/(np.pi/2)).astype(int),4)
    delta=(heading-sector*np.pi/2+np.pi)%(2*np.pi)-np.pi
    u=239.5-135*np.tan(delta)
    v=134.5-135*np.tan(lat)/np.cos(delta)
    visible=(u>=0)&(u<480)&(v>=0)&(v<270)
    output=np.full((height,width,3),(50,48,27),np.uint8)
    output[~visible]=(85,85,85)
    for i,mask in enumerate(masks):
        sampled=cv2.remap(mask,u.astype(np.float32),v.astype(np.float32),cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT)
        output[visible&(sector==i)&(sampled!=0)]=(140,167,94)
    return output


class ConnectomeRoadPerception:
    def __init__(self,encoder,path,device='mps'):
        path=Path(path);checkpoint=torch.load(path/'road-readout.pt',map_location='cpu',weights_only=True)
        if checkpoint['encoder_sha256']!=encoder.details['bundle_sha256']:
            raise ValueError('road readout was trained with another connectome')
        self.device=torch.device(device);self.features=CircuitFeatures(encoder)
        self.model=RoadReadout().to(self.device).eval().requires_grad_(False)
        self.model.load_state_dict(checkpoint['state_dict'],strict=True)
        self.threshold=checkpoint['threshold']
        self.details={'backend':'flyvis-road-readout','input':'four RGB camera projections; grayscale BoxEye -> frozen Flyvis neurons -> trained road readout',
            'device':device,'weights_sha256':hashlib.sha256((path/'road-readout.pt').read_bytes()).hexdigest(),
            'encoder_sha256':encoder.details['bundle_sha256'],'encoder_frozen':True,'road_readout_trained':True,
            'raw_image_skip_connection':False,'uses_ground_truth_at_runtime':False,'uses_pose_or_map':False,
            'azimuth_coverage_deg':360,'vertical_view_fov_deg':90,'views':4,'output_cell_types':self.features.cell_types,
            'presentation_steps':self.features.steps,'integration_dt_s':self.features.dt,
            'training':json.loads((path/'simulation-training.json').read_text())}

    def infer_views(self,frames):
        if len(frames)!=4:raise ValueError('all four 360-camera views required')
        start=time.perf_counter();features=self.features(frames);circuit_end=time.perf_counter()
        with torch.inference_mode():
            logits=self.model(torch.as_tensor(features,device=self.device))
            probability=F.interpolate(logits,size=(270,480),mode='bilinear',align_corners=False).sigmoid()[:,0].cpu().numpy()
        masks=(probability>=self.threshold).astype(np.uint8)
        metrics={'ms':(time.perf_counter()-start)*1000,'circuit_ms':(circuit_end-start)*1000,
                 'road_fraction':float(masks[0].mean()),'views':4,
                 'view_road_fractions':[float(m.mean()) for m in masks],
                 'mean_road_probability':float(probability[0][masks[0]!=0].mean()) if masks[0].any() else 0.}
        return masks[0],np.zeros_like(masks[0]),metrics,panorama_mask(masks)
