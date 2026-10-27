"""RGB-only road inference for the local simulator; no world pose or mesh input."""
from pathlib import Path
import hashlib
import json
import time
import cv2
import numpy as np

MODEL_ID='nvidia/segformer-b0-finetuned-cityscapes-640-1280'
REVISION='618918f3e955c8c4364d73cdbd403a40282b98b9'


class RoadPerception:
    def __init__(self,path,device='mps'):
        import torch
        from transformers import SegformerForSemanticSegmentation
        self.torch=torch;self.device=torch.device(device)
        self.model=SegformerForSemanticSegmentation.from_pretrained(str(path),local_files_only=True).to(self.device).eval()
        self.mean=torch.tensor([.485,.456,.406],device=self.device)[None,:,None,None]
        self.std=torch.tensor([.229,.224,.225],device=self.device)[None,:,None,None]
        weight=next(p for p in (Path(path)/'model.safetensors',Path(path)/'pytorch_model.bin') if p.exists())
        manifest=Path(path)/'simulation-training.json'
        self.details={'backend':'segformer-b0','input':'RGB only','model_id':MODEL_ID,'revision':REVISION,
                      'device':device,'weights_sha256':hashlib.sha256(weight.read_bytes()).hexdigest(),
                      'simulation_finetuned':manifest.exists(),'uses_ground_truth_at_runtime':False}
        if manifest.exists():self.details['training']=json.loads(manifest.read_text())

    def tensor(self,bgr):
        t=self.torch;rgb=cv2.cvtColor(cv2.resize(bgr,(448,256)),cv2.COLOR_BGR2RGB)
        x=t.from_numpy(rgb.copy()).to(self.device,dtype=t.float32).permute(2,0,1)[None]/255
        return (x-self.mean)/self.std

    def infer(self,bgr):
        t=self.torch;started=time.perf_counter()
        with t.inference_mode():
            logits=self.model(pixel_values=self.tensor(bgr)).logits
            probability=t.nn.functional.interpolate(logits,size=bgr.shape[:2],mode='bilinear',align_corners=False).softmax(1)[0,0].cpu().numpy()
        selected=(probability>=.55).astype(np.uint8)
        # A model prediction is deliberately not filled with map-derived road.
        selected[:bgr.shape[0]//2]=0
        hazard=np.zeros_like(selected)
        return selected,hazard,{'ms':(time.perf_counter()-started)*1000,'road_fraction':float(selected.mean()),
                                'mean_road_probability':float(probability[selected!=0].mean()) if selected.any() else 0.}
