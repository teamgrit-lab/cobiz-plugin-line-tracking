"""Four-view tensor preprocessing shared by Jetson Road inference services.

Projection maps are prepared once from the existing camera contracts. Live
pixels stay on CUDA from the decoded panorama upload to model inference.
The explicit CPU device is useful only for offline numerical parity checks.
"""
import numpy as np
import torch
from torch.nn import functional as F

from surround_camera import load_profile, projection_maps


class SurroundTensorProjector:
    def __init__(self, kind, *, input_width=1024, device='cuda'):
        self.device = torch.device(device)
        if self.device.type == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('CUDA preprocessing unavailable; no CPU fallback')
        if kind == 'flyvis':
            maps = projection_maps(load_profile()['alignment']['forward_heading_deg'])
        elif kind == 'fcn':
            from fcn360_road import camera_maps
            # OpenCV INTER_LINEAR uses 1/32-pixel interpolation tables. Retain
            # that sampling convention for the existing FCN checkpoints.
            maps = tuple((np.rint(x*32)/32, np.rint(y*32)/32) for x, y in camera_maps(input_width))
        else:
            raise ValueError('Expected flyvis or fcn camera projection')
        grids = []
        for x, y in maps:
            # One halo pixel at either horizontal edge preserves longitude wrap.
            u = ((x % 2880 + 1 + .5) / 2882 * 2 - 1).astype(np.float32)
            v = ((y.clip(0, 1439) + .5) / 1440 * 2 - 1).astype(np.float32)
            grids.append(np.stack((u, v), -1))
        self.grid = torch.from_numpy(np.stack(grids)).to(self.device)
        self.mean = torch.tensor([.485, .456, .406], device=self.device)[None, :, None, None]
        self.std = torch.tensor([.229, .224, .225], device=self.device)[None, :, None, None]

    @torch.inference_mode()
    def project(self, panorama):
        if (not isinstance(panorama, torch.Tensor) or panorama.dtype != torch.uint8
                or tuple(panorama.shape) not in ((1440, 2880, 3), (1440, 2880, 4))
                or panorama.device != self.grid.device):
            raise ValueError('Expected a device-resident 2880x1440 uint8 BGR/BGRx panorama')
        source = panorama[..., :3].permute(2, 0, 1).to(torch.float32)[None]
        source = torch.cat((source[..., -1:], source, source[..., :1]), -1)
        views = F.grid_sample(source.expand(4, -1, -1, -1), self.grid,
                              mode='bilinear', padding_mode='border', align_corners=False)
        return views.add_(.5).floor_().clamp_(0, 255).to(torch.uint8)

    @torch.inference_mode()
    def fcn_input(self, panorama):
        bgr = self.project(panorama)
        rgb = bgr[:, [2, 1, 0]].to(torch.float32).div_(255)
        return rgb.sub_(self.mean).div_(self.std)


def eye_canvases(views, size):
    """BGR uint8 NCHW -> letterboxed grayscale [0,1], matching BoxEye input."""
    if views.dtype != torch.uint8 or views.ndim != 4 or tuple(views.shape[1:]) != (3, 270, 480):
        raise ValueError('Expected 480x270 BGR tensor views')
    bgr = views.to(torch.int32)
    # OpenCV's uint8 BGR2GRAY fixed-point coefficients and rounding.
    gray = ((bgr[:, 0]*3735 + bgr[:, 1]*19235 + bgr[:, 2]*9798 + 16384) >> 15).float().div_(255)
    scale = min(size/480, size/270)
    w, h = round(480*scale), round(270*scale)
    ox, oy = (size-w)//2, (size-h)//2
    canvas = torch.full((len(views), 1, size, size), .5, device=views.device)
    canvas[:, :, oy:oy+h, ox:ox+w] = F.interpolate(gray[:, None], size=(h, w),
                                                   mode='bilinear', align_corners=False)
    return canvas


def area_weights(source, target, device):
    """Exact pixel-overlap AREA weights, including non-integer downscales."""
    edges = torch.arange(target+1, dtype=torch.float32, device=device) * (source/target)
    pixels = torch.arange(source, dtype=torch.float32, device=device)
    overlap = (torch.minimum(edges[1:, None], pixels+1)
               - torch.maximum(edges[:-1, None], pixels)).clamp_min_(0)
    return overlap / (source/target)
