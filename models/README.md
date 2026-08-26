# Segmentation model

The plugin is wired for the official **YOLOP** ONNX export. YOLOP uses one
shared network to emit both a drivable-area segmentation and a lane-line
segmentation, which is the required “line inside the road” pipeline. The
runtime reads the `drive_area_seg` and `lane_line_seg` outputs with OpenCV DNN,
then gates lane pixels by a dilated drivable-area mask before path fitting.
Test mode exposes both the raw lane mask and the road-gated lane mask.

The camera stream and model input are configured as a matched profile:

| profile | camera frame / model input | expected ONNX filename |
|---|---:|---|
| `360p` | 640x360 / 640x384 | `yolop-360-640.onnx` |
| `720p` | 1280x720 / 1280x736 | `yolop-720-1280.onnx` |

YOLOP's skip connections require model dimensions divisible by 32. The runtime
keeps the camera frame at 16:9 and adds/removes the small bottom letterbox
padding around inference. The files in this directory were generated from the
official pretrained weights with the upstream exporter:

```bash
PYTHONPATH=. python3 export_onnx.py --height 384 --width 640
cp weights/yolop-384-640.onnx /path/to/cobiz-plugin-line-tracking/models/yolop-360-640.onnx

PYTHONPATH=. python3 export_onnx.py --height 736 --width 1280
cp weights/yolop-736-1280.onnx /path/to/cobiz-plugin-line-tracking/models/yolop-720-1280.onnx

cd /path/to/cobiz-plugin-line-tracking
cp .env.example .env
docker compose up -d --build
```

`360p` is the default profile for lower CPU latency. `720p` retains more
image detail but costs substantially more inference time and memory. The
official pretrained YOLOP release is documented around 640x640; these
16:9 exports use the same pretrained weights and should still be validated on
the target camera before field operation.

YOLOP is trained on BDD100K road scenes. For the A2 camera, retrain or
fine-tune its two segmentation heads with local images labeled for the actual
road, sidewalk/grass, and guide line. A pretrained model is not a guarantee of
accuracy in the user's field environment; the ONNX interface and class/output
contract must remain unchanged after export.
