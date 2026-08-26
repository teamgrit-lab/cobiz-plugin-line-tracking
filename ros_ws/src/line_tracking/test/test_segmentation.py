import numpy as np

from line_tracking.segmentation import YolopConfig, YolopSegmenter


class _FakeNet:
    def __init__(self, outputs):
        self.outputs = outputs
        self.input = None

    def setInput(self, blob, name):
        self.input = (blob, name)

    def forward(self, names):
        assert names == ["det_out", "drive_area_seg", "lane_line_seg"]
        return self.outputs


def _fake_segmenter():
    segmenter = object.__new__(YolopSegmenter)
    segmenter.config = YolopConfig(
        model_path="unused.onnx",
        input_width=32,
        input_height=32,
        road_gate_kernel=3,
    )
    road = np.zeros((1, 2, 8, 8), dtype=np.float32)
    line = np.zeros((1, 2, 8, 8), dtype=np.float32)
    road[:, 1, 2:6, 2:6] = 1.0
    line[:, 1, 3:5, 3:5] = 1.0
    segmenter._output_names = ["det_out", "drive_area_seg", "lane_line_seg"]
    segmenter._net = _FakeNet(
        [np.zeros((1, 1, 6), dtype=np.float32), road, line]
    )
    return segmenter


def test_yolop_uses_one_forward_pass_for_road_and_line_masks():
    segmenter = _fake_segmenter()

    result = segmenter.segment(np.zeros((20, 40, 3), dtype=np.uint8))

    assert result.road_mask.shape == (20, 40)
    assert result.line_mask.shape == (20, 40)
    assert result.raw_line_mask.shape == (20, 40)
    assert np.count_nonzero(result.road_mask) > 0
    assert np.count_nonzero(result.line_mask) > 0
    assert np.count_nonzero(result.raw_line_mask) >= np.count_nonzero(result.line_mask)
    assert segmenter._net.input[1] == "images"


def test_yolop_rejects_even_road_gate_kernel():
    config = YolopConfig(model_path="unused.onnx", road_gate_kernel=4)

    try:
        config.validate()
    except ValueError as error:
        assert "odd" in str(error)
    else:
        raise AssertionError("expected invalid morphology kernel to be rejected")
