"""Fixed-shape CUDA replay of the unchanged inference function.

No compilation, precision conversion or CPU/model fallback. Capture failures
are startup/inference errors. Output storage is reused; callers retain a clone
when keeping temporal state. Each instance belongs to one inference worker.
"""
import torch


class CudaReplay:
    def __init__(self, function, inputs):
        if not inputs or any(x.device.type != 'cuda' for x in inputs.values()):
            raise ValueError('CUDA replay requires CUDA tensor inputs')
        self.function = function
        self.inputs = {k: v.clone() for k, v in inputs.items()}
        stream = torch.cuda.Stream(device=next(iter(inputs.values())).device)
        stream.wait_stream(torch.cuda.current_stream())
        with torch.inference_mode(), torch.cuda.stream(stream):
            for _ in range(3):
                function(self.inputs)
        torch.cuda.current_stream().wait_stream(stream)
        self.graph = torch.cuda.CUDAGraph()
        with torch.inference_mode(), torch.cuda.graph(self.graph):
            self.output = function(self.inputs)

    @torch.inference_mode()
    def __call__(self, inputs):
        if inputs.keys() != self.inputs.keys() or any(
            v.shape != self.inputs[k].shape or v.dtype != self.inputs[k].dtype
            or v.device != self.inputs[k].device for k, v in inputs.items()
        ):
            raise ValueError('CUDA replay input signature changed')
        for k, v in inputs.items():
            self.inputs[k].copy_(v)
        self.graph.replay()
        return self.output
