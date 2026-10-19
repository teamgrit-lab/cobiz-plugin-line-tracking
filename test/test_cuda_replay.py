"""Native inference capture must preserve values and reject changed inputs."""
import sys
from pathlib import Path
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from cuda_replay import CudaReplay
from best_so_far_runtime import BestSoFarConfig, R50_PROFILE


def test_capture_rejects_cpu_and_unsupported_model_without_fallback():
    with pytest.raises(ValueError, match='CUDA tensor'):
        CudaReplay(lambda x: x['x']*2, {'x': torch.ones(2, device='cpu')})
    with pytest.raises(ValueError, match='PyTorch R50'):
        BestSoFarConfig(cuda_graph=True).validate()
    BestSoFarConfig(profile=R50_PROFILE, cuda_graph=True).validate()


@pytest.mark.skipif(not torch.cuda.is_available(), reason='CUDA hardware required')
def test_cuda_replay_matches_native_for_new_values_and_rejects_new_shape():
    def function(inputs):
        return (inputs['x'].sin() @ inputs['x'].T).softmax(-1)
    initial = {'x': torch.randn(12, 8, device='cuda')}
    replay = CudaReplay(function, initial)
    for _ in range(4):
        inputs = {'x': torch.randn_like(initial['x'])}
        torch.testing.assert_close(replay(inputs), function(inputs))
    with pytest.raises(ValueError, match='signature'):
        replay({'x': torch.ones(8, 8, device='cuda')})
