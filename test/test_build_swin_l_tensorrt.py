import json
import sys
from pathlib import Path
from types import SimpleNamespace

import torch

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

from build_swin_l_tensorrt import (  # noqa: E402
    PYTORCH_SHAPE_OPS,
    _CompiledStageProxy,
    _FixedStageCall,
    _is_tensorrt_partition_node,
    _rewrite_tensorrt_incompatible_ops,
    _write_artifacts_atomically,
    parse_args,
)


def test_validated_engine_and_manifest_replace_existing_artifacts(tmp_path):
    engine = tmp_path / "model.plan"
    manifest = tmp_path / "model.plan.json"
    engine.write_bytes(b"old-engine")
    manifest.write_text('{"old": true}\n', encoding="utf-8")

    _write_artifacts_atomically(
        b"new-engine",
        {"engine_sha256": "digest"},
        engine,
        manifest,
    )

    assert engine.read_bytes() == b"new-engine"
    assert json.loads(manifest.read_text(encoding="utf-8")) == {
        "engine_sha256": "digest"
    }
    assert not list(tmp_path.glob("*.tmp"))


def test_jetson_builder_defaults_to_least_aggressive_optimization():
    args = parse_args(["--checkpoint", "/checkpoint", "--output", "/model.plan"])

    assert args.optimization_level == 0
    assert args.workspace_mib == 2048
    assert args.min_block_size == 3


def test_hybrid_stage_adapter_keeps_non_tensor_shape_arguments_in_pytorch():
    class Stage(torch.nn.Module):
        def forward(self, hidden, dimensions, head_mask=None, output_attentions=False):
            assert head_mask is None
            assert output_attentions is False
            return hidden + dimensions[0]

    hidden = torch.ones(1, 2)
    captured_args = (hidden, (7, 11), None, False)
    fixed = _FixedStageCall(Stage(), captured_args, {})
    proxy = _CompiledStageProxy(
        fixed,
        tensor_input_count=1,
        has_downsample=True,
    )

    assert torch.equal(fixed(hidden), torch.full((1, 2), 8.0))
    assert torch.equal(proxy(*captured_args), torch.full((1, 2), 8.0))
    assert proxy.downsample is not None


def test_rank_changing_shape_ops_are_forced_to_pytorch():
    forced = {str(op) for op in PYTORCH_SHAPE_OPS}

    assert "aten._reshape_copy.default" in forced
    assert "aten.expand.default" in forced
    assert "aten.repeat.default" in forced
    assert "aten.unsqueeze.default" in forced


def test_tensorrt_partition_detection_supports_compiled_and_exported_graphs():
    compiled = SimpleNamespace(op="call_module", target="_run_on_acc_0")
    exported = SimpleNamespace(
        op="call_function",
        target="tensorrt.execute_engine.default",
    )
    pytorch = SimpleNamespace(op="call_module", target="_run_on_gpu_0")

    assert _is_tensorrt_partition_node(compiled)
    assert _is_tensorrt_partition_node(exported)
    assert not _is_tensorrt_partition_node(pytorch)


def test_tensorrt_graph_rewrite_preserves_mask_and_attention_results():
    graph = torch.fx.Graph()
    mask = graph.placeholder("mask")
    valid = graph.placeholder("valid")
    bias = graph.placeholder("bias")
    batch1 = graph.placeholder("batch1")
    batch2 = graph.placeholder("batch2")
    boolean_product = graph.call_function(
        torch.ops.aten.mul.Tensor,
        args=(mask, valid),
    )
    boolean_product.meta["val"] = torch.empty((1, 2, 2), dtype=torch.bool)
    attention = graph.call_function(
        torch.ops.aten.baddbmm.default,
        args=(bias, batch1, batch2),
    )
    attention.meta["val"] = torch.empty((1, 2, 2), dtype=torch.float32)
    graph.output((boolean_product, attention))
    graph_module = torch.fx.GraphModule({}, graph)

    inputs = (
        torch.tensor([[[True, False], [True, True]]]),
        torch.tensor([[[True], [False]]]),
        torch.randn(1, 2, 2),
        torch.randn(1, 2, 3),
        torch.randn(1, 3, 2),
    )
    expected = graph_module(*inputs)

    rewritten = _rewrite_tensorrt_incompatible_ops(
        SimpleNamespace(graph_module=graph_module)
    )
    actual = graph_module(*inputs)
    targets = {
        node.target for node in graph_module.graph.nodes if node.op == "call_function"
    }

    assert rewritten == {"boolean_mul": 1, "baddbmm": 1}
    assert torch.equal(actual[0], expected[0])
    assert torch.equal(actual[1], expected[1])
    assert torch.ops.aten.mul.Tensor not in targets
    assert torch.ops.aten.baddbmm.default not in targets
    assert torch.ops.aten.bitwise_and.Tensor in targets
    assert torch.ops.aten.bmm.default in targets
    assert torch.ops.aten.add.Tensor in targets
