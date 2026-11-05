"""Keep developer/robot .env values out of deterministic unit scenarios."""

import sys

import pytest


@pytest.fixture(autouse=True)
def isolated_runtime_environment(monkeypatch):
    module = sys.modules.get("line_tracking_runtime")
    if module is not None:
        monkeypatch.setattr(module, "ENV", {})
