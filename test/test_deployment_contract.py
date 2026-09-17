from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_compose_uses_host_network_and_a2_joy_contract():
    compose = (ROOT / "docker-compose.yml").read_text()

    assert "network_mode: host" in compose
    assert "SWIN_L_IMAGE_TOPIC: ${SWIN_L_IMAGE_TOPIC:-" in compose
    assert "JOY_TOPIC: ${JOY_TOPIC:-/a2_control}" in compose
    assert "CMD_VEL_TOPIC" not in compose


def test_dds_contract_is_read_only_and_debug_entrypoint_fails_when_missing():
    compose = (ROOT / "docker-compose.yml").read_text()
    debug_entrypoint = (ROOT / "docker" / "swin_l_debug_entrypoint.sh").read_text()

    assert "/opt/ros/teamgrit/dds:ro" in compose
    assert "set +u" in debug_entrypoint
    assert "set -u" in debug_entrypoint


def test_swin_l_debug_service_is_explicit_and_has_no_drive_contract():
    import yaml

    compose = (ROOT / "docker-compose.yml").read_text()
    entrypoint = (ROOT / "docker" / "swin_l_debug_entrypoint.sh").read_text()

    assert "debugging-swin-l:" in compose
    assert "profiles: [debug]" in compose
    assert "Dockerfile.swin-l-debug" in compose
    assert "runtime: nvidia" in compose
    debug_service = yaml.safe_load(compose)["services"]["debugging-swin-l"]
    assert "JOY_TOPIC" not in debug_service["environment"]
    assert debug_service["environment"]["SWIN_L_PROFILE"] == "swin-l-aspect-224x384"
    assert "SWIN_L_MODE:-ros2" in entrypoint
    assert 'swin_l_local_path_debug.py "${mode}"' in entrypoint
    assert "/workspace/tools" in entrypoint


def test_live_swin_l_services_forward_configured_path_mask_class():
    import yaml

    services = yaml.safe_load((ROOT / "docker-compose.yml").read_text())["services"]
    for name in ("debugging-swin-l", "actual-activate"):
        assert services[name]["environment"]["SWIN_L_PATH_MASK_CLASS"].endswith(":-2}")
    assert "SWIN_L_PATH_MASK_CLASS=2" in (ROOT / ".env.example").read_text()


def test_default_compose_is_cobiz_task_listener():
    import yaml

    services = yaml.safe_load((ROOT / "docker-compose.yml").read_text())["services"]
    listener = services["actual-activate"]
    assert "profiles" not in listener
    assert listener["environment"]["SWIN_L_MODE"] == "task-drive"
    assert listener["environment"]["SWIN_L_PROFILE"] == "swin-l-aspect-224x384"
    assert listener["environment"]["SWIN_L_DRIVE_ENABLED"].endswith(":-false}")
    assert listener["environment"]["SWIN_L_CALIBRATION_CONFIRMED"].endswith(":-false}")
    assert listener["environment"]["LINE_TRACKING_TASK_EVENT_TOPIC"].endswith(
        "/task_event}"
    )
    assert listener["environment"]["LINE_TRACKING_TASK_STATE_TOPIC"].endswith(
        "/task_state}"
    )


def test_jetson_swin_l_base_build_contract():
    compose = (ROOT / "docker-compose.yml").read_text()
    env_example = (ROOT / ".env.example").read_text()
    debug_dockerfile = (ROOT / "Dockerfile.swin-l-debug").read_text()

    assert (
        "SWIN_L_BASE_IMAGE: ${SWIN_L_BASE_IMAGE:-cobiz:jetson-swin-l-l4t-r36.5.0}"
        in compose
    )
    assert "SWIN_L_TORCH_INDEX_URL" not in compose
    assert "SWIN_L_TORCH_VERSION" not in compose
    assert "SWIN_L_TORCHVISION_INDEX_URL" in compose
    assert "SWIN_L_TORCHVISION_VERSION" in compose
    assert "SWIN_L_BASE_IMAGE=cobiz:jetson-swin-l-l4t-r36.5.0" in env_example
    assert "SWIN_L_TORCH_INDEX_URL" not in env_example
    assert "SWIN_L_TORCH_VERSION" not in env_example
    assert (
        "SWIN_L_TORCHVISION_INDEX_URL=https://pypi.jetson-ai-lab.io/jp6/cu126"
        in env_example
    )
    assert "SWIN_L_TORCHVISION_VERSION=0.23.0" in env_example
    assert "ARG SWIN_L_BASE_IMAGE=cobiz:jetson-swin-l-l4t-r36.5.0" in debug_dockerfile
    assert "pip install" in debug_dockerfile
    assert "torchvision" in debug_dockerfile
    assert '"torchvision==${SWIN_L_TORCHVISION_VERSION}"' in debug_dockerfile
    assert "--no-deps" in debug_dockerfile
    assert "torch.version.cuda" in debug_dockerfile
    assert "12.6" in debug_dockerfile
    assert "ros-humble-cv-bridge" in debug_dockerfile
    assert "ros-humble-rmw-cyclonedds-cpp" in debug_dockerfile
    assert '"transformers==5.16.1"' in debug_dockerfile


def test_offline_swin_l_service_reuses_cuda_without_starting_ros():
    import yaml

    services = yaml.safe_load((ROOT / "docker-compose.yml").read_text())["services"]
    offline = services["test-swin-l"]
    live = services["debugging-swin-l"]

    assert offline["build"] == live["build"]
    assert offline["image"] == live["image"]
    assert offline["profiles"] == ["test"]
    assert offline["runtime"] == "nvidia"
    assert offline["entrypoint"] == [
        "/opt/venv/bin/python",
        "/workspace/tools/swin_l_rosbag_overlay.py",
    ]
    assert offline["command"] == ["--help"]
    assert "depends_on" not in offline
    assert "network_mode" not in offline
    assert "restart" not in offline
    assert "SWIN_L_TEST_UID" in offline["user"]
    assert "SWIN_L_TEST_GID" in offline["user"]
    mounts = {
        mount.rsplit(":", 2)[-2]: mount
        for mount in offline["volumes"]
        if mount.endswith(":ro")
    }
    assert "/bags" in mounts
    assert "/workspace/.env" in mounts
    assert all("teamgrit/dds" not in mount for mount in offline["volumes"])
    assert any(
        mount.endswith(":/workspace/rosbag-results/swin-l-tests")
        for mount in offline["volumes"]
    )
    assert any(
        mount.endswith(":" + offline["environment"]["HF_HOME"])
        for mount in offline["volumes"]
    )
