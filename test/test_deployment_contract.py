from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_compose_uses_host_network_and_a2_joy_contract():
    compose = (ROOT / "docker-compose.yml").read_text()

    assert "network_mode: host" in compose
    assert "IMAGE_TOPIC: ${IMAGE_TOPIC:-}" in compose
    assert "JOY_TOPIC: ${JOY_TOPIC:-}" in compose
    assert "CMD_VEL_TOPIC" not in compose


def test_dds_contract_is_read_only_and_entrypoint_fails_when_missing():
    compose = (ROOT / "docker-compose.yml").read_text()
    entrypoint = (ROOT / "docker" / "entrypoint.sh").read_text()
    debug_entrypoint = (ROOT / "docker" / "swin_l_debug_entrypoint.sh").read_text()

    assert "/opt/ros/teamgrit/dds:ro" in compose
    assert "if [[ ! -f" in entrypoint
    assert "teamgrit_dds_env.sh" in entrypoint
    assert "set +u" in entrypoint
    assert "set -u" in entrypoint
    assert "set +u" in debug_entrypoint
    assert "set -u" in debug_entrypoint


def test_runtime_config_is_mounted_read_only():
    compose = (ROOT / "docker-compose.yml").read_text()

    assert "line_tracking.yaml:ro" in compose


def test_yolop_model_mount_and_parameter_are_exposed():
    compose = (ROOT / "docker-compose.yml").read_text()
    launch = (
        ROOT / "ros_ws/src/line_tracking/launch/line_tracking.launch.py"
    ).read_text()

    assert "${MODEL_DIR:-./models}:/models:ro" in compose
    assert "SEGMENTATION_MODEL_PATH" in compose
    assert "SEGMENTATION_MODEL_PATH" in launch


def test_test_mode_topics_and_resolution_profiles_are_exposed():
    compose = (ROOT / "docker-compose.yml").read_text()
    config = (ROOT / "ros_ws/src/line_tracking/config/line_tracking.yaml").read_text()
    launch = (
        ROOT / "ros_ws/src/line_tracking/launch/line_tracking.launch.py"
    ).read_text()

    assert "TEST_MODE" in compose
    assert "test_centerline_topic" in config
    assert "nav_msgs" in (ROOT / "ros_ws/src/line_tracking/package.xml").read_text()
    assert '"360p": (640, 384' in launch
    assert '"720p": (1280, 736' in launch


def test_swin_l_debug_service_is_explicit_and_has_no_drive_contract():
    compose = (ROOT / "docker-compose.yml").read_text()
    entrypoint = (ROOT / "docker" / "swin_l_debug_entrypoint.sh").read_text()

    assert "debugging-swin-l:" in compose
    assert "profiles: [debug]" in compose
    assert "Dockerfile.swin-l-debug" in compose
    assert "runtime: nvidia" in compose
    debug_service = compose.split("debugging-swin-l:", maxsplit=1)[1]
    assert "JOY_TOPIC:" not in debug_service
    assert "/a2_control" not in debug_service
    assert "swin_l_local_path_debug.py ros2" in entrypoint
    assert "/workspace/tools" in entrypoint


def test_jetson_swin_l_base_build_contract():
    compose = (ROOT / "docker-compose.yml").read_text()
    env_example = (ROOT / ".env.example").read_text()
    debug_dockerfile = (ROOT / "Dockerfile.swin-l-debug").read_text()

    assert "SWIN_L_BASE_IMAGE: ${SWIN_L_BASE_IMAGE:-cobiz:jetson-swin-l-l4t-r36.5.0}" in compose
    assert "SWIN_L_TORCH_INDEX_URL" not in compose
    assert "SWIN_L_TORCH_VERSION" not in compose
    assert "SWIN_L_TORCHVISION_INDEX_URL" in compose
    assert "SWIN_L_TORCHVISION_VERSION" in compose
    assert "SWIN_L_BASE_IMAGE=cobiz:jetson-swin-l-l4t-r36.5.0" in env_example
    assert "SWIN_L_TORCH_INDEX_URL" not in env_example
    assert "SWIN_L_TORCH_VERSION" not in env_example
    assert "SWIN_L_TORCHVISION_INDEX_URL=https://pypi.jetson-ai-lab.io/jp6/cu126" in env_example
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
