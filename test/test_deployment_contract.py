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

    assert "/opt/ros/teamgrit/dds:ro" in compose
    assert "if [[ ! -f" in entrypoint
    assert "teamgrit_dds_env.sh" in entrypoint


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
    assert '"360p": (640, 360' in launch
    assert '"720p": (1280, 720' in launch
