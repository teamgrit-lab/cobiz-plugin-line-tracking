from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_compose_uses_host_network_and_direct_sport_contract():
    import yaml

    compose_path = ROOT / "docker-compose.yml"
    compose = compose_path.read_text()
    listener = yaml.safe_load(compose)["services"]["actual-activate"]

    assert "network_mode: host" in compose
    assert "SWIN_L_IMAGE_TOPIC: ${SWIN_L_IMAGE_TOPIC:-" in compose
    assert listener["environment"]["LINE_TRACKING_SPORT_REQUEST_TOPIC"].endswith(
        ":-/api/sport/request}"
    )
    assert "JOY_TOPIC" not in listener["environment"]
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
    assert "LINE_TRACKING_SPORT_REQUEST_TOPIC" not in debug_service["environment"]
    assert (
        debug_service["environment"]["SWIN_L_PROFILE"] == "swin-l-aspect-224x384-fp16"
    )
    assert "SWIN_L_MODE:-ros2" in entrypoint
    assert 'swin_l_local_path_debug.py "${mode}"' in entrypoint
    assert "/workspace/tools" in entrypoint


def test_live_swin_l_services_forward_configured_path_mask_class():
    import yaml

    services = yaml.safe_load((ROOT / "docker-compose.yml").read_text())["services"]
    for name in ("debugging-swin-l", "actual-activate"):
        assert services[name]["environment"]["SWIN_L_PATH_MASK_CLASS"].endswith(":-2}")
    assert "SWIN_L_PATH_MASK_CLASS=2" in (ROOT / ".env.example").read_text()


def test_live_services_forward_path_safety_master_switch():
    import yaml

    services = yaml.safe_load((ROOT / "docker-compose.yml").read_text())["services"]
    for name in ("debugging-swin-l", "actual-activate"):
        assert services[name]["environment"]["SWIN_L_PATH_SAFETY_ENABLED"].endswith(
            ":-true}"
        )
    assert "SWIN_L_PATH_SAFETY_ENABLED=true" in (
        ROOT / ".env.example"
    ).read_text()


def test_default_compose_is_cobiz_task_listener():
    import yaml

    services = yaml.safe_load((ROOT / "docker-compose.yml").read_text())["services"]
    listener = services["actual-activate"]
    assert "profiles" not in listener
    assert listener["environment"]["SWIN_L_MODE"] == "task-drive"
    assert listener["environment"]["SWIN_L_PROFILE"] == "swin-l-aspect-224x384-fp16"
    assert listener["environment"]["SWIN_L_APRILTAG_DETECTIONS_TOPIC"].endswith(
        "/detections}"
    )
    assert listener["environment"]["SWIN_L_APRILTAG_MAX_AGE_SEC"].endswith(":-1.0}")
    assert listener["environment"]["SWIN_L_APRILTAG_CONFIRM_WINDOW_SEC"].endswith(
        ":-1.0}"
    )
    assert listener["environment"]["SWIN_L_APRILTAG_CONFIRM_MIN_HITS"].endswith(
        ":-3}"
    )
    assert listener["environment"]["LINE_TRACKING_TASK_EVENT_TOPIC"].endswith(
        "/task_event}"
    )
    assert listener["environment"]["LINE_TRACKING_TASK_STATE_TOPIC"].endswith(
        "/task_state}"
    )
    assert listener["environment"]["LINE_TRACKING_MAX_FORWARD_MPS"].endswith(
        ":-0.50}"
    )
    assert listener["environment"]["LINE_TRACKING_DEFAULT_DURATION_SEC"].endswith(
        ":-500}"
    )
    assert listener["environment"]["LINE_TRACKING_MAX_DURATION_SEC"].endswith(
        ":-1000}"
    )
    assert "LINE_TRACKING_MAX_FORWARD_MPS=0.50" in (
        ROOT / ".env.example"
    ).read_text()
    assert "LINE_TRACKING_DEFAULT_DURATION_SEC=500" in (
        ROOT / ".env.example"
    ).read_text()
    assert "LINE_TRACKING_MAX_DURATION_SEC=1000" in (
        ROOT / ".env.example"
    ).read_text()


def test_active_deployment_has_no_manual_arm_or_lidar_contract():
    active = "\n".join(
        (ROOT / path).read_text() for path in ("docker-compose.yml", ".env.example")
    )
    for forbidden in (
        "SWIN_L_" + "DRIVE_ENABLED",
        "SWIN_L_" + "CALIBRATION_CONFIRMED",
        "SWIN_L_" + "LIDAR_",
        "SWIN_L_" + "SAFETY_" + "STOP_TOPIC",
        "SWIN_L_" + "CLEARANCE_TOPIC",
    ):
        assert forbidden not in active


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


def test_jetson_image_builds_and_sources_unitree_request_interface():
    dockerfile = (ROOT / "Dockerfile.swin-l-debug").read_text()
    entrypoint = (ROOT / "docker" / "swin_l_debug_entrypoint.sh").read_text()
    package_root = ROOT / "third_party" / "unitree_msgs" / "unitree_api"
    apriltag_root = ROOT / "third_party" / "apriltag_msgs" / "apriltag_msgs"

    assert (ROOT / "third_party" / "unitree_msgs" / "LICENSE").is_file()
    assert (package_root / "package.xml").is_file()
    assert (package_root / "msg" / "Request.msg").read_text().splitlines() == [
        "RequestHeader header",
        "string parameter",
        "uint8[] binary",
    ]
    assert (package_root / "msg" / "RequestIdentity.msg").read_text().splitlines() == [
        "int64 id",
        "int64 api_id",
    ]
    assert "python3-colcon-common-extensions" in dockerfile
    assert "ros-humble-rosidl-default-generators" in dockerfile
    assert "ros-humble-rosidl-generator-dds-idl" in dockerfile
    assert (
        "COPY third_party/unitree_msgs/unitree_api /unitree_ws/src/unitree_api"
        in dockerfile
    )
    assert "colcon build --merge-install --packages-select unitree_api" in dockerfile
    assert "from unitree_api.msg import Request" in dockerfile
    assert 'source "/unitree_ws/install/setup.bash"' in entrypoint
    assert (apriltag_root / "LICENSE").is_file()
    assert "d03bbf21724f35b4304c688793b05c28e98802a0" in (
        ROOT / "third_party" / "apriltag_msgs" / "README.md"
    ).read_text()
    assert (apriltag_root / "msg" / "Point.msg").read_text().splitlines() == [
        "float64 x",
        "float64 y",
    ]
    assert (
        apriltag_root / "msg" / "AprilTagDetectionArray.msg"
    ).read_text().splitlines() == [
        "std_msgs/Header header",
        "AprilTagDetection[] detections",
    ]
    assert (
        "COPY third_party/apriltag_msgs/apriltag_msgs "
        "/unitree_ws/src/apriltag_msgs"
    ) in dockerfile
    assert (
        "colcon build --merge-install --packages-select unitree_api apriltag_msgs"
    ) in dockerfile
    assert "from apriltag_msgs.msg import AprilTagDetectionArray" in dockerfile


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
