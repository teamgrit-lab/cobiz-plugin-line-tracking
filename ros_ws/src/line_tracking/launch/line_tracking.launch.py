import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _optional_env(name: str):
    value = os.environ.get(name, "").strip()
    return value or None


def _bool_env(name: str):
    value = _optional_env(name)
    if value is None:
        return None
    return value.lower() in {"1", "true", "yes", "on"}


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("line_tracking")
    params_path = os.path.join(package_share, "config", "line_tracking.yaml")

    overrides = {}
    topic_environment = {
        "image_topic": "IMAGE_TOPIC",
        "debug_image_topic": "DEBUG_IMAGE_TOPIC",
        "mask_topic": "MASK_TOPIC",
        "confidence_topic": "CONFIDENCE_TOPIC",
        "test_debug_image_topic": "TEST_DEBUG_IMAGE_TOPIC",
        "test_road_mask_topic": "TEST_ROAD_MASK_TOPIC",
        "test_raw_line_mask_topic": "TEST_RAW_LINE_MASK_TOPIC",
        "test_line_mask_topic": "TEST_LINE_MASK_TOPIC",
        "test_birdseye_mask_topic": "TEST_BIRDSEYE_MASK_TOPIC",
        "test_centerline_topic": "TEST_CENTERLINE_TOPIC",
        "test_metrics_topic": "TEST_METRICS_TOPIC",
        "centerline_frame_id": "CENTERLINE_FRAME_ID",
        "input_reliability": "INPUT_RELIABILITY",
    }
    for parameter_name, environment_name in topic_environment.items():
        value = _optional_env(environment_name)
        if value is not None:
            overrides[parameter_name] = value

    joy_topic = _optional_env("CONTROL_TOPIC") or _optional_env("JOY_TOPIC")
    if joy_topic is not None:
        overrides["joy_topic"] = joy_topic

    profile = _optional_env("CAMERA_PROFILE")
    profile_defaults = {
        "360p": (640, 360, "/models/yolop-360-640.onnx"),
        "720p": (1280, 720, "/models/yolop-720-1280.onnx"),
    }
    if profile is not None:
        profile = profile.lower()
        if profile not in profile_defaults:
            raise ValueError("CAMERA_PROFILE must be either 360p or 720p")
        overrides["camera_profile"] = profile
        width, height, model_path = profile_defaults[profile]
        overrides["segmentation_input_width"] = int(
            _optional_env("SEGMENTATION_INPUT_WIDTH") or width
        )
        overrides["segmentation_input_height"] = int(
            _optional_env("SEGMENTATION_INPUT_HEIGHT") or height
        )
        overrides["segmentation_model_path"] = (
            _optional_env("SEGMENTATION_MODEL_PATH") or model_path
        )
    else:
        for parameter_name, environment_name in {
            "segmentation_input_width": "SEGMENTATION_INPUT_WIDTH",
            "segmentation_input_height": "SEGMENTATION_INPUT_HEIGHT",
            "segmentation_model_path": "SEGMENTATION_MODEL_PATH",
        }.items():
            value = _optional_env(environment_name)
            if value is not None:
                overrides[parameter_name] = (
                    int(value)
                    if parameter_name != "segmentation_model_path"
                    else value
                )

    for parameter_name, environment_name in {
        "test_mode": "TEST_MODE",
        "publish_control_in_test_mode": "PUBLISH_CONTROL_IN_TEST_MODE",
    }.items():
        value = _bool_env(environment_name)
        if value is not None:
            overrides[parameter_name] = value

    node = Node(
        package="line_tracking",
        executable="line_tracking_node",
        name="line_tracking",
        output="screen",
        parameters=[
            params_path,
            overrides,
        ],
    )
    return LaunchDescription([node])
