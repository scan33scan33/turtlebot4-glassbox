#!/usr/bin/env python3
"""
Minimal launch for the OAK-D Lite spatial-YOLO pipeline that bypasses the
turtlebot4_bringup wrapper. Parameters are passed INLINE (not via a YAML file
keyed to a node name), so the driver cannot silently drop them and fall back to
its RGB defaults. output='screen' + emulate_tty puts the C++ driver logs on
stdout (not the ~/.ros/log tree) so we can read "Pipeline type: RGBD" live.
"""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import json
import os
import tempfile

# Which compiled blob to run. The default lives in models/DEFAULT_MODEL so this
# file and the startup preflight in run_oakd.sh / services/tb4-oakd-run.sh cannot
# drift apart. Override to A/B the staged YOLOv8s swap without editing anything
# (see models/YOLOV8S_SWAP.md):
#   TB4_OAKD_MODEL=yolov8s_416_fixed_6shave bash run_oakd.sh
DEFAULT_MODEL_FILE = "models/DEFAULT_MODEL"


def workspace():
    """The checkout this launch file lives in — override with TB4_ROOT."""
    return os.environ.get(
        "TB4_ROOT", os.path.expanduser("~/Workspace/turtlebot4-glassbox"))


def default_model():
    """Blob name (without .blob) to run unless TB4_OAKD_MODEL says otherwise."""
    override = os.environ.get("TB4_OAKD_MODEL")
    if override:
        return override
    path = os.path.join(workspace(), DEFAULT_MODEL_FILE)
    with open(path) as f:
        name = f.read().strip()
    if not name:
        raise RuntimeError("%s is empty" % path)
    return name


def write_nn_config():
    """Materialise the depthai decode config and return its path.

    This used to be three committed JSONs (`nn_yolov5mu.json`, `nn_yolov8n.json`,
    `nn_yolov8s.json`) that were byte-identical apart from `model.model_name` —
    so the 80 COCO labels were stored four times counting `COCO_LABELS` in the
    navigator. Worse, that one differing value was an absolute path hardcoded to
    `/home/ubuntu/Workspace/turtlebot4-glassbox/...`, which defeated the
    `TB4_ROOT` portability every launch script otherwise honours: any clone
    elsewhere pointed depthai at a blob that did not exist.

    Now one committed `models/nn_base.json` holds the shared `nn_config` +
    label mappings, and the `model` block is injected here from wherever the
    blob actually is.
    """
    ws = workspace()
    model = default_model()
    with open(os.path.join(ws, "models", "nn_base.json")) as f:
        cfg = json.load(f)
    blob = os.path.join(ws, "models", model + ".blob")
    cfg["model"] = {"zoo": "path", "model_name": blob}
    out = os.path.join(tempfile.gettempdir(), "tb4_nn_%s.json" % model)
    with open(out, "w") as f:
        json.dump(cfg, f)
    print("[oakd] nn config : %s" % out)
    print("[oakd] nn blob   : %s" % blob)
    return out


def generate_launch_description():
    # Bandwidth/stability-first config (per Gemini): the VPU was choking the
    # USB XLink (nn_pt crash + sys_logger_queue spam, no detection data). Slash
    # everything leaving the camera — low fps, low-bandwidth MJPEG, USB2, no
    # diagnostics/passthrough. Spatial YOLO at ~10 fps is plenty for nav.
    params = [{
        'camera.i_pipeline_type': 'RGBD',          # stereo+rgb so spatial NN has depth
        'camera.i_nn_type': 'spatial',
        'camera.i_enable_imu': False,
        'camera.i_enable_ir': False,
        'camera.i_usb_speed': 'HIGH',              # force USB2 — stable; rules out USB3 downgrade/brownout
        'camera.i_enable_diagnostics': False,      # kill the sys_logger_queue stream that was X_LINK_ERROR-spamming
        'nn.i_nn_config_path': write_nn_config(),  # generated at launch; TB4_OAKD_MODEL picks the blob
        'nn.i_num_inference_threads': 2,   # pipeline 2 inferences (5 shaves x 2 = device's 10)
                                                            # — newer/stronger model than YOLOv4-tiny, same 416 input + bandwidth.
        'nn.i_enable_passthrough': False,          # nn_pt passthrough stream SIGABRTs the pipeline
        'rgb.i_fps': 2.0,
        'rgb.i_low_bandwidth': False,              # OFF: the MJPEG encoder mangled the frame format
                                                   # (VideoEncoder "frame type 14" warning) feeding the NN garbage.
                                                   # USB2 + 10fps + 416 preview is light enough without it.
        'rgb.i_resolution': '720P',
        'rgb.i_preview_size': 416,                 # YOLOv8n 416 input
        'rgb.i_enable_preview': True,
        'rgb.i_publish_topic': True,
        'stereo.i_fps': 2.0,                         # depth is the USB2 bandwidth hog; only feeds per-detection p25 distance, so 2fps is plenty and far more stable
        'stereo.i_low_bandwidth': False,            # depth can stay compressed; it doesn't feed the NN image
    }]
    container = ComposableNodeContainer(
        name='oakd_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depthai_ros_driver',
                plugin='depthai_ros_driver::Camera',
                name='oakd',
                parameters=params,
            ),
        ],
        output='screen',
        emulate_tty=True,
    )
    return LaunchDescription([container])
