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


def generate_launch_description():
    # Bandwidth/stability-first config (per Gemini): the VPU was choking the
    # USB XLink (nn_pt crash + sys_logger_queue spam, no detection data). Slash
    # everything leaving the camera — low fps, low-bandwidth MJPEG, USB2, no
    # diagnostics/passthrough. Spatial YOLO at ~10 fps is plenty for nav.
    import os
    # Allow overriding the workspace location (default is the stock TurtleBot 4 Pi path)
    _ws = os.environ.get("TB4_ROOT", os.path.expanduser("~/Workspace/turtlebot4-glassbox"))
    params = [{
        'camera.i_pipeline_type': 'RGBD',          # stereo+rgb so spatial NN has depth
        'camera.i_nn_type': 'spatial',
        'camera.i_enable_imu': False,
        'camera.i_enable_ir': False,
        'camera.i_usb_speed': 'HIGH',              # force USB2 — stable; rules out USB3 downgrade/brownout
        'camera.i_enable_diagnostics': False,      # kill the sys_logger_queue stream that was X_LINK_ERROR-spamming
        'nn.i_nn_config_path': os.path.join(_ws, "models/nn_yolov5mu.json"),  # YOLOv5mu (v5 medium anchor-free) — fallback: models/nn_yolov8n.json
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
