#!/usr/bin/env python3
"""Capture the running Gazebo Harmonic model using temporary scene cameras.

Avoids WSLg screenshot limitations. Cameras have no collision/inertia and
are deleted after each capture. This script does not move robot joints.
Run after gazebo.launch.py (unpaused). Requires ros_gz_image and Pillow.
"""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from PIL import Image as PillowImage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage


def ros_image_to_pillow(message):
    """Convert common ROS image encodings without NumPy / OpenCV."""
    encodings = {
        "rgb8": ("RGB", "RGB", 3),
        "bgr8": ("RGB", "BGR", 3),
        "rgba8": ("RGBA", "RGBA", 4),
        "bgra8": ("RGBA", "BGRA", 4),
        "mono8": ("L", "L", 1),
    }
    encoding = message.encoding.lower()
    if encoding not in encodings:
        raise RuntimeError(f"Unsupported camera encoding: {message.encoding}")

    mode, raw_mode, channels = encodings[encoding]
    row_bytes = message.width * channels
    raw = bytes(message.data)
    if message.step != row_bytes:
        raw = b"".join(
            raw[row * message.step:row * message.step + row_bytes]
            for row in range(message.height)
        )

    image = PillowImage.frombytes(
        mode,
        (message.width, message.height),
        raw,
        "raw",
        raw_mode,
    )
    if mode != "RGB":
        image = image.convert("RGB")
    return image


def has_scene_detail(image):
    """Reject a flat background even when its R, G and B values differ."""
    return any(high - low >= 2 for low, high in image.getextrema())

def run_checked(command):
    result = subprocess.run(command, capture_output=True, text=True, timeout=30)
    if result.returncode:
        raise RuntimeError(
            f"Command failed ({result.returncode}): {' '.join(command)}\n"
            f"{result.stdout}{result.stderr}"
        )


def capture(node, output, label, eye, target, fov):
    name = "validation_camera_" + uuid.uuid4().hex[:10]
    dx, dy, dz = [b-a for a, b in zip(eye, target)]
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    yaw = math.atan2(dy, dx)
    topic = f"/{name}/image_raw"
    xml = f'''<sdf version="1.9"><model name="{name}">
      <static>true</static>
      <link name="camera_link"><sensor name="camera" type="camera">
        <always_on>true</always_on><update_rate>5</update_rate><visualize>true</visualize><topic>{topic}</topic>
        <camera><horizontal_fov>{fov}</horizontal_fov>
          <image><width>1440</width><height>1080</height><format>R8G8B8</format></image>
          <clip><near>0.01</near><far>100</far></clip>
        </camera>
      </sensor></link></model></sdf>'''
    latest_frame = [None]
    sub = node.create_subscription(
        RosImage,
        topic,
        lambda message: latest_frame.__setitem__(0, message),
        qos_profile_sensor_data,
    )
    spawned = False
    bridge = None
    try:
        run_checked([
            "ros2", "run", "ros_gz_sim", "create",
            "-world", "default", "-name", name, "-string", xml,
            "-x", str(eye[0]), "-y", str(eye[1]), "-z", str(eye[2]),
            "-R", "0", "-P", str(pitch), "-Y", str(yaw),
        ])
        spawned = True
        bridge = subprocess.Popen(
            ["ros2", "run", "ros_gz_image", "image_bridge", topic],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        deadline = time.monotonic() + 30
        rendered = None
        while rendered is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            if latest_frame[0] is not None:
                candidate = ros_image_to_pillow(latest_frame[0])
                if has_scene_detail(candidate):
                    rendered = candidate
        if rendered is None:
            raise RuntimeError(f"Camera {label} returned only blank frames")
        path = output / f"{label}.png"
        rendered.save(path)
        print(path, flush=True)
    finally:
        if bridge is not None and bridge.poll() is None:
            os.killpg(bridge.pid, signal.SIGINT)
            try:
                bridge.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(bridge.pid, signal.SIGTERM)
        if spawned:
            run_checked([
                "gz", "service", "-s", "/world/default/remove",
                "--reqtype", "gz.msgs.Entity",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "5000",
                "--req", f'name: "{name}", type: 2',
            ])
        node.destroy_subscription(sub)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node("capture_model_validation")
    try:
        capture(node, args.output, "isometric", (1.7, -2.0, 2.65), (0, 0, 1.42), 0.82)
        capture(node, args.output, "side", (0.0, -2.4, 1.42), (0, 0, 1.42), 0.82)
        capture(node, args.output, "arm_detail", (0.75, -0.90, 1.37), (0, 0, 1.36), 0.78)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
