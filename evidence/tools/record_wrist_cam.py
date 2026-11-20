#!/usr/bin/env python3
"""Record /wrist_cam/image_raw to an MP4 with a live projection overlay.

For every frame the cube centre and edges are projected using TF
base_link -> camera_optical_frame at the frame's stamp and the published
camera_info. If the camera frames are right, the yellow dot and green box sit
on the red cube in every frame, including while the arm moves.

    python3 record_wrist_cam.py OUT.mp4 SECONDS
"""
import itertools
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

CUBE, HALF = np.array([0.5, 0.0, 0.2]), 0.025

out_path, seconds = sys.argv[1], float(sys.argv[2])
rclpy.init()
node = Node("record_wrist_cam", parameter_overrides=[Parameter("use_sim_time", value=True)])
buf = Buffer()
TransformListener(buf, node)
state = {"k": None, "pending": []}
node.create_subscription(CameraInfo, "/wrist_cam/camera_info", lambda m: state.__setitem__("k", np.array(m.k).reshape(3, 3)), 10)
node.create_subscription(Image, "/wrist_cam/image_raw", lambda m: state["pending"].append(m), 30)
writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), 15.0, (640, 480))


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def project(k, t_cam_base, p):
    pc = t_cam_base @ np.append(p, 1.0)
    if pc[2] <= 0.01:
        return None
    return int(round(k[0, 0] * pc[0] / pc[2] + k[0, 2])), int(round(k[1, 1] * pc[1] / pc[2] + k[1, 2]))


frames, end = 0, time.time() + seconds
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.02)
    while state["pending"]:
        msg = state["pending"][0]
        stamp = Time.from_msg(msg.header.stamp)
        if not buf.can_transform("base_link", "camera_optical_frame", stamp):
            if len(state["pending"]) > 20:
                state["pending"].pop(0)  # TF for this stamp is not coming; drop it
            break
        state["pending"].pop(0)
        img = cv2.cvtColor(np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[..., :3], cv2.COLOR_RGB2BGR).copy()
        tf = buf.lookup_transform("base_link", "camera_optical_frame", stamp)
        t = np.eye(4)
        q, p = tf.transform.rotation, tf.transform.translation
        t[:3, :3], t[:3, 3] = quat_to_matrix(q.x, q.y, q.z, q.w), [p.x, p.y, p.z]
        t_cam_base = np.linalg.inv(t)
        k = state["k"]
        if k is not None:
            corners = [CUBE + HALF * np.array(s) for s in itertools.product((-1, 1), repeat=3)]
            pts = [project(k, t_cam_base, c) for c in corners]
            for i, j in itertools.combinations(range(8), 2):
                if np.count_nonzero(corners[i] != corners[j]) == 1 and pts[i] and pts[j]:
                    cv2.line(img, pts[i], pts[j], (0, 255, 0), 1, cv2.LINE_AA)
            c = project(k, t_cam_base, CUBE)
            if c and -1000 < c[0] < 2000 and -1000 < c[1] < 2000:
                cv2.circle(img, c, 6, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.circle(img, c, 6, (0, 255, 255), -1, cv2.LINE_AA)
        label = f"/wrist_cam/image_raw  sim t = {msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9:7.2f} s  frame {msg.header.frame_id}"
        cv2.putText(img, label, (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, label, (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img, "yellow dot / green box: cube projected from TF + camera_info", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, "yellow dot / green box: cube projected from TF + camera_info", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        writer.write(img)
        frames += 1
writer.release()
print(f"Wrote {frames} frames to {out_path}")
