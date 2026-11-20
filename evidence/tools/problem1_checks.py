#!/usr/bin/env python3
"""Problem 1 evidence: check the wrist camera topics against the worksheet.

Worksheet: /wrist_cam/image_raw and /wrist_cam/camera_info, 640x480, 15 Hz,
simulation-time stamps, frame_id camera_optical_frame, camera pose in TF
changes as the arm moves.
"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

rclpy.init()
n = Node("problem1_checks", parameter_overrides=[Parameter("use_sim_time", value=True)])
buf = Buffer(); TransformListener(buf, n)
imgs, infos, clocks = [], [], []
wall_arrivals = []
n.create_subscription(Image, "/wrist_cam/image_raw", lambda m: (imgs.append(m), wall_arrivals.append(time.time())), 10)
n.create_subscription(CameraInfo, "/wrist_cam/camera_info", lambda m: infos.append(m), 10)
n.create_subscription(Clock, "/clock", lambda m: clocks.append(m.clock.sec + m.clock.nanosec * 1e-9), 10)
end = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 6)
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.02)

def sec(s): return s.sec + s.nanosec * 1e-9
stamps = np.array([sec(m.header.stamp) for m in imgs])
ok = lambda c: "PASS" if c else "FAIL"
out = []
P_ = out.append
P_("PROBLEM 1 - WRIST CAMERA CHECKS")
P_(f"Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}   ({len(imgs)} images, {len(infos)} camera_info messages sampled)")
P_("")
m, ci = imgs[-1], infos[-1]
P_(f"[{ok((m.width, m.height) == (640, 480))}] image size             {m.width} x {m.height}  (required 640 x 480)  encoding {m.encoding}")
P_(f"[{ok((ci.width, ci.height) == (640, 480))}] camera_info size       {ci.width} x {ci.height}")
rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
P_(f"[{ok(abs(rate - 15) < 0.5)}] rate (sim time)        {rate:.2f} Hz  (required 15 Hz; from header stamps)")
P_(f"       rate (wall clock)      {(len(wall_arrivals) - 1) / (wall_arrivals[-1] - wall_arrivals[0]):.2f} Hz  (depends on how fast the simulator runs)")
fids = {x.header.frame_id for x in imgs} | {x.header.frame_id for x in infos}
P_(f"[{ok(fids == {'camera_optical_frame'})}] frame_id               {sorted(fids)}  (required camera_optical_frame)")
lag = clocks[-1] - stamps[-1]
P_(f"[{ok(0 <= lag < 1.0 and stamps[-1] < 1e6)}] simulation-time stamps last image stamp {stamps[-1]:.3f} s, /clock {clocks[-1]:.3f} s (difference {lag*1000:.0f} ms; wall time would be ~1.8e9 s)")
k = np.array(ci.k).reshape(3, 3)
P_(f"       intrinsics K           fx={k[0,0]:.2f} fy={k[1,1]:.2f} cx={k[0,2]:.1f} cy={k[1,2]:.1f}  distortion {ci.distortion_model} {list(ci.d)}")
P_("")
for parent, child in (("tool0", "camera_link"), ("camera_link", "camera_optical_frame"), ("base_link", "camera_optical_frame")):
    t = buf.lookup_transform(parent, child, Time())
    tr, q = t.transform.translation, t.transform.rotation
    P_(f"       TF {parent:>11s} -> {child:<21s} xyz ({tr.x:+.4f}, {tr.y:+.4f}, {tr.z:+.4f})  quat xyzw ({q.x:+.4f}, {q.y:+.4f}, {q.z:+.4f}, {q.w:+.4f})")
print("\n".join(out))
