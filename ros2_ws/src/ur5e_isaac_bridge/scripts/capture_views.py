#!/usr/bin/env python3
"""Move the UR5e to camera views around the target cube and record each view.

Run with the simulator and MoveIt already up (ros2 launch ur5e_isaac_bridge
ur5e_isaac.launch.py), from a terminal with ROS 2 sourced:

    python3 capture_views.py --out ~/robotics/ur5e_isaacsim_demo/captures

For each view the script:
  1. computes a camera pose on an arc around the cube, optical Z aimed at it,
  2. converts that to a tool0 pose and solves IK with MoveIt /compute_ik,
  3. plans and executes the joint goal through the MoveGroup action,
  4. waits for the arm to settle (joint positions still for 0.5 s sim time),
  5. takes the first image stamped after the settle time,
  6. looks up TF base_link -> camera_optical_frame at that image stamp,
  7. saves the PNG (+ depth) and appends a line to poses.jsonl.
camera_info.yaml is written once.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
GROUP = "ur_manipulator"
BASE_FRAME = "base_link"
PLANNING_FRAME = "world"  # identity to base_link in this URDF
OPTICAL_FRAME = "camera_optical_frame"
IK_LINK = "tool0"
# Elbow up, camera looking straight down from above the cube's side of the
# workspace. The arm goes here first (the launch pose points straight up and
# the direct path down sweeps the forearm through the cube), and it seeds IK
# so every view uses the same branch.
READY = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]


# --------------------------------------------------------------------------
# Small geometry helpers (no tf_transformations dependency)
# --------------------------------------------------------------------------
def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat(r: np.ndarray) -> tuple[float, float, float, float]:
    """Rotation matrix -> quaternion (x, y, z, w)."""
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w, x = 0.25 / s, (r[2, 1] - r[1, 2]) * s
        y, z = (r[0, 2] - r[2, 0]) * s, (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w, x = (r[2, 1] - r[1, 2]) / s, 0.25 * s
        y, z = (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w, x = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s
        y, z = 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w, x = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s
        y, z = (r[1, 2] + r[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    return tuple(float(v) for v in q)


def transform_to_matrix(tf) -> np.ndarray:
    t, q = tf.transform.translation, tf.transform.rotation
    m = np.eye(4)
    m[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def look_at_optical(cam_pos: np.ndarray, target: np.ndarray) -> np.ndarray:
    """4x4 pose of an optical frame at cam_pos whose +Z points at target.

    Optical convention: Z forward, X right, Y down. "Down" in the image is
    chosen as world -Z projected onto the image plane, so the horizon stays
    level.
    """
    z = target - cam_pos
    z /= np.linalg.norm(z)
    down = np.array([0.0, 0.0, -1.0])
    y = down - np.dot(down, z) * z
    if np.linalg.norm(y) < 1e-6:  # looking straight down: pick any image-down
        y = np.array([-1.0, 0.0, 0.0]) - np.dot([-1.0, 0.0, 0.0], z) * z
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    m = np.eye(4)
    m[:3, 0], m[:3, 1], m[:3, 2], m[:3, 3] = x, y, z, cam_pos
    return m


def arc_views(cube: np.ndarray, radius: float, elevation_deg: float, azimuths_deg: list[float]):
    """Camera poses on an arc around the cube, on the robot side of it.

    Azimuth 0 is the camera between the robot base and the cube (towards -X
    from the cube); positive azimuth swings it towards -Y.
    """
    el = math.radians(elevation_deg)
    views = []
    for az_deg in azimuths_deg:
        az = math.radians(az_deg)
        offset = radius * np.array(
            [-math.cos(el) * math.cos(az), -math.cos(el) * math.sin(az), math.sin(el)]
        )
        views.append((az_deg, look_at_optical(cube + offset, cube)))
    return views


def stamp_to_sec(stamp: TimeMsg) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


# --------------------------------------------------------------------------
# ROS node
# --------------------------------------------------------------------------
class Capture(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(
            "capture_views", parameter_overrides=[Parameter("use_sim_time", value=True)]
        )
        self.args = args
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.joint_hist: list[tuple[float, dict[str, float]]] = []
        self.images: list[Image] = []
        self.depths: list[Image] = []
        self.camera_info: CameraInfo | None = None
        ns = args.camera_namespace.rstrip("/")
        self.create_subscription(JointState, "/joint_states", self._on_joints, 50)
        self.create_subscription(Image, f"{ns}/image_raw", self._on_image, 10)
        self.create_subscription(Image, f"{ns}/depth/image_raw", self._on_depth, 10)
        self.create_subscription(CameraInfo, f"{ns}/camera_info", self._on_info, 10)

        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.move = ActionClient(self, MoveGroup, "/move_action")

    # --- callbacks -------------------------------------------------------
    def _on_joints(self, msg: JointState) -> None:
        t = stamp_to_sec(msg.header.stamp)
        self.joint_hist.append((t, dict(zip(msg.name, msg.position))))
        self.joint_hist = self.joint_hist[-400:]

    def _on_image(self, msg: Image) -> None:
        self.images = (self.images + [msg])[-30:]

    def _on_depth(self, msg: Image) -> None:
        self.depths = (self.depths + [msg])[-30:]

    def _on_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    # --- helpers ---------------------------------------------------------
    def spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_until(self, predicate, timeout: float, what: str):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            result = predicate()
            if result:
                return result
        raise TimeoutError(f"Timed out after {timeout:.0f}s waiting for {what}")

    def call(self, client, request, timeout: float = 10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"Service {client.srv_name} not available")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f"Service {client.srv_name} did not answer")
        return future.result()

    def current_arm_positions(self) -> list[float]:
        _, joints = self.joint_hist[-1]
        return [joints[j] for j in ARM_JOINTS]

    # --- steps -----------------------------------------------------------
    def add_cube_to_planning_scene(self, cube: np.ndarray, size: float) -> None:
        """MoveIt does not see Isaac's stage, so tell it about the cube."""
        obj = CollisionObject()
        obj.header.frame_id = PLANNING_FRAME
        obj.id = "target_cube"
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[size] * 3)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in cube)
        pose.orientation.w = 1.0
        obj.primitives, obj.primitive_poses = [box], [pose]
        obj.operation = CollisionObject.ADD
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [obj]
        resp = self.call(self.scene, ApplyPlanningScene.Request(scene=scene))
        self.get_logger().info(f"Cube added to planning scene: {resp.success}")

    def solve_ik(self, tool0_pose: np.ndarray, seed: list[float]) -> list[float] | None:
        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = GROUP
        ik.ik_link_name = IK_LINK
        ik.avoid_collisions = True
        ik.timeout = Duration(seconds=1.0).to_msg()
        ik.robot_state = RobotState()
        ik.robot_state.joint_state.name = list(ARM_JOINTS)
        ik.robot_state.joint_state.position = [float(v) for v in seed]
        ps = PoseStamped()
        ps.header.frame_id = PLANNING_FRAME
        p = tool0_pose[:3, 3]
        q = matrix_to_quat(tool0_pose[:3, :3])
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in p)
        (
            ps.pose.orientation.x,
            ps.pose.orientation.y,
            ps.pose.orientation.z,
            ps.pose.orientation.w,
        ) = q
        ik.pose_stamped = ps
        resp = self.call(self.ik, req)
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        sol = dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))
        return [sol[j] for j in ARM_JOINTS]

    def move_to_joints(self, target: list[float], attempts: int = 3) -> bool:
        """Plan and execute a joint goal, replanning if MoveIt rejects a plan.

        OMPL checks collisions between states at a finite resolution, so a
        plan can occasionally clip the cube; MoveIt's ValidateSolution step
        catches that and returns INVALID_MOTION_PLAN. A fresh plan usually
        finds a clean path.
        """
        for attempt in range(1, attempts + 1):
            if self._move_once(target):
                return True
            self.get_logger().warn(f"Motion attempt {attempt}/{attempts} failed")
        return False

    def _move_once(self, target: list[float]) -> bool:
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.args.speed
        req.max_acceleration_scaling_factor = self.args.speed
        constraints = Constraints()
        for name, value in zip(ARM_JOINTS, target):
            constraints.joint_constraints.append(
                JointConstraint(
                    joint_name=name,
                    position=float(value),
                    tolerance_above=1e-3,
                    tolerance_below=1e-3,
                    weight=1.0,
                )
            )
        req.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        if not self.move.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveGroup action /move_action not available")
        send = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=60.0)
        code = result.result().result.error_code.val if result.done() else None
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"MoveGroup failed with error code {code}")
            return False
        return True

    def wait_until_settled(self, window: float, tol: float, timeout: float = 20.0) -> float:
        """Return the sim time at which the arm has been still for `window` s.

        Uses joint *positions*, not velocities: on this Isaac Sim build the
        velocities published by the bridge are noisy (up to ~3 rad/s while
        the arm is stationary), so a velocity threshold never triggers.
        """

        def settled():
            if len(self.joint_hist) < 2:
                return None
            t_end = self.joint_hist[-1][0]
            if self.joint_hist[0][0] > t_end - window:
                return None  # not enough history yet to cover the window
            recent = [(t, j) for t, j in self.joint_hist if t >= t_end - window]
            q = np.array([[j[n] for n in ARM_JOINTS] for _, j in recent])
            if np.max(q.max(axis=0) - q.min(axis=0)) < tol:
                return t_end
            return None

        return self.spin_until(settled, timeout, "the arm to settle")

    def fresh_frames(self, after: float) -> tuple[Image, Image | None]:
        """First RGB image stamped strictly after `after`, plus matching depth."""

        def first_after():
            for img in self.images:
                if stamp_to_sec(img.header.stamp) > after:
                    return img
            return None

        img = self.spin_until(first_after, 10.0, "a fresh camera image")
        t_img = stamp_to_sec(img.header.stamp)

        def depth_match():
            for d in self.depths:
                if abs(stamp_to_sec(d.header.stamp) - t_img) < 1e-6:
                    return d
            return None

        try:
            depth = self.spin_until(depth_match, 2.0, "a matching depth image")
        except TimeoutError:
            depth = None
        return img, depth

    def write_camera_info(self, path: str) -> None:
        ci = self.camera_info
        data = {
            "image_width": ci.width,
            "image_height": ci.height,
            "camera_name": "wrist_cam",
            "frame_id": ci.header.frame_id,
            "camera_matrix": {"rows": 3, "cols": 3, "data": [float(v) for v in ci.k]},
            "distortion_model": ci.distortion_model,
            "distortion_coefficients": {
                "rows": 1,
                "cols": len(ci.d),
                "data": [float(v) for v in ci.d],
            },
            "rectification_matrix": {"rows": 3, "cols": 3, "data": [float(v) for v in ci.r]},
            "projection_matrix": {"rows": 3, "cols": 4, "data": [float(v) for v in ci.p]},
        }
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)

    # --- main ------------------------------------------------------------
    def run(self) -> None:
        a = self.args
        out = os.path.expanduser(a.out)
        os.makedirs(os.path.join(out, "images"), exist_ok=True)
        os.makedirs(os.path.join(out, "depth"), exist_ok=True)
        cube = np.array(a.cube_xyz, dtype=float)

        self.get_logger().info("Waiting for joint states, camera and TF...")
        self.spin_until(
            lambda: self.joint_hist and self.images and self.camera_info,
            30.0,
            "/joint_states, image and camera_info",
        )
        tf = self.spin_until(
            lambda: self.tf_buffer.can_transform(IK_LINK, OPTICAL_FRAME, Time())
            and self.tf_buffer.lookup_transform(IK_LINK, OPTICAL_FRAME, Time()),
            10.0,
            f"TF {IK_LINK} -> {OPTICAL_FRAME}",
        )
        t_tool0_optical = transform_to_matrix(tf)  # fixed: from the URDF
        t_optical_tool0 = np.linalg.inv(t_tool0_optical)

        self.write_camera_info(os.path.join(out, "camera_info.yaml"))
        self.add_cube_to_planning_scene(cube, a.cube_size)

        views = arc_views(cube, a.radius, a.elevation, a.azimuths)
        poses_path = os.path.join(out, "poses.jsonl")
        open(poses_path, "w").close()
        self.get_logger().info("Moving to the ready pose")
        if not self.move_to_joints(READY):
            raise RuntimeError("Could not reach the ready pose")
        seed = list(READY)

        for index, (azimuth, t_base_optical_goal) in enumerate(views):
            tag = f"view_{index:02d}"
            t_base_tool0_goal = t_base_optical_goal @ t_optical_tool0
            joints = self.solve_ik(t_base_tool0_goal, seed)
            if joints is None:
                self.get_logger().error(f"{tag}: no IK solution for azimuth {azimuth} deg, skipped")
                continue
            seed = joints
            self.get_logger().info(
                f"{tag}: azimuth {azimuth:+.0f} deg, joints {np.round(joints, 3).tolist()}"
            )
            if not self.move_to_joints(joints):
                self.get_logger().error(f"{tag}: motion failed, skipped")
                continue

            t_settled = self.wait_until_settled(a.settle_window, a.settle_tol)
            img, depth = self.fresh_frames(after=t_settled)
            stamp = Time.from_msg(img.header.stamp)
            # TF for this stamp can arrive a tick after the image. A blocking
            # lookup with a timeout would not process callbacks in this
            # single-threaded node, so spin until the buffer can answer.
            self.spin_until(
                lambda: self.tf_buffer.can_transform(BASE_FRAME, OPTICAL_FRAME, stamp),
                5.0,
                f"TF {BASE_FRAME} -> {OPTICAL_FRAME} at the image stamp",
            )
            tf = self.tf_buffer.lookup_transform(BASE_FRAME, OPTICAL_FRAME, stamp)
            t_base_optical = transform_to_matrix(tf)

            rgb = np.frombuffer(img.data, np.uint8).reshape(img.height, img.width, -1)[..., :3]
            rgb_path = os.path.join("images", f"{tag}.png")
            cv2.imwrite(os.path.join(out, rgb_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            depth_path = None
            if depth is not None:
                dm = np.frombuffer(depth.data, np.float32).reshape(depth.height, depth.width)
                depth_path = os.path.join("depth", f"{tag}.npy")
                np.save(os.path.join(out, depth_path), dm)
                vis = np.clip(np.nan_to_num(dm, posinf=0.0) / 1.0, 0, 1)
                cv2.imwrite(
                    os.path.join(out, "depth", f"{tag}.png"),
                    cv2.applyColorMap((vis * 255).astype(np.uint8), cv2.COLORMAP_TURBO),
                )

            # The pose actually reached, not the requested one.
            q = matrix_to_quat(t_base_optical[:3, :3])
            joints_now = self.joint_hist[-1][1]
            goal_err = np.linalg.norm(t_base_optical[:3, 3] - t_base_optical_goal[:3, 3])
            record = {
                "index": index,
                "image": rgb_path,
                "depth": depth_path,
                "stamp": stamp_to_sec(img.header.stamp),
                "settled_at": t_settled,
                "azimuth_deg": azimuth,
                "joint_names": ARM_JOINTS,
                "joint_positions": [float(joints_now[j]) for j in ARM_JOINTS],
                "parent_frame": BASE_FRAME,
                "child_frame": OPTICAL_FRAME,
                "translation": [float(v) for v in t_base_optical[:3, 3]],
                "rotation_xyzw": list(q),
                "position_error_vs_goal_m": float(goal_err),
                "cube_xyz": [float(v) for v in cube],
                "cube_size": a.cube_size,
            }
            with open(poses_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(record) + "\n")
            self.get_logger().info(
                f"{tag}: saved, image stamp {record['stamp']:.3f}s "
                f"(settled {t_settled:.3f}s), camera within {goal_err * 1000:.1f} mm of goal"
            )

        self.get_logger().info(f"Done. Output in {out}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="captures")
    parser.add_argument("--camera-namespace", default="/wrist_cam")
    parser.add_argument("--cube-xyz", type=float, nargs=3, default=[0.5, 0.0, 0.2])
    parser.add_argument("--cube-size", type=float, default=0.05)
    parser.add_argument("--radius", type=float, default=0.30, help="Camera distance to cube centre, m")
    parser.add_argument("--elevation", type=float, default=45.0, help="Arc elevation above horizontal, deg")
    parser.add_argument("--azimuths", type=float, nargs="+", default=[-40, -20, 0, 20, 40])
    parser.add_argument("--speed", type=float, default=0.3, help="MoveIt velocity/acceleration scaling")
    parser.add_argument("--settle-window", type=float, default=0.5, help="Seconds of stillness, sim time")
    parser.add_argument(
        "--settle-tol",
        type=float,
        default=5e-4,
        help="Max joint motion in window, rad. Isaac publishes positions rounded to 1e-4 rad.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Capture(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
