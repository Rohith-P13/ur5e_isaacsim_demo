# UR5e wrist camera exercise: evidence

Results for both problems in the candidate brief, recorded on 17 Nov 2026 from the running simulation: Isaac Sim 6.1 (GUI), ROS 2 Jazzy, MoveIt.

| | Result |
|---|---|
| **Problem 1:** wrist camera | All checks pass: 640 x 480, 15.00 Hz in sim time, `frame_id: camera_optical_frame`, sim-time stamps. The camera pose in TF follows the arm. |
| **Problem 2:** 5 images and projection check | The projected cube centre lands on the cube in all 5 images, 0.8 to 2.9 px from the centroid of the red cube pixels. |
| **Bonus:** depth | Depth at the projected point agrees with the cube geometry to within 0.4 mm. |

## Start here

1. `videos/1_isaac_sim_robot_and_wrist_camera.mp4` (64 s): the Isaac Sim window. Left viewport: the arm moving to the 5 views. Right viewport: the wrist camera's own view.
2. `../captures/overlays/contact_sheet.png`: all 5 views with the projected cube drawn on them.

## Videos (`videos/`)

All three were recorded at the same time, during one run of `capture_views.py`.

| File | What it shows |
|---|---|
| `1_isaac_sim_robot_and_wrist_camera.mp4` | Isaac Sim: the arm and the red target cube, plus the wrist camera viewport. |
| `2_rviz_camera_overlay_and_tf.mp4` | RViz **Camera** display on `/wrist_cam/image_raw` (top left). RViz draws a green box at the cube's position over the image from the camera's TF pose, and it stays on the red cube as the arm moves. Right: the robot model with the `tool0`, `camera_link` and `camera_optical_frame` axes. This is the Problem 1 "done when" check. |
| `3_wrist_camera_feed_live_projection.mp4` | Every frame of `/wrist_cam/image_raw`. The yellow dot and green box are projected for each frame from TF at that frame's stamp and the published `camera_info`, including while the arm moves. |

## Problem 1: wrist camera (`problem1_wrist_camera/`)

| File | Content |
|---|---|
| `topic_checks_before_motion.txt`, `topic_checks_after_motion.txt` | Pass/fail checks against the brief: size, rate (from header stamps), frame_id, sim-time stamps vs `/clock`, intrinsics, and the TF chain `tool0 -> camera_link -> camera_optical_frame`. |
| `tf_camera_pose_changes_with_arm.txt` | The camera pose in TF at 5 different arm configurations. |
| `screenshot_rviz_camera_display_overlay.png` | RViz Camera display with the TF-projected cube (green) over the red cube in the image. |
| `screenshot_isaac_sim_robot_and_wrist_camera_view.png` | Isaac Sim, robot and wrist camera viewport. |
| `screenshot_rviz_moveit.png` | The MoveIt RViz window. |

The wall-clock image rate in the checks (about 8 Hz) is lower than 15 Hz only because the simulator runs slower than real time with the GUI on a laptop GPU. In simulation time the camera publishes at exactly 15 Hz.

## Problem 2: capture and check (`../captures/`)

Target cube: 5 cm edge, centred at **(0.50, 0.00, 0.20) m in `base_link`**. The 5 views lie on an arc 0.30 m from the cube at 45 degree elevation, with azimuths from -40 to +40 degrees.

| File | Content |
|---|---|
| `images/view_00.png` to `view_04.png` | The 5 camera images. |
| `overlays/view_00.png` to `view_04.png` | The same images with the projection drawn: yellow dot = projected cube centre, green = projected cube edges, cyan cross = centroid of the red pixels. |
| `overlays/contact_sheet.png` | All 5 overlays on one page. |
| `poses.jsonl` | One line per image: stamp, joint names and angles, camera pose `base_link -> camera_optical_frame` (TF at the image stamp). |
| `camera_info.yaml` | Intrinsics (ROS `camera_calibration` format): fx = fy = 554.26, cx = 320, cy = 240, no distortion. |
| `projection_check_results.txt`, `report.json` | Pixel offset and depth error for each view. |
| `depth/` | Bonus depth images: `.npy` in metres, `.png` preview. |

| View | Azimuth | Projected centre (px) | Offset to red centroid | Depth error |
|---|---|---|---|---|
| 0 | -40 deg | (320.7, 237.0) | 1.9 px | -0.4 mm |
| 1 | -20 deg | (319.4, 239.0) | 2.1 px | -0.3 mm |
| 2 | 0 deg | (319.0, 237.2) | 0.8 px | +0.2 mm |
| 3 | +20 deg | (319.4, 239.7) | 2.9 px | -0.4 mm |
| 4 | +40 deg | (322.0, 238.4) | 2.0 px | -0.1 mm |

A small offset to the red centroid is expected: the centroid of the visible red faces is not the projection of the cube centre under perspective.

## A bug the check found in the original demo

On the first run the dot was on the cube, but every view was 17 px off and depth was 20 mm too far, by the same amount in all 5 views. Back-projecting the depth image showed the cube 1.4 cm low in Z only. Reading the live prim poses in Isaac showed the cause: the robot base sat 1.47 cm above the origin. The base collision mesh overlaps the Simple Room table, and the base is held only by a non-rigid fixed joint, so PhysX pushed the robot up. TF (from the URDF) did not know about it. Two fixes went into `spawn_ur5e.py`:

- Filter contacts between the base and the room (the arm is bolted to its table).
- Spawn the arm in its ros2_control start pose instead of all-zero joints, so it never starts inside the table.

After the fixes the error dropped to under 3 px and under 0.5 mm. The full write-up is in the repository README.

## Other files

| Path | Content |
|---|---|
| `tools/` | Scripts used only to produce this evidence: the Problem 1 checks, the wrist-feed recorder, the RViz config and the cube marker. |

The camera, capture and check code is in `../ros2_ws/src/ur5e_isaac_bridge/`; the full write-up is in `../README.md`. AI tool used: Claude (Anthropic), through Claude Code.
