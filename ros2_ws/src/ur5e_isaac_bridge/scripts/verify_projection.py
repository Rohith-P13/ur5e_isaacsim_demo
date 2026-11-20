#!/usr/bin/env python3
"""Check the captured views: project the cube into each image and measure.

Offline, no ROS needed. Reads what capture_views.py wrote:

    python3 verify_projection.py --dir ~/robotics/ur5e_isaacsim_demo/captures

For every view it uses only the recorded camera pose (TF base_link ->
camera_optical_frame at the image stamp) and camera_info.yaml to predict
where the cube is, then compares that prediction with the image itself:

  * draws the projected cube centre (yellow dot) and the 12 projected cube
    edges (green),
  * segments the red cube and reports the pixel offset between the
    projected centre and the centroid of the red pixels (cyan cross),
  * if depth was saved, ray-casts the cube along the centre pixel and
    compares the expected depth with the depth image.

Writes overlays/view_XX.png, overlays/contact_sheet.png and report.json.
"""

from __future__ import annotations

import argparse
import itertools
import json
import os

import cv2
import numpy as np
import yaml


def quat_to_matrix(x, y, z, w) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def project(k: np.ndarray, t_cam_base: np.ndarray, p_base: np.ndarray):
    """Pinhole projection of a base_link point. Returns (u, v, z_cam) or None."""
    p_c = t_cam_base @ np.append(p_base, 1.0)
    if p_c[2] <= 0:  # behind the camera
        return None
    u = k[0, 0] * p_c[0] / p_c[2] + k[0, 2]
    v = k[1, 1] * p_c[1] / p_c[2] + k[1, 2]
    return u, v, p_c[2]


def ray_box_depth(k, t_base_cam, u, v, box_min, box_max):
    """Depth (camera Z) where the ray through pixel (u, v) first hits the box."""
    d_cam = np.array([(u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1], 1.0])
    origin = t_base_cam[:3, 3]
    d = t_base_cam[:3, :3] @ d_cam
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (box_min - origin) / d, (box_max - origin) / d
    t_near = np.max(np.minimum(t1, t2))
    t_far = np.min(np.maximum(t1, t2))
    if t_near > t_far or t_far < 0:
        return None
    return float(t_near)  # d_cam has z = 1, so the ray parameter is camera Z


def red_mask(bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lo = cv2.inRange(hsv, (0, 120, 50), (10, 255, 255))
    hi = cv2.inRange(hsv, (170, 120, 50), (180, 255, 255))
    return cv2.morphologyEx(lo | hi, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def put_label(img, text):
    cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dir", default="captures")
    args = parser.parse_args()
    root = os.path.expanduser(args.dir)
    os.makedirs(os.path.join(root, "overlays"), exist_ok=True)

    with open(os.path.join(root, "camera_info.yaml"), encoding="utf-8") as f:
        info = yaml.safe_load(f)
    k = np.array(info["camera_matrix"]["data"]).reshape(3, 3)
    with open(os.path.join(root, "poses.jsonl"), encoding="utf-8") as f:
        records = [json.loads(line) for line in f if line.strip()]

    results, tiles = [], []
    for rec in records:
        bgr = cv2.imread(os.path.join(root, rec["image"]))
        cube = np.array(rec["cube_xyz"])
        half = rec["cube_size"] / 2.0

        # Saved pose is base_link -> camera_optical_frame; invert it to map
        # base_link points into the camera frame.
        t_base_cam = np.eye(4)
        t_base_cam[:3, :3] = quat_to_matrix(*rec["rotation_xyzw"])
        t_base_cam[:3, 3] = rec["translation"]
        t_cam_base = np.linalg.inv(t_base_cam)

        overlay = bgr.copy()
        corners = [cube + half * np.array(s) for s in itertools.product((-1, 1), repeat=3)]
        proj = [project(k, t_cam_base, c) for c in corners]
        for i, j in itertools.combinations(range(8), 2):
            # A cube edge joins two corners that differ in exactly one axis.
            if np.count_nonzero(corners[i] != corners[j]) == 1 and proj[i] and proj[j]:
                p1 = (int(round(proj[i][0])), int(round(proj[i][1])))
                p2 = (int(round(proj[j][0])), int(round(proj[j][1])))
                cv2.line(overlay, p1, p2, (0, 255, 0), 1, cv2.LINE_AA)

        result = {"index": rec["index"], "image": rec["image"], "azimuth_deg": rec["azimuth_deg"]}
        centre = project(k, t_cam_base, cube)
        if centre is None:
            result["error"] = "cube centre is behind the camera"
            results.append(result)
            print(f"view {rec['index']}: {result['error']}")
            continue
        u, v, z = centre
        in_view = 0 <= u < bgr.shape[1] and 0 <= v < bgr.shape[0]
        result.update(projected_uv=[u, v], cube_centre_depth_m=z, in_view=bool(in_view))

        # Independent check: where is the red cube in the image itself?
        m = cv2.moments(red_mask(bgr), binaryImage=True)
        if m["m00"] > 50:
            cu, cv_ = m["m10"] / m["m00"], m["m01"] / m["m00"]
            result.update(
                red_centroid_uv=[cu, cv_],
                red_pixels=int(m["m00"]),
                centroid_offset_px=float(np.hypot(cu - u, cv_ - v)),
            )
            cv2.drawMarker(overlay, (int(round(cu)), int(round(cv_))), (255, 255, 0), cv2.MARKER_CROSS, 14, 1)

        if rec.get("depth") and in_view:
            depth = np.load(os.path.join(root, rec["depth"]))
            measured = float(depth[int(round(v)), int(round(u))])
            expected = ray_box_depth(k, t_base_cam, u, v, cube - half, cube + half)
            result.update(depth_measured_m=measured, depth_expected_m=expected)
            if expected is not None and np.isfinite(measured):
                result["depth_error_mm"] = (measured - expected) * 1000.0

        cv2.circle(overlay, (int(round(u)), int(round(v))), 6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.circle(overlay, (int(round(u)), int(round(v))), 6, (0, 255, 255), -1, cv2.LINE_AA)
        label = f"view {rec['index']}  az {rec['azimuth_deg']:+.0f} deg"
        if "centroid_offset_px" in result:
            label += f"  offset {result['centroid_offset_px']:.1f} px"
        put_label(overlay, label)

        out_name = os.path.join("overlays", os.path.basename(rec["image"]))
        cv2.imwrite(os.path.join(root, out_name), overlay)
        result["overlay"] = out_name
        results.append(result)
        tiles.append(cv2.resize(overlay, (overlay.shape[1] // 2, overlay.shape[0] // 2)))

        line = f"view {rec['index']} az {rec['azimuth_deg']:+4.0f}: projected ({u:6.1f}, {v:6.1f})"
        if "centroid_offset_px" in result:
            line += f"  red-centroid offset {result['centroid_offset_px']:5.1f} px"
        if "depth_error_mm" in result:
            line += f"  depth error {result['depth_error_mm']:+6.1f} mm"
        print(line)

    if tiles:
        while len(tiles) % 3:
            tiles.append(np.zeros_like(tiles[0]))
        rows = [np.hstack(tiles[i : i + 3]) for i in range(0, len(tiles), 3)]
        cv2.imwrite(os.path.join(root, "overlays", "contact_sheet.png"), np.vstack(rows))

    with open(os.path.join(root, "report.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"Wrote {len(results)} results to {os.path.join(root, 'report.json')}")


if __name__ == "__main__":
    main()
