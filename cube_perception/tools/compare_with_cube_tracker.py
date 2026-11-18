"""Check tracker_core.py gives the same cube pose as the original cube_tracker.py, on saved images.

Runs both on every image: cube_tracker.py's own functions (the same steps as its track() loop)
and tracker_core.CubeTracker. Prints the cube pose from the hand root (mm, deg) for each, and the
difference. They should agree to well under 0.1 mm / 0.1 deg (same maths, same inputs).

    python3 tools/compare_with_cube_tracker.py ~/cube_frames/*.png \
        --cube-tracker ~/Pictures/Webcam/cube_tracker.py
"""
import argparse
import importlib.util
import os
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_DIR)
from cube_perception.tracker_core import CubeTracker  # noqa: E402


def load_module(path):
    spec = importlib.util.spec_from_file_location("cube_tracker_original", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def original_poses(ct, frame, camera_matrix, dist_coeffs, learned, corner_model):
    """The steps of cube_tracker.track() for one frame, without drawing.
    Returns T_cam_root, T_cam_cube (None where not seen) and a note."""
    markers = ct.detect_markers(frame, camera_matrix, dist_coeffs)
    faces = {m: v for m, v in markers.items() if m in ct.FACE_NAMES}
    T_cam_root = None
    if ct.HAND_MARKER_ID in markers:
        R_hm, t_hm, _ = markers[ct.HAND_MARKER_ID]
        T_cam_root = ct.to_T(R_hm, t_hm)
    estimates = []
    for mid, (R_cm, t_cm, _) in faces.items():
        if mid not in learned:
            continue
        R_cam_cube, center = ct.cube_pose_from_face(R_cm, t_cm, learned[mid])
        estimates.append((R_cam_cube, center, max(-R_cm[2, 2], 0.0)))
    if not estimates:
        return T_cam_root, None, "no cube face"
    R_cam_cube, center, _, _ = ct.fuse(estimates)
    joint = ct.solve_cube_jointly(faces, corner_model, R_cam_cube, center, camera_matrix, dist_coeffs)
    rms = float("nan")
    if joint is not None:
        R_cam_cube, center, rms = joint
    return T_cam_root, ct.to_T(R_cam_cube, center), f"fit {rms:.2f} px"


def fmt(T):
    x, y, z = T[:3, 3]
    r, p, yw = R.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True)
    return f"({x:7.2f}, {y:7.2f}, {z:7.2f}) mm  rpy ({r:7.2f}, {p:7.2f}, {yw:7.2f})"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("images", nargs="+")
    parser.add_argument("--cube-tracker", default=os.path.expanduser("~/Pictures/Webcam/cube_tracker.py"))
    parser.add_argument("--intrinsics", default=os.path.join(PKG_DIR, "config", "camera_intrinsics.npz"))
    parser.add_argument("--faces", default=os.path.join(PKG_DIR, "config", "cube_faces.json"))
    args = parser.parse_args()

    ct = load_module(args.cube_tracker)
    intr = np.load(args.intrinsics)
    camera_matrix, dist_coeffs = intr["camera_matrix"], intr["dist_coeffs"]
    import json
    with open(args.faces) as f:
        learned = {int(k): np.array(v["R_cube_marker"], dtype=float) for k, v in json.load(f)["faces"].items()}
    corner_model = ct.cube_corner_model(learned)

    # No fit-error rejection here: compare the raw numbers.
    tracker = CubeTracker(args.intrinsics, args.faces, side_mm=ct.SIDE_MM, marker_mm=ct.MARKER_MM,
                          hand_marker_id=ct.HAND_MARKER_ID, hand_marker_mm=ct.HAND_MARKER_MM, hd=ct.HD,
                          max_fit_error_px=float("inf"))

    worst_mm = worst_deg = 0.0
    compared = 0
    for path in args.images:
        frame = cv2.imread(path)
        if frame is None:
            print(f"{os.path.basename(path)}: could not read")
            continue
        print(f"{os.path.basename(path)}  {frame.shape[1]}x{frame.shape[0]}")
        T_cam_root, T_cam_cube, note = original_poses(ct, frame, camera_matrix, dist_coeffs, learned, corner_model)
        est = tracker.estimate(frame)
        if T_cam_cube is None or not est.cube_ok:
            print(f"  cube: original {note if T_cam_cube is None else 'ok'}, core {'ok' if est.cube_ok else est.reason}")
            continue
        pairs = [("cube from camera", T_cam_cube, est.T_cam_cube)]
        if T_cam_root is not None and est.valid:
            pairs.append(("cube from root  ", np.linalg.inv(T_cam_root) @ T_cam_cube, est.T_root_cube()))
        else:
            print("  (hand marker ID 10 not visible: only the camera-frame pose is compared)")
        for label, T_orig, T_core in pairs:
            d_mm = np.linalg.norm(T_orig[:3, 3] - T_core[:3, 3])
            d_deg = np.degrees((R.from_matrix(T_orig[:3, :3]) * R.from_matrix(T_core[:3, :3]).inv()).magnitude())
            worst_mm, worst_deg = max(worst_mm, d_mm), max(worst_deg, d_deg)
            print(f"  {label} original: {fmt(T_orig)}  {note}")
            print(f"  {label} core:     {fmt(T_core)}  fit {est.fit_error_px:.2f} px  faces {est.face_ids}")
            print(f"  {label} diff:     {d_mm:.4f} mm  {d_deg:.4f} deg")
        compared += 1

    print(f"\n{compared} image(s) compared; worst difference {worst_mm:.4f} mm, {worst_deg:.4f} deg")
    if compared == 0:
        print("Nothing to compare: no image showed a cube face.")
        sys.exit(1)
    sys.exit(0 if worst_mm < 0.1 and worst_deg < 0.1 else 1)


if __name__ == "__main__":
    main()
