"""Measure exactly how each sticker sits on its cube face, and save it to cube_faces.json.

cube_tracker.py --learn only finds each sticker's rotation in whole quarter turns and assumes it is
centred on its face. A sticker stuck on a few degrees crooked (or a few mm off centre) then tilts the
whole cube's computed orientation whenever that face is in view. This tool measures, for every face:
  - angle:  how far the sticker is turned on its face, beyond the quarter turn (deg)
  - offset: how far its centre is from the middle of the face (mm)
by fitting all of them at once to many frames that show two or three faces (a bundle adjustment).

Needs the camera driver running and ROS sourced:
    python3 tools/learn_faces.py --side-mm 50
Turn the cube slowly in front of the camera, edge-on so 2-3 faces are visible, until every face is done.
The frames are saved, so the fit can be re-run without the camera:
    python3 tools/learn_faces.py --data ~/cube_learn_data.json --side-mm 50
"""
import argparse
import json
import os
import sys
import threading
import time
from collections import Counter

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation as R

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_DIR)
from cube_perception.tracker_core import FACE_NAMES, CubeTracker, marker_obj_pts  # noqa: E402


# ---------------------------------------------------------------- collection (ROS)

def collect(args, tracker):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cube_perception.image_utils import image_to_bgr

    frames = []          # {"ids": [...], "corners": [[4x2], ...], "T_cam_cube": 4x4}
    seen = Counter()     # face id -> frames it appeared in (with at least one other face)
    pairs = Counter()    # (id_a, id_b) -> frames both appeared in
    done = threading.Event()

    def complete():
        return all(seen[m] >= args.per_face and
                   sum(1 for p, n in pairs.items() if m in p and n >= args.per_pair) >= 2
                   for m in tracker.learned)

    class Learn(Node):
        def __init__(self):
            super().__init__("learn_faces")
            self.last = 0.0
            self.create_subscription(Image, args.topic, self.on_image, 1)

        def on_image(self, msg):
            if done.is_set():
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp - self.last < args.every:
                return
            markers = tracker.detect_markers(image_to_bgr(msg))
            faces = {m: v for m, v in markers.items() if m in tracker.learned}
            if len(faces) < 2:
                return
            est = tracker.estimate_from_markers(markers)
            if not est.cube_ok:
                return
            self.last = stamp
            ids = sorted(faces)
            frames.append({"ids": ids, "corners": [faces[m][2].tolist() for m in ids],
                           "T_cam_cube": est.T_cam_cube.tolist()})
            for m in ids:
                seen[m] += 1
            for i, a in enumerate(ids):
                for b in ids[i + 1:]:
                    pairs[(a, b)] += 1
            if complete() or len(frames) >= args.max_frames:
                done.set()

    rclpy.init()
    node = Learn()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    threading.Thread(target=lambda: (input(), done.set()), daemon=True).start()

    print("Turn the cube slowly in front of the camera, edge-on so 2-3 faces are visible at once.")
    print("Go round every face. Stops by itself when all faces are done (or press Enter to stop).\n")
    try:
        while not done.wait(0.5):
            status = "  ".join(
                f"{FACE_NAMES[m]}:{min(seen[m], args.per_face)}/{args.per_face}"
                + ("✓" if seen[m] >= args.per_face else "")
                for m in sorted(tracker.learned, key=lambda m: FACE_NAMES[m]))
            print(f"\r  {len(frames):4d} frames   {status}   ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    print()
    rclpy.try_shutdown()
    node.destroy_node()
    return frames


# ---------------------------------------------------------------- fit

def rz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def refine(frames, tracker):
    """Bundle adjustment: per frame the cube pose (6), per face the sticker's extra angle about its
    normal and its in-plane offset (3). Sticker sizes, the cube size and the faces' directions are fixed."""
    face_ids = sorted({m for f in frames for m in f["ids"]})
    fi = {m: i for i, m in enumerate(face_ids)}
    R0 = {m: tracker.learned[m] for m in face_ids}
    t0 = {m: tracker.centres[m] for m in face_ids}
    P = marker_obj_pts(tracker.marker_mm).astype(np.float64)
    K, D = tracker.camera_matrix, tracker.dist_coeffs
    nf, nm = len(frames), len(face_ids)

    def face_model(p_face, m):
        theta, u, v = p_face
        Rm = R0[m] @ rz(theta)
        t = t0[m] + R0[m] @ np.array([u, v, 0.0])
        return (Rm @ P.T).T + t

    def residuals(p):
        pf = p[:6 * nf].reshape(nf, 6)
        pm = p[6 * nf:].reshape(nm, 3)
        models = {m: face_model(pm[fi[m]], m) for m in face_ids}
        out = []
        for k, f in enumerate(frames):
            obj = np.vstack([models[m] for m in f["ids"]])
            proj, _ = cv2.projectPoints(obj, pf[k, :3], pf[k, 3:], K, D)
            out.append((proj.reshape(-1, 2) - f["img"]).ravel())
        return np.concatenate(out)

    x0 = []
    for f in frames:
        T = np.array(f["T_cam_cube"])
        x0 += list(cv2.Rodrigues(T[:3, :3])[0].ravel()) + list(T[:3, 3])
        f["img"] = np.vstack([np.array(c, dtype=np.float64) for c in f["corners"]])
    x0 = np.array(x0 + [0.0] * (3 * nm))

    # Each frame's residuals depend on that frame's pose and on its visible faces only.
    n_res = sum(8 * len(f["ids"]) for f in frames)
    sparsity = lil_matrix((n_res, x0.size), dtype=int)
    row = 0
    for k, f in enumerate(frames):
        rows = slice(row, row + 8 * len(f["ids"]))
        sparsity[rows, 6 * k:6 * k + 6] = 1
        for m in f["ids"]:
            sparsity[rows, 6 * nf + 3 * fi[m]:6 * nf + 3 * fi[m] + 3] = 1
        row += 8 * len(f["ids"])

    # First re-fit the frame poses with the current model, for a fair 'before' number.
    frame_only = np.zeros(x0.size, dtype=bool)
    frame_only[:6 * nf] = True
    before = least_squares(lambda q: residuals(np.concatenate([q, x0[6 * nf:]])), x0[:6 * nf],
                           jac_sparsity=sparsity[:, :6 * nf], loss="soft_l1", f_scale=1.0, x_scale="jac")
    x0[:6 * nf] = before.x
    res_before = residuals(x0)

    sol = least_squares(residuals, x0, jac_sparsity=sparsity, loss="soft_l1", f_scale=1.0, x_scale="jac",
                        max_nfev=200, verbose=0)
    res_after = residuals(sol.x)
    pm = sol.x[6 * nf:].reshape(nm, 3)

    def per_face_rms(res):
        sq = {m: [] for m in face_ids}
        i = 0
        for f in frames:
            for m in f["ids"]:
                sq[m].append(res[i:i + 8] ** 2)
                i += 8
        return {m: float(np.sqrt(np.mean(np.concatenate(v)))) for m, v in sq.items()}

    counts = Counter(m for f in frames for m in f["ids"])
    result = {}
    for m in face_ids:
        theta, u, v = pm[fi[m]]
        result[m] = {"R": R0[m] @ rz(theta), "t": t0[m] + R0[m] @ np.array([u, v, 0.0]),
                     "angle_deg": float(np.degrees(theta)), "offset_mm": [float(u), float(v)],
                     "frames": counts[m]}
    rms = lambda r: float(np.sqrt(np.mean(r ** 2)))  # noqa: E731
    return result, rms(res_before), rms(res_after), per_face_rms(res_before), per_face_rms(res_after)


# ---------------------------------------------------------------- main

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--intrinsics", default=os.path.join(PKG_DIR, "config", "camera_intrinsics.npz"))
    parser.add_argument("--faces", default=os.path.join(PKG_DIR, "config", "cube_faces.json"),
                        help="current face file (which face is which; refined values are improved further)")
    parser.add_argument("--out", help="where to write the result (default: overwrite --faces)")
    parser.add_argument("--side-mm", type=float, default=50.0, help="cube side")
    parser.add_argument("--marker-mm", type=float, default=40.0, help="black square of the face markers")
    parser.add_argument("--data", help="re-use saved frames instead of the camera")
    parser.add_argument("--save", default=os.path.expanduser("~/cube_learn_data.json"), help="save frames here")
    parser.add_argument("--per-face", type=int, default=40, help="frames needed per face")
    parser.add_argument("--per-pair", type=int, default=10, help="frames needed per neighbouring pair")
    parser.add_argument("--every", type=float, default=0.15, help="seconds between kept frames")
    parser.add_argument("--max-frames", type=int, default=600)
    args = parser.parse_args()
    out = args.out or args.faces

    tracker = CubeTracker(args.intrinsics, args.faces, side_mm=args.side_mm, marker_mm=args.marker_mm,
                          max_fit_error_px=5.0)
    if args.data:
        with open(args.data) as f:
            frames = json.load(f)["frames"]
        print(f"loaded {len(frames)} frames from {args.data}")
    else:
        frames = collect(args, tracker)
        with open(args.save, "w") as f:
            json.dump({"frames": frames, "side_mm": args.side_mm}, f)
        print(f"saved {len(frames)} frames to {args.save}")
    if len(frames) < 20:
        raise SystemExit("Not enough frames with 2+ faces to fit. Try again, turning the cube slower.")

    print("fitting ...")
    t0 = time.time()
    result, before, after, face_before, face_after = refine(frames, tracker)
    print(f"done in {time.time() - t0:.1f} s\n")

    print(f"reprojection error, all frames: {before:.2f} px -> {after:.2f} px\n")
    print(f"  {'face':8s} {'ID':>3s} {'frames':>6s}  {'sticker turned':>14s}  {'sticker off-centre (mm)':>24s}"
          f"  {'error px before -> after':>24s}")
    for m in sorted(result, key=lambda m: FACE_NAMES[m]):
        r = result[m]
        print(f"  {FACE_NAMES[m]:8s} {m:3d} {r['frames']:6d}  {r['angle_deg']:+12.2f}°  "
              f"({r['offset_mm'][0]:+6.2f}, {r['offset_mm'][1]:+6.2f}){'':10s}"
              f"  {face_before[m]:10.2f} -> {face_after[m]:.2f}")
    missing = [FACE_NAMES[m] for m in tracker.learned if m not in result]
    if missing:
        print(f"\n  not seen with another face, left unchanged: {', '.join(missing)}")

    with open(args.faces) as f:
        doc = json.load(f)
    for m, r in result.items():
        entry = doc["faces"][str(m)]
        entry["R_cube_marker"] = np.round(r["R"], 6).tolist()
        entry["t_cube_marker_mm"] = np.round(r["t"], 3).tolist()
        entry["refined_angle_deg"] = round(r["angle_deg"], 3)
        entry["refined_offset_mm"] = [round(v, 3) for v in r["offset_mm"]]
        entry["refined_frames"] = r["frames"]
    doc.update({"side_mm": args.side_mm, "marker_mm": args.marker_mm, "refined": True,
                "refined_reprojection_px": round(after, 3)})
    with open(out, "w") as f:
        json.dump(doc, f, indent=2)
    print(f"\nsaved {out}")


if __name__ == "__main__":
    main()
