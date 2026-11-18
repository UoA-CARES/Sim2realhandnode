"""Measure the cube centre relative to the hand marker (STag 10), for accuracy tests.

Place the cube, type a label and press Enter: the tool averages the next --frames frames and prints the
cube centre in the hand_root frame (mm), its spread, and the rotation. Every new position is also compared
with the first one and the previous one, so you can check known moves (e.g. 20 mm along a ruler).

Needs the camera driver running and ROS sourced:
    python3 tools/measure_cube.py
    python3 tools/measure_cube.py --frames 90 --out ~/cube_accuracy.csv

Axes (hand_root = centre of marker 10): x toward the little finger, y toward the fingertips,
z out of the marker's surface.
"""
import argparse
import csv
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_DIR)
from cube_perception.image_utils import image_to_bgr  # noqa: E402
from cube_perception.tracker_core import CubeTracker, HandRootAverage  # noqa: E402


class Collector(Node):
    """Runs the tracker only while a measurement is being collected.

    Marker 10 goes through HandRootAverage, like in the node: the hand doesn't move, so its pose is
    averaged and frames where its detection jumps are ignored. --raw-hand uses each frame's own pose.
    """

    def __init__(self, topic, tracker, raw_hand=False):
        super().__init__("measure_cube")
        self.tracker = tracker
        self.raw_hand = raw_hand
        self.hand = HandRootAverage()
        self.lock = threading.Lock()
        self.collecting = False
        self.results = []      # (T_root_cube_mm, fit_px)
        self.seen = 0          # frames processed while collecting
        self.missing = {}      # reason -> count
        self.frames_in = 0
        self.create_subscription(Image, topic, self.on_image, 1)

    def on_image(self, msg):
        self.frames_in += 1
        with self.lock:
            if not self.collecting:
                return
        est = self.tracker.estimate(image_to_bgr(msg))
        with self.lock:
            self.seen += 1
            T_cam_root = est.T_cam_root if self.raw_hand else self.hand.update(est.T_cam_root)
            if est.cube_ok and T_cam_root is not None:
                self.results.append((np.linalg.inv(T_cam_root) @ est.T_cam_cube, est.fit_error_px))
            else:
                reason = "hand marker 10 not visible" if est.cube_ok else est.reason
                self.missing[reason] = self.missing.get(reason, 0) + 1

    def collect(self, n, timeout_s):
        with self.lock:
            self.results, self.seen, self.missing = [], 0, {}
            self.collecting = True
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            with self.lock:
                if len(self.results) >= n:
                    break
            time.sleep(0.05)
        with self.lock:
            self.collecting = False
            return list(self.results), self.seen, dict(self.missing)


def summarise(results):
    pos = np.array([T[:3, 3] for T, _ in results])
    rots = R.from_matrix([T[:3, :3] for T, _ in results])
    mean_rot = rots.mean()
    spread_deg = np.degrees((rots * mean_rot.inv()).magnitude())
    return {
        "pos": pos.mean(axis=0),
        "pos_std": pos.std(axis=0),
        "rot": mean_rot,
        "rot_std_deg": float(np.sqrt(np.mean(spread_deg ** 2))),
        "fit": float(np.mean([f for _, f in results])),
        "n": len(results),
    }


def surface_tilt(rot):
    """Tilt of the surface the cube rests on, relative to marker 10's surface, in deg.

    Uses the cube axis closest to marker 10's z axis (the cube's 'up', whichever face is on top).
    forward: tilt around x; negative = the surface drops toward the fingertips (like the palm).
    sideways: tilt around y; positive = the surface drops toward the little finger.
    """
    axes = rot.as_matrix()                      # columns: cube x, y, z axes in the hand frame
    k = int(np.argmax(np.abs(axes[2, :])))
    up = axes[:, k] * np.sign(axes[2, k])       # flip so it points out of marker 10's side
    forward = -np.degrees(np.arctan2(up[1], up[2]))
    sideways = -np.degrees(np.arctan2(up[0], up[2]))
    return forward, sideways


def vec(v):
    return f"({v[0]:7.1f}, {v[1]:7.1f}, {v[2]:7.1f})"


def compare(name, a, b):
    d = b["pos"] - a["pos"]
    d_deg = np.degrees((b["rot"] * a["rot"].inv()).magnitude())
    # Slope of the move relative to marker 10's surface: e.g. sliding down a palm tilted 20 deg gives -20.
    slope = np.degrees(np.arctan2(d[2], np.hypot(d[0], d[1])))
    # Heading of the move across marker 10's surface: 0 = toward the fingertips (+y),
    # +90 = toward the little finger (+x), -90 = the other side, 180 = toward the elbow.
    heading = np.degrees(np.arctan2(d[0], d[1]))
    # Turn between the two, split over the hand axes (exact when the cube turned around one axis).
    about = np.degrees((b["rot"] * a["rot"].inv()).as_rotvec())
    print(f"    vs {name:12s} moved {vec(d)} mm   distance {np.linalg.norm(d):6.1f} mm   "
          f"slope {slope:+5.1f} deg   heading {heading:+6.1f} deg   turned {d_deg:5.1f} deg")
    print(f"    {'':15s} turned about hand x {about[0]:+6.1f}   y {about[1]:+6.1f}   z {about[2]:+6.1f} deg")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--frames", type=int, default=60, help="frames averaged per measurement")
    parser.add_argument("--timeout", type=float, default=15.0, help="seconds to wait for enough good frames")
    parser.add_argument("--intrinsics", default=os.path.join(PKG_DIR, "config", "camera_intrinsics.npz"))
    parser.add_argument("--faces", default=os.path.join(PKG_DIR, "config", "cube_faces.json"))
    parser.add_argument("--side-mm", type=float, default=50.0, help="cube side")
    parser.add_argument("--marker-mm", type=float, default=40.0, help="black square of the cube face markers")
    parser.add_argument("--hand-marker-mm", type=float, default=40.0, help="black square of hand marker 10")
    parser.add_argument("--out", help="append each measurement to this CSV file")
    parser.add_argument("--raw-hand", action="store_true",
                        help="use marker 10's pose from each frame instead of the running average")
    args = parser.parse_args()

    tracker = CubeTracker(args.intrinsics, args.faces, side_mm=args.side_mm, marker_mm=args.marker_mm,
                          hand_marker_mm=args.hand_marker_mm)
    rclpy.init()
    node = Collector(args.topic, tracker, raw_hand=args.raw_hand)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    time.sleep(1.5)
    if node.frames_in == 0:
        print(f"No images on {args.topic} yet. Is the camera running? (ros2 launch cube_perception camera.launch.py)")

    print("Axes: x = little finger, y = fingertips, z = out of marker 10's surface. Units mm.")
    print("Type a label (e.g. P1) and press Enter to measure. Empty label = auto. 'q' to quit.\n")
    measured = []  # (label, summary)
    try:
        while True:
            try:
                label = input("label> ").strip()
            except EOFError:
                break
            if label.lower() in ("q", "quit", "exit"):
                break
            label = label or f"P{len(measured) + 1}"

            results, seen, missing = node.collect(args.frames, args.timeout)
            if not results:
                why = ", ".join(f"{k} ({v})" for k, v in missing.items()) or "no frames received"
                print(f"  {label}: no measurement. {why}\n")
                continue
            s = summarise(results)
            roll, pitch, yaw = s["rot"].as_euler("xyz", degrees=True)
            print(f"  {label}: cube centre {vec(s['pos'])} mm   jitter (std) {vec(s['pos_std'])} mm")
            print(f"    rotation rpy ({roll:.1f}, {pitch:.1f}, {yaw:.1f}) deg   jitter {s['rot_std_deg']:.2f} deg"
                  f"   fit {s['fit']:.2f} px   {s['n']}/{seen} frames usable")
            fwd, side = surface_tilt(s["rot"])
            print(f"    surface under the cube (if it sits flat): forward tilt {fwd:+.1f} deg, "
                  f"sideways tilt {side:+.1f} deg   (vs marker 10's surface)")
            if len(results) < args.frames:
                print(f"    only {len(results)} of {args.frames} frames: {missing}")
            if measured:
                compare(measured[0][0], measured[0][1], s)
                if len(measured) > 1:
                    compare(measured[-1][0], measured[-1][1], s)
            print()
            measured.append((label, s))

            if args.out:
                new = not os.path.exists(args.out)
                with open(args.out, "a", newline="") as f:
                    w = csv.writer(f)
                    if new:
                        w.writerow(["time", "label", "x_mm", "y_mm", "z_mm", "x_std", "y_std", "z_std",
                                    "roll_deg", "pitch_deg", "yaw_deg", "rot_std_deg", "fit_px", "frames"])
                    w.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), label, *np.round(s["pos"], 2),
                                *np.round(s["pos_std"], 2), round(roll, 2), round(pitch, 2), round(yaw, 2),
                                round(s["rot_std_deg"], 3), round(s["fit"], 3), s["n"]])
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()  # stops the spin thread first, then the node can go
        spinner.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
