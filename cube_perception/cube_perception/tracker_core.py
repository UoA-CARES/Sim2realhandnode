"""Cube pose from STag markers, relative to the hand marker. No ROS, no camera, no GUI.

Moved from cube_tracker.py (detection, cube model, joint fit). Works in mm internally so the
numbers stay comparable with cube_tracker.py; use Estimate.T_root_cube_m() for ROS (metres).

    tracker = CubeTracker("config/camera_intrinsics.npz", "config/cube_faces.json")
    est = tracker.estimate(bgr_or_grey_image)
    if est.valid:
        T = est.T_root_cube_m()   # 4x4, cube pose in hand_root, metres
"""
import json
from collections import deque
from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

FACE_NAMES = {6: "top", 1: "bottom", 3: "front", 4: "back", 5: "left", 2: "right"}


def marker_obj_pts(size_mm):
    # Same corner order as validate_stag.py; marker z points out of the surface.
    h = size_mm / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)


def to_T(R_, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R_, t
    return T


def solve_marker(img_pts, camera_matrix, dist_coeffs, size_mm):
    ok, rvec, tvec = cv2.solvePnP(marker_obj_pts(size_mm), img_pts.reshape(4, 1, 2).astype(np.float32),
                                  camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    return cv2.Rodrigues(rvec)[0], tvec.flatten()


def default_marker_centre(R_cube_marker, side_mm):
    """Marker centre in the cube frame (mm) when it sits exactly in the middle of its face."""
    return R_cube_marker[:, 2] * side_mm / 2.0


def cube_pose_from_face(R_cam_marker, t_cam_marker, R_cube_marker, t_cube_marker):
    """Cube pose from one face marker. R/t_cube_marker: the marker's pose in the cube frame."""
    R_cam_cube = R_cam_marker @ R_cube_marker.T
    center = t_cam_marker - R_cam_cube @ t_cube_marker
    return R_cam_cube, center


def cube_corner_model(learned, centres, marker_mm):
    """{marker_id: 4x3 marker corner positions in the cube frame (mm)}.
    learned: {id: R_cube_marker}; centres: {id: marker centre in the cube frame (mm)}."""
    return {mid: (Rcm @ marker_obj_pts(marker_mm).T).T + centres[mid] for mid, Rcm in learned.items()}


def solve_cube_jointly(faces, corner_model, R_init, t_init, camera_matrix, dist_coeffs):
    """One PnP over every visible face's corners. Returns R_cam_cube, centre_mm, RMS reprojection error (px)."""
    ids = [m for m in faces if m in corner_model]
    obj = np.vstack([corner_model[m] for m in ids]).astype(np.float64)
    img = np.vstack([faces[m][2] for m in ids]).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img.reshape(-1, 1, 2), camera_matrix, dist_coeffs,
                                  rvec=cv2.Rodrigues(R_init)[0], tvec=np.array(t_init, dtype=np.float64).reshape(3, 1),
                                  useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    return cv2.Rodrigues(rvec)[0], tvec.flatten(), rms


def fuse(estimates):
    """estimates: list of (R_cam_cube, center_mm, weight). Returns fused R, center, spread_mm, spread_deg."""
    w = np.clip([e[2] for e in estimates], 1e-3, None)
    center = np.average([e[1] for e in estimates], axis=0, weights=w)
    rot = R.from_matrix([e[0] for e in estimates]).mean(weights=w)
    spread_mm = max(np.linalg.norm(e[1] - center) for e in estimates)
    spread_deg = max(np.degrees((R.from_matrix(e[0]) * rot.inv()).magnitude()) for e in estimates)
    return rot.as_matrix(), center, spread_mm, spread_deg


@dataclass
class Estimate:
    """Result of one image. Poses are 4x4 in mm; None where not seen."""
    T_cam_root: np.ndarray | None = None   # hand marker (hand_root) in the camera frame
    T_cam_cube: np.ndarray | None = None   # cube centre in the camera frame (after the joint fit)
    fit_error_px: float = float("nan")     # RMS reprojection error of the joint fit
    face_ids: list = field(default_factory=list)
    spread_mm: float = 0.0                 # disagreement between single-face estimates (before the joint fit)
    spread_deg: float = 0.0
    reason: str = ""                       # why the cube pose is missing / rejected, if it is

    @property
    def cube_ok(self):
        return self.T_cam_cube is not None

    @property
    def valid(self):
        """Cube and hand marker both available, so T_root_cube can be computed."""
        return self.T_cam_cube is not None and self.T_cam_root is not None

    def T_root_cube(self):
        """Cube pose in hand_root, mm. Same as cube_tracker.py's 'CUBE from root'."""
        return np.linalg.inv(self.T_cam_root) @ self.T_cam_cube

    def T_root_cube_m(self):
        T = self.T_root_cube()
        T[:3, 3] /= 1000.0
        return T


class CubeTracker:
    def __init__(self, intrinsics_path, faces_path, side_mm=50.0, marker_mm=40.0,
                 hand_marker_id=10, hand_marker_mm=40.0, hd=11, max_fit_error_px=3.0):
        intrinsics = np.load(intrinsics_path)
        self.camera_matrix = intrinsics["camera_matrix"]
        self.dist_coeffs = intrinsics["dist_coeffs"]
        with open(faces_path) as f:
            faces = json.load(f)["faces"]
        # R_cube_marker: how each marker is turned on its face (exact if refined by learn_faces.py,
        # whole quarter turns if learned by cube_tracker.py --learn).
        # t_cube_marker_mm (refined files only): where its centre is; otherwise the middle of the face.
        self.learned = {int(k): np.array(v["R_cube_marker"], dtype=float) for k, v in faces.items()}
        self.centres = {int(k): (np.array(v["t_cube_marker_mm"], dtype=float) if "t_cube_marker_mm" in v
                                 else default_marker_centre(self.learned[int(k)], side_mm))
                        for k, v in faces.items()}
        self.side_mm = side_mm
        self.marker_mm = marker_mm
        self.hand_marker_id = hand_marker_id
        self.hand_marker_mm = hand_marker_mm
        self.hd = hd
        self.max_fit_error_px = max_fit_error_px
        self.corner_model = cube_corner_model(self.learned, self.centres, marker_mm)

    def detect_markers(self, image):
        """{marker_id: (R_cam_marker, t_cam_marker_mm, image_corners)} for cube faces and the hand marker."""
        import stag
        corners, ids, _ = stag.detectMarkers(image, self.hd)
        found = {}
        if ids is None:
            return found
        for c, mid in zip(corners, np.asarray(ids).flatten()):
            mid = int(mid)
            if mid in FACE_NAMES:
                size = self.marker_mm
            elif mid == self.hand_marker_id:
                size = self.hand_marker_mm
            else:
                continue
            img_pts = np.asarray(c, dtype=np.float32).reshape(4, 2)
            sol = solve_marker(img_pts, self.camera_matrix, self.dist_coeffs, size)
            if sol is not None:
                found[mid] = (sol[0], sol[1], img_pts)
        return found

    def estimate(self, image):
        """image: BGR or greyscale (convert ROS rgb8 first). Returns an Estimate."""
        return self.estimate_from_markers(self.detect_markers(image))

    def estimate_from_markers(self, markers):
        """Same as estimate(), from detect_markers() output."""
        est = Estimate()

        if self.hand_marker_id in markers:
            R_hm, t_hm, _ = markers[self.hand_marker_id]
            est.T_cam_root = to_T(R_hm, t_hm)

        faces = {m: v for m, v in markers.items() if m in self.learned}
        if not faces:
            est.reason = "no cube face visible"
            return est
        est.face_ids = sorted(faces)

        # Single-face estimates, fused: the starting guess for the joint fit.
        estimates = []
        for mid, (R_cm, t_cm, _) in faces.items():
            R_cam_cube, center = cube_pose_from_face(R_cm, t_cm, self.learned[mid], self.centres[mid])
            facing = max(-R_cm[2, 2], 0.0)  # 1 = marker faces the camera head-on
            estimates.append((R_cam_cube, center, facing))
        R_init, t_init, est.spread_mm, est.spread_deg = fuse(estimates)

        joint = solve_cube_jointly(faces, self.corner_model, R_init, t_init, self.camera_matrix, self.dist_coeffs)
        if joint is None:
            est.reason = "joint fit failed"
            return est
        R_cam_cube, center, est.fit_error_px = joint
        if est.fit_error_px > self.max_fit_error_px:
            est.reason = f"fit error {est.fit_error_px:.2f} px > {self.max_fit_error_px} px"
            return est
        est.T_cam_cube = to_T(R_cam_cube, center)
        return est


class HandRootAverage:
    """Running average of T_cam_root. The hand base doesn't move, so averaging removes the marker's
    jitter, and the average is still there while a finger hides the marker.

    A measurement far from the average is ignored, unless `reset_after` of them in a row agree
    with each other: then the camera (or hand) really moved, and the average restarts.
    """

    def __init__(self, window=60, max_jump_mm=5.0, max_jump_deg=3.0, reset_after=10):
        self.window = window
        self.max_jump_mm = max_jump_mm
        self.max_jump_deg = max_jump_deg
        self.reset_after = reset_after
        self.samples = deque(maxlen=window)
        self.outliers = []
        self.T = None

    def _far(self, T_a, T_b, scale=1.0):
        d_mm = np.linalg.norm(T_a[:3, 3] - T_b[:3, 3])
        d_deg = np.degrees((R.from_matrix(T_a[:3, :3]) * R.from_matrix(T_b[:3, :3]).inv()).magnitude())
        return d_mm > scale * self.max_jump_mm or d_deg > scale * self.max_jump_deg

    def _recompute(self):
        pos = np.mean([T[:3, 3] for T in self.samples], axis=0)
        rot = R.from_matrix([T[:3, :3] for T in self.samples]).mean().as_matrix()
        self.T = to_T(rot, pos)

    def update(self, T_cam_root):
        """Add a measurement (or None if the marker isn't visible). Returns the current average, or None."""
        if T_cam_root is None:
            return self.T
        if self.T is not None and self._far(T_cam_root, self.T):
            # Two noisy readings differ by up to twice the noise of one, hence the looser check.
            if self.outliers and self._far(T_cam_root, self.outliers[0], scale=2.0):
                self.outliers = []  # outliers disagree with each other: noise, not a move
            self.outliers.append(T_cam_root)
            if len(self.outliers) >= self.reset_after:
                self.samples = deque(self.outliers, maxlen=self.window)
                self.outliers = []
                self._recompute()
            return self.T
        self.outliers = []
        self.samples.append(T_cam_root)
        self._recompute()
        return self.T

    def reset(self):
        self.samples.clear()
        self.outliers = []
        self.T = None
