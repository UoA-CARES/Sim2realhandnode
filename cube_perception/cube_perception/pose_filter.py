"""Gap handling, outlier rejection and optional smoothing of the cube pose. No ROS.

The OpenAI policy doesn't use cube velocity, so this only cleans up the pose:
  - a measurement that jumps too far from the last good pose in one step is ignored (outlier),
    unless `reset_after` of them in a row agree with each other (the cube really moved);
  - while the cube is hidden, the last pose is held for up to `max_hold_s` (status "predicting"),
    then there is no pose (status "lost");
  - optional smoothing: exponential average of position, slerp of rotation (`alpha` = weight of the
    new measurement; 1 = no smoothing).

    f = PoseFilter()
    pose, status = f.update(t, T)     # t: image stamp in s; T: 4x4 cube pose (m) or None
    # pose: 4x4 or None; status: "tracking" | "predicting" | "lost"
"""
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp


def _distance(T_a, T_b):
    """(position difference in m, rotation difference in deg)."""
    d_m = np.linalg.norm(T_a[:3, 3] - T_b[:3, 3])
    d_deg = np.degrees((R.from_matrix(T_a[:3, :3]) * R.from_matrix(T_b[:3, :3]).inv()).magnitude())
    return d_m, d_deg


class PoseFilter:
    def __init__(self, max_hold_s=0.2, max_jump_m=0.020, max_jump_deg=30.0, reset_after=3,
                 smoothing=False, alpha=0.5):
        self.max_hold_s = max_hold_s
        self.max_jump_m = max_jump_m
        self.max_jump_deg = max_jump_deg
        self.reset_after = reset_after
        self.smoothing = smoothing
        self.alpha = alpha
        self.T = None        # last accepted (and smoothed) pose
        self.t_last = None   # stamp of the last accepted measurement
        self.outliers = []
        self.rejected = 0    # outliers ignored since start (for the log)

    def _far(self, T_a, T_b):
        d_m, d_deg = _distance(T_a, T_b)
        return d_m > self.max_jump_m or d_deg > self.max_jump_deg

    def _smooth(self, T_new):
        if not self.smoothing or self.T is None:
            return T_new.copy()
        T = np.eye(4)
        T[:3, 3] = (1 - self.alpha) * self.T[:3, 3] + self.alpha * T_new[:3, 3]
        rots = R.from_matrix([self.T[:3, :3], T_new[:3, :3]])
        T[:3, :3] = Slerp([0.0, 1.0], rots)(self.alpha).as_matrix()
        return T

    def _accept(self, t, T_meas):
        self.T = self._smooth(T_meas)
        self.t_last = t
        self.outliers = []

    def _held(self, t):
        return self.T is not None and t - self.t_last <= self.max_hold_s

    def update(self, t, T_meas):
        """t: image stamp (s). T_meas: measured 4x4 pose (m), or None if not measured this frame."""
        if T_meas is not None:
            if not self._held(t):
                # Nothing recent to compare with (start, or after "lost"): take it as it is.
                self.T = None
                self._accept(t, T_meas)
                return self.T, "tracking"
            if not self._far(T_meas, self.T):
                self._accept(t, T_meas)
                return self.T, "tracking"
            # Too far from the last pose. Real move if the last few outliers agree with each other.
            if self.outliers and self._far(T_meas, self.outliers[-1]):
                self.outliers = []
            self.outliers.append(T_meas)
            if len(self.outliers) >= self.reset_after:
                self.T = None
                self._accept(t, T_meas)
                return self.T, "tracking"
            self.rejected += 1

        if self._held(t):
            return self.T, "predicting"
        return None, "lost"

    def reset(self):
        self.T = None
        self.t_last = None
        self.outliers = []
