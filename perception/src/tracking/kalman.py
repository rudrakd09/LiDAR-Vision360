"""Constant-velocity Kalman filter for one track's `(x, y, vx, vy)` state.

State vector: ``[x, y, vx, vy]^T``. Motion model between scans (dt seconds apart):

    x' = x + vx*dt
    y' = y + vy*dt
    vx' = vx
    vy' = vy

plus process noise modeling unmodeled acceleration on each axis (discrete white-noise-
acceleration / "DWNA" model -- see docs/tracking.md "Kalman filter" for the full derivation of
the state-transition matrix `F` and process-noise matrix `Q` below). The only thing ever measured
is position `(x, y)` -- a cluster's centroid has no direct velocity sensor -- so velocity is
purely inferred by the filter from how position changes across updates.

Independent of `models`, `simulator`, and every other pipeline stage: this is a small, generic,
directly-unit-testable 2D CV Kalman filter, reusable outside this project's specific data models.
"""

from __future__ import annotations

import numpy as np


class KalmanFilter2D:
    """A single constant-velocity Kalman filter tracking one object's `(x, y, vx, vy)`.

    All noise/uncertainty parameters are constructor arguments (see `common.config.Settings`'s
    `tracking_*` fields for this project's defaults and the reasoning behind them) -- nothing
    here is hard-coded.
    """

    def __init__(
        self,
        x: float,
        y: float,
        process_noise_std: float,
        measurement_noise_std_m: float,
        initial_position_uncertainty_m: float,
        initial_velocity_uncertainty_mps: float,
        vx: float = 0.0,
        vy: float = 0.0,
    ) -> None:
        self.state = np.array([x, y, vx, vy], dtype=np.float64)

        # Initial state covariance P: independent, diagonal uncertainty on position and velocity
        # -- there is no prior correlation between them until the filter has seen an update.
        self.P = np.diag(
            [
                initial_position_uncertainty_m ** 2,
                initial_position_uncertainty_m ** 2,
                initial_velocity_uncertainty_mps ** 2,
                initial_velocity_uncertainty_mps ** 2,
            ]
        ).astype(np.float64)

        self.process_noise_std = process_noise_std

        # Measurement model: only (x, y) is observed, never (vx, vy) directly.
        self.H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float64)
        self.R = np.diag([measurement_noise_std_m ** 2, measurement_noise_std_m ** 2]).astype(np.float64)

    @property
    def x(self) -> float:
        return float(self.state[0])

    @property
    def y(self) -> float:
        return float(self.state[1])

    @property
    def vx(self) -> float:
        return float(self.state[2])

    @property
    def vy(self) -> float:
        return float(self.state[3])

    def predict(self, dt: float) -> tuple[float, float]:
        """Advance the filter's own state/covariance by `dt` seconds (constant-velocity motion +
        process noise) and return the resulting `(x, y)`. Mutates the filter -- call once per
        scan, per track, before associating/updating.
        """
        dt = max(dt, 0.0)
        F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        # Discrete white-noise-acceleration process noise, one independent block per axis (the
        # standard closed-form Q for a constant-velocity model driven by white acceleration noise
        # of variance q = process_noise_std**2): a nonzero position/velocity cross-covariance term
        # models that an unmodeled acceleration over this dt affects both consistently, not
        # independently.
        q = self.process_noise_std ** 2
        dt2, dt3, dt4 = dt ** 2, dt ** 3, dt ** 4
        Q = q * np.array(
            [
                [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
                [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
                [dt3 / 2.0, 0.0, dt2, 0.0],
                [0.0, dt3 / 2.0, 0.0, dt2],
            ],
            dtype=np.float64,
        )

        self.state = F @ self.state
        self.P = F @ self.P @ F.T + Q
        return self.x, self.y

    def update(self, x_meas: float, y_meas: float) -> None:
        """Measurement-update (correction) step given an observed `(x, y)` position. Mutates the
        filter -- call once per scan, per track, only when a detection was associated to it.
        """
        z = np.array([x_meas, y_meas], dtype=np.float64)
        innovation = z - self.H @ self.state
        S = self.H @ self.P @ self.H.T + self.R
        kalman_gain = self.P @ self.H.T @ np.linalg.inv(S)

        self.state = self.state + kalman_gain @ innovation
        identity = np.eye(4)
        self.P = (identity - kalman_gain @ self.H) @ self.P

    def peek_predict(self, dt: float) -> tuple[float, float]:
        """Return the position the constant-velocity model would reach `dt` seconds ahead of the
        *current* state, without mutating `state`/`P`. Used to report a "next position" prediction
        alongside this scan's own corrected estimate, distinct from `predict()`, which actually
        advances the filter for the next scan's association/update cycle.
        """
        dt = max(dt, 0.0)
        return float(self.state[0] + self.state[2] * dt), float(self.state[1] + self.state[3] * dt)
