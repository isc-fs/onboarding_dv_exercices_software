"""
Dead-reckoning odometry filter for the DV pipeline.

Hosted by sim_supervisor_node in sim; the same algorithm runs on the
real-car uDV firmware (separate codebase, identical input/output
contract). Inputs are the two signals available on both sides:

  * IMU (BMI088-class) — accel + gyro at native rate (~400 Hz on the
    sim's BMI088 model; whatever the real IMU runs at on the car).
    Used for prediction (vx integration via accel-x, yaw integration
    via gyro-z) and for stationary bias calibration.
  * Motor RPM (~80 Hz) — primary longitudinal velocity correction.
    Multiplied by RPM_TO_MS (the 2026-04-28 empirical constant from
    the GT-comparison diagnostic) to yield body-frame longitudinal
    speed at the rear axle.

GSS is **not** an input on either side: the real IFS-08 doesn't have a
ground-speed sensor. The simulator does publish /fsds/gss but we
deliberately ignore it here so the algorithm transfers cleanly.

Steering angle and brake pressure are listed as future cross-checks
(kinematic-bicycle yaw-rate residual; brake-event slip detection)
but are not consumed in the MVP — neither is exposed on the sim
bridge surface today.

Output is a 6-state body-frame estimate:
  * pose:  x, y, yaw  (world-frame, integrated from spawn)
  * twist: vx, vy, yaw_rate  (body-frame instantaneous)

Sim_supervisor publishes this as nav_msgs/Odometry on /odom at 100 Hz
(decoupled from the IMU's native 400 Hz — the integration runs at IMU
rate but publication is timer-driven; see push_imu vs the publisher
loop). On the real car, the uDV publishes the same topic over
microROS/USB-CDC at whatever rate its firmware produces.

Filter: complementary (predict on IMU, correct on RPM). The
parameters (alpha_vx, beta_vy_decay, calibration window) are tuned
against the existing slam_node motor-RPM velocity prior; revisit
once we have GT-aligned residual data on a few real drives.

Open question — see docs/AUTONOMY.md §"Open questions" Q2:
the algorithm currently ingests IMU at full 400 Hz native rate. There
is a follow-up to evaluate whether downsampling to 100 Hz (matching
the publish rate) loses meaningful filter quality. Bias estimation
during the 3 s stationary window benefits from full-rate sampling;
the steady-state predict step likely doesn't need it. Quantify
before tightening.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


# Motor-RPM → body-frame longitudinal velocity.
#
# 2026-05-10 re-derivation (issue #380): bagged /odom and
# /testing_only/odom over a 41 s motion window on test_submodule.csv,
# paired GT vs filter-output samples within ±50 ms windows, computed
# mean(|GT.vx|) / mean(|/odom.vx|) = 0.9140 across 3324 paired
# samples (p10=0.898, p90=0.944, spread 0.046 — tight, steady).
# Implies CURRENT 0.00898 produces a +9.4 % vx overestimate. New
# constant 0.00898 × 0.9140 = 0.00821.
#
# This recovers exactly the doc-derived value from
# docs/dv_pipeline_rebuild.md §3.5:
#     RPM_TO_MS = (2π × WheelRadius / GearRatio) / 60
#               = (2π × 0.228   / 2.909      ) / 60
#               = 0.00821
# i.e. (2π·r_wheel) / (gearratio·60) — pure geometry. The 2026-04-28
# adjustment to 0.00898 measured raw RPM×const against GT (no filter
# in between); the lap bag this time measured filter-output vs GT,
# which is the closed-loop quantity that actually matters for /odom
# users. Both are valid measurements; the closed-loop one wins for
# consumer-facing accuracy.
#
# slam_node uses this same constant as a velocity prior in iSAM2;
# the prior factor's covariance lets cone observations correct the
# small residual bias, so reverting to 0.00821 doesn't hurt SLAM
# either (its observed 4.6 % path-length error in #378 was dominated
# by yaw drift and DA cascades, not the velocity prior magnitude).
RPM_TO_MS: float = 0.00821

# Drop motor-RPM samples this old (seconds). Sustained staleness means
# the bridge stopped publishing; fall back to IMU-only prediction
# rather than feeding a stale velocity correction.
RPM_STALE_S: float = 0.5

# Stationary-calibration window. While the supervisor is collecting
# IMU samples the filter does not publish — the autonomy stack tolerates
# brief startup delay (it's already inside Phase 1's "configuring"
# stage). Same value slam_node uses for its own bias calibration so
# behaviour is consistent if both run side-by-side.
CALIBRATION_SECONDS: float = 3.0

# Complementary-filter blend on vx. RPM correction strength per RPM
# message. 0.10 means each new RPM sample pulls vx by 10 % of the
# residual. With RPM at ~80 Hz that gives an effective time constant
# of ~125 ms — fast enough to track real accel/decel, slow enough to
# average out RPM quantisation noise.
ALPHA_VX: float = 0.10

# Phase 3 (#383) — wheelbase used in the kinematic-bicycle yaw
# prediction:
#     ω_pred = (v_x / wheelbase) · tan(δ)
# Compared against IMU gyro_z to publish a yaw residual diagnostic +
# detect lateral-slip events. 1.570 m is the authoritative IFS-08 spec
# (docs/MODEL_IFS_08/DYNAMIC_MOD/Cooling/BR_regenerativeBR.m: `L=1.570`
# + docs/MODEL_IFS_08/ISC_IFS_08.xlsx MONO sheet BQ64: `L=1570 mm`).
# Previous value 1.55 was URDF-sourced (approximate). See issue #462.
# Kept in sync with kWheelbaseM in odometry_filter.hpp (C++ port).
WHEELBASE_M: float = 1.570

# Brake-pressure threshold above which RPM is considered unreliable
# (drive wheels potentially locked). When exceeded the complementary
# filter's α_vx is scaled toward zero so vx integration tracks the
# IMU prediction rather than the RPM-derived target. 0.30 (30 % brake
# command) is a deliberately conservative floor — light braking
# during throttle-off doesn't trigger fallback; only meaningful
# brake authority does.
BRAKE_LOCKUP_THRESHOLD: float = 0.30

# α_vx multiplier when brake pressure exceeds BRAKE_LOCKUP_THRESHOLD.
# 0.0 = ignore RPM entirely during brake events; 1.0 = trust RPM as
# normal. 0.05 keeps a small residual pull toward RPM (so the filter
# eventually re-converges after the brake event) but lets IMU
# integration dominate during the slip window.
ALPHA_VX_BRAKE: float = 0.05

# Maximum allowed RPM-vs-steering-prediction disagreement before a
# scan is flagged as a slip event. The OdometryFilter doesn't act on
# the flag itself (that's for downstream consumers + diagnostics); it
# just publishes the residual on /odom_diag/yaw_residual via the
# supervisor. 0.3 rad/s ≈ 17 °/s — comfortably above sensor noise
# but well below the steering-angle-range × wheelbase product would
# imply at typical FS speeds.
SLIP_YAW_RESIDUAL_THRESHOLD: float = 0.3

# Body-frame lateral-velocity decay per IMU step. The kinematic
# bicycle assumes vy ≈ 0 in clean rolling regimes; this slow leak
# pulls vy toward zero, with the IMU accel-y prediction term still
# free to track real lateral accel during yaw maneuvers. 1e-3 per
# IMU sample at 400 Hz → ~2.5 s time constant.
BETA_VY_LEAK: float = 1e-3

# Gravity magnitude (sim's BMI088 model uses standard gravity for
# the accel offset; the real BMI088 reads ~9.806).
G: float = 9.81


@dataclass
class OdometryState:
    """The 6-DOF body-frame state /odom carries."""

    # World-frame position, integrated from spawn (drifts long-term)
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    # Body-frame velocity (vx forward, vy left in REP-103)
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0


@dataclass
class FilterDiagnostics:
    """Phase 3 (#383) — cross-check residuals the supervisor surfaces
    on /odom_diag/* for tuning + slip detection. Populated each IMU
    tick when both steering and IMU samples are fresh.
    """

    # ω_pred - ω_measured, where ω_pred = (vx/L)·tan(δ).
    # Persistent non-zero residual = car is sliding (yaw rate from
    # IMU differs from what steering geometry would predict).
    yaw_residual_rad_s: float = 0.0

    # True iff |yaw_residual| > SLIP_YAW_RESIDUAL_THRESHOLD on the
    # latest tick. Sticky-latched would be a nicer downstream signal;
    # this is the raw per-tick truth (downstream consumers can debounce).
    slip_flag: bool = False

    # Effective α_vx applied to the most recent RPM correction. Drops
    # toward ALPHA_VX_BRAKE when brake_pressure > threshold.
    effective_alpha_vx: float = ALPHA_VX


@dataclass
class _Calibration:
    """Bias-estimation accumulators, populated during the stationary
    calibration window then frozen."""

    n_samples: int = 0
    accel_sum: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro_sum: np.ndarray = field(default_factory=lambda: np.zeros(3))
    t_first: Optional[float] = None
    completed: bool = False

    accel_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))


class OdometryFilter:
    """Complementary filter — IMU prediction + RPM correction.

    Lifecycle:
      1. Construction: state is zero, calibration not started.
      2. push_imu() during first CALIBRATION_SECONDS s: accumulate
         bias estimates (assumes the car is stationary, accel reads
         (0, 0, +g_body) modulo bias, gyro reads (0, 0, 0) modulo
         bias). Filter output is undefined; is_calibrated() returns
         False.
      3. After calibration: push_imu() drives prediction (integrate
         accel-x into vx, accel-y into vy with leak, gyro-z into
         yaw). push_rpm() applies the correction step.
      4. state property always returns the latest estimate.

    Parameters keep their module-level defaults unless overridden in
    the constructor — convenient for unit tests that want to bypass
    calibration or use deterministic values.
    """

    def __init__(
        self,
        *,
        rpm_to_ms: float = RPM_TO_MS,
        rpm_stale_s: float = RPM_STALE_S,
        calibration_seconds: float = CALIBRATION_SECONDS,
        alpha_vx: float = ALPHA_VX,
        beta_vy_leak: float = BETA_VY_LEAK,
        wheelbase_m: float = WHEELBASE_M,
        brake_lockup_threshold: float = BRAKE_LOCKUP_THRESHOLD,
        alpha_vx_brake: float = ALPHA_VX_BRAKE,
        slip_yaw_residual_threshold: float = SLIP_YAW_RESIDUAL_THRESHOLD,
    ) -> None:
        self._rpm_to_ms = rpm_to_ms
        self._rpm_stale_s = rpm_stale_s
        self._calibration_seconds = calibration_seconds
        self._alpha_vx = alpha_vx
        self._beta_vy_leak = beta_vy_leak
        self._wheelbase_m = wheelbase_m
        self._brake_lockup_threshold = brake_lockup_threshold
        self._alpha_vx_brake = alpha_vx_brake
        self._slip_yaw_residual_threshold = slip_yaw_residual_threshold

        self._state = OdometryState()
        self._calib = _Calibration()
        self._diag = FilterDiagnostics(effective_alpha_vx=alpha_vx)

        # Last-seen timestamps. Wall-clock for RPM staleness, IMU
        # timestamp for integration dt.
        self._t_imu_last: Optional[float] = None
        self._t_rpm_last: Optional[float] = None
        self._latest_rpm_vx: Optional[float] = None

        # Phase 3 (#383) inputs — latest steering angle (rad) +
        # brake authority [0, 1]. Updated by push_steering / push_brake
        # from the bridge's /fsds/steering_angle and /fsds/brake_pressure
        # topics. Default zero is the safe assumption when no samples
        # have arrived yet (straight-ahead, no brake).
        self._latest_steering_rad: float = 0.0
        self._latest_brake: float = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    @property
    def state(self) -> OdometryState:
        return self._state

    @property
    def diagnostics(self) -> FilterDiagnostics:
        """Phase 3 cross-check residuals (yaw residual + slip flag +
        effective α_vx). Published by sim_supervisor on /odom_diag/*."""
        return self._diag

    def is_calibrated(self) -> bool:
        return self._calib.completed

    def reset(self) -> None:
        """Tear down all state. Used on lifecycle on_cleanup."""
        self._state = OdometryState()
        self._calib = _Calibration()
        self._diag = FilterDiagnostics(effective_alpha_vx=self._alpha_vx)
        self._t_imu_last = None
        self._t_rpm_last = None
        self._latest_rpm_vx = None
        self._latest_steering_rad = 0.0
        self._latest_brake = 0.0

    def push_imu(
        self,
        t: float,
        accel: np.ndarray,
        gyro: np.ndarray,
    ) -> None:
        """Ingest one IMU sample. Called from the IMU subscription
        callback at the IMU's native rate (~400 Hz on BMI088).

        accel, gyro are length-3 numpy arrays in body frame. accel is
        m/s² with gravity included (so a stationary upright IMU reads
        ~(0, 0, +9.81)); gyro is rad/s.
        """
        if not self._calib.completed:
            self._accumulate_calibration(t, accel, gyro)
            return

        # Integration dt — clamp to a sane bound so a clock glitch
        # (e.g. sim time discontinuity at refresh-bridge) doesn't
        # produce a one-tick velocity spike.
        if self._t_imu_last is None:
            self._t_imu_last = t
            return
        dt = t - self._t_imu_last
        self._t_imu_last = t
        if dt <= 0.0 or dt > 0.1:
            return

        # Bias-corrected readings
        a_body = accel - self._calib.accel_bias
        w_body = gyro - self._calib.gyro_bias

        # Yaw rate is gyro-z directly (no integration step needed —
        # we read instantaneous angular velocity from the gyro).
        self._state.yaw_rate = float(w_body[2])

        # Integrate yaw
        self._state.yaw += self._state.yaw_rate * dt
        # Wrap to [-pi, pi] to avoid unbounded growth
        if self._state.yaw > math.pi:
            self._state.yaw -= 2 * math.pi
        elif self._state.yaw < -math.pi:
            self._state.yaw += 2 * math.pi

        # Predict vx, vy from accel. Body-frame: ax pushes vx, ay
        # pushes vy. We do NOT include accel-z (gravity removal with
        # an unrolled chassis is a kinematic-bicycle assumption that
        # holds for FSD courses).
        ax_body = float(a_body[0])
        ay_body = float(a_body[1])

        # vx prediction: pure integration. RPM correction lands in
        # push_rpm() asynchronously.
        self._state.vx += ax_body * dt

        # vy prediction with kinematic-bicycle decay toward zero.
        # Real lateral accel during a yaw maneuver still shows up;
        # the leak just keeps integration noise from accumulating.
        self._state.vy = (
            (1.0 - self._beta_vy_leak) * self._state.vy
            + ay_body * dt
        )

        # Integrate position (rotate body velocity into world frame).
        c, s = math.cos(self._state.yaw), math.sin(self._state.yaw)
        self._state.x += (c * self._state.vx - s * self._state.vy) * dt
        self._state.y += (s * self._state.vx + c * self._state.vy) * dt

        # Phase 3 (#383): kinematic-bicycle yaw-rate cross-check.
        # Predicted yaw rate from current vx + steering angle:
        #     ω_pred = (v_x / wheelbase) · tan(δ)
        # Residual = ω_pred - ω_measured (IMU gyro_z). Persistent
        # non-zero residual = the car is sliding (real ω diverges
        # from kinematic prediction). Publishes via .diagnostics —
        # supervisor surfaces on /odom_diag/yaw_residual.
        if self._wheelbase_m > 1e-3:
            yaw_rate_pred = (
                self._state.vx / self._wheelbase_m
                * math.tan(self._latest_steering_rad)
            )
            self._diag.yaw_residual_rad_s = yaw_rate_pred - self._state.yaw_rate
            self._diag.slip_flag = (
                abs(self._diag.yaw_residual_rad_s)
                > self._slip_yaw_residual_threshold
            )

    def push_rpm(self, t: float, rpm: float) -> None:
        """Ingest one motor-RPM sample. Called from the RPM
        subscription callback (~80 Hz). t is wall-clock seconds; we
        use it for the staleness check inside push_imu's predict
        step (not implemented yet — currently we just store the
        latest correction target).
        """
        self._t_rpm_last = t
        self._latest_rpm_vx = float(rpm) * self._rpm_to_ms

        if not self._calib.completed:
            return

        # Phase 3 (#383): scale α_vx down when brake authority is high
        # (drive wheels potentially locked → RPM unreliable). The
        # filter falls back to IMU integration during the slip window;
        # vx re-converges to RPM after brake releases.
        if self._latest_brake > self._brake_lockup_threshold:
            alpha = self._alpha_vx_brake
        else:
            alpha = self._alpha_vx
        self._diag.effective_alpha_vx = alpha

        # Apply the complementary-filter correction immediately on
        # arrival rather than waiting for the next IMU step. RPM is
        # the slower input; deferring would add latency.
        residual = self._latest_rpm_vx - self._state.vx
        self._state.vx += alpha * residual

    def push_steering(self, t: float, angle_rad: float) -> None:
        """Ingest one steering-angle sample (#383). Called from the
        sim_supervisor /fsds/steering_angle subscription. Stored as
        the latest input to the kinematic-bicycle yaw cross-check in
        push_imu. No state mutation here — the prediction lives in
        push_imu so it stays time-aligned with the IMU gyro_z it's
        compared against."""
        del t  # currently only the value matters; staleness not gated
        self._latest_steering_rad = float(angle_rad)

    def push_brake(self, t: float, brake: float) -> None:
        """Ingest one brake-pressure sample (#383). Called from the
        sim_supervisor /fsds/brake_pressure subscription. Stored as
        the latest input to the α_vx scaling in push_rpm; clamped to
        [0, 1] defensively even though the bridge enforces the
        normalisation."""
        del t
        b = float(brake)
        if b < 0.0:
            b = 0.0
        elif b > 1.0:
            b = 1.0
        self._latest_brake = b

    # ------------------------------------------------------------------
    # Internal — calibration
    # ------------------------------------------------------------------
    def _accumulate_calibration(
        self, t: float, accel: np.ndarray, gyro: np.ndarray,
    ) -> None:
        if self._calib.t_first is None:
            self._calib.t_first = t

        self._calib.accel_sum += accel
        self._calib.gyro_sum += gyro
        self._calib.n_samples += 1

        if (t - self._calib.t_first) < self._calibration_seconds:
            return
        if self._calib.n_samples == 0:
            return

        # Estimate biases as the mean of the stationary readings.
        # Accel bias: subtract gravity (assumed body-z, since the car
        # is stationary on a level surface). The simulator places the
        # car upright at spawn so this assumption holds at t=0.
        accel_mean = self._calib.accel_sum / self._calib.n_samples
        gyro_mean = self._calib.gyro_sum / self._calib.n_samples

        self._calib.accel_bias = accel_mean - np.array([0.0, 0.0, G])
        self._calib.gyro_bias = gyro_mean
        self._calib.completed = True

        # Anchor pose at origin once calibration is done. Velocity is
        # zero (we just verified stationary).
        self._state = OdometryState()
        self._t_imu_last = t
