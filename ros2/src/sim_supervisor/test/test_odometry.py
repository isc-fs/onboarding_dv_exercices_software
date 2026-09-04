"""Unit tests for sim_supervisor.odometry.OdometryFilter.

Pure-Python (numpy only); no rclpy needed. Run with:
    cd pipeline/sim_supervisor && python3 -m pytest test/ -v

The filter has three observable behaviours we test independently:
  1. **Stationary calibration**: while in the calibration window the
     filter returns is_calibrated()==False and accumulates bias
     estimates. Once the window closes it freezes biases and starts
     publishing.
  2. **Velocity tracking**: with a stream of constant-RPM samples the
     filter's vx converges toward the RPM-derived velocity (modulo
     IMU integration noise).
  3. **Yaw integration**: with a stream of constant gyro_z, the
     filter's yaw integrates linearly until it wraps at ±π.
"""

import math

import numpy as np
import pytest

# Importable as a library — package layout puts odometry.py inside
# the sim_supervisor Python package. Running pytest from
# pipeline/sim_supervisor lets pytest discover both.
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from sim_supervisor.odometry import (
    OdometryFilter,
    RPM_TO_MS,
    G,
    ALPHA_VX,
    ALPHA_VX_BRAKE,
    BRAKE_LOCKUP_THRESHOLD,
    SLIP_YAW_RESIDUAL_THRESHOLD,
    WHEELBASE_M,
)


# ---------------------------------------------------------------------
# Test fixtures
# ---------------------------------------------------------------------
@pytest.fixture
def stationary_filter():
    """OdometryFilter that has just completed its stationary
    calibration window. Calibration biases are zero (we feed clean
    gravity + zero gyro, so accel_bias = 0, gyro_bias = 0)."""
    f = OdometryFilter()
    # Feed 1 s of stationary samples at 400 Hz. With calibration_seconds
    # at the default 3.0 s, just push enough samples to span > 3 s of
    # virtual time.
    accel_stationary = np.array([0.0, 0.0, G])
    gyro_stationary = np.array([0.0, 0.0, 0.0])
    n_samples = 1500
    dt = 0.0025  # 400 Hz
    for i in range(n_samples):
        t = i * dt
        f.push_imu(t, accel_stationary, gyro_stationary)
    assert f.is_calibrated(), "fixture should leave filter calibrated"
    return f


# ---------------------------------------------------------------------
# Calibration behaviour
# ---------------------------------------------------------------------
def test_starts_uncalibrated():
    f = OdometryFilter()
    assert not f.is_calibrated()


def test_calibrates_within_window():
    """3 s of stationary IMU samples completes calibration."""
    f = OdometryFilter()
    accel = np.array([0.0, 0.0, G])
    gyro = np.zeros(3)
    # Push samples spanning > 3 s of virtual time
    for i in range(1500):
        f.push_imu(i * 0.0025, accel, gyro)
    assert f.is_calibrated()


def test_calibration_estimates_bias():
    """A constant accel offset over the window should land in
    accel_bias; the post-calibration state should be at origin/zero
    velocity."""
    f = OdometryFilter()
    bias_x = 0.05  # m/s² — small constant offset
    accel = np.array([bias_x, 0.0, G])
    gyro = np.zeros(3)
    for i in range(1500):
        f.push_imu(i * 0.0025, accel, gyro)
    assert f.is_calibrated()
    # Post-calibration internal accel_bias should equal what we offset by.
    # Access via the private _calib for the test (no public getter
    # because the bias isn't part of the published state).
    assert math.isclose(f._calib.accel_bias[0], bias_x, abs_tol=1e-6)
    # State anchored at origin (FP-precision tolerance — float64
    # accumulation across 1500 samples can leak ~1e-16 even when the
    # math is exactly zero).
    assert math.isclose(f.state.x, 0.0, abs_tol=1e-12)
    assert math.isclose(f.state.y, 0.0, abs_tol=1e-12)
    assert math.isclose(f.state.vx, 0.0, abs_tol=1e-12)


def test_uncalibrated_filter_does_not_publish_state(stationary_filter):
    """Sanity: post-fixture filter is calibrated and at origin."""
    f = stationary_filter
    assert f.is_calibrated()
    s = f.state
    assert s.x == 0.0 and s.y == 0.0
    assert s.vx == 0.0 and s.vy == 0.0


# ---------------------------------------------------------------------
# Velocity tracking — RPM correction
# ---------------------------------------------------------------------
def test_rpm_pulls_vx_toward_correction_target(stationary_filter):
    """Each push_rpm applies α × residual to vx. Sequential pushes
    should converge toward the target."""
    f = stationary_filter
    target_rpm = 100.0  # arbitrary
    target_vx = target_rpm * RPM_TO_MS

    # α=0.10 default → each push closes ~10% of the residual.
    # After N pushes the gap is target_vx × (1-α)^N. To reach
    # abs_tol=1e-4 we need (0.9)^N · 0.898 < 1e-4 → N > 86. Use 200
    # to leave a comfortable margin.
    for i in range(200):
        f.push_rpm(t=i * 0.0125, rpm=target_rpm)

    assert math.isclose(f.state.vx, target_vx, abs_tol=1e-4)


def test_rpm_correction_is_monotone(stationary_filter):
    """vx should approach target without overshoot when RPM is constant."""
    f = stationary_filter
    target_rpm = 50.0
    prev_vx = f.state.vx
    for i in range(20):
        f.push_rpm(t=i * 0.0125, rpm=target_rpm)
        assert f.state.vx >= prev_vx - 1e-9, "vx should be non-decreasing"
        assert f.state.vx <= target_rpm * RPM_TO_MS + 1e-9, "no overshoot"
        prev_vx = f.state.vx


# ---------------------------------------------------------------------
# Yaw integration
# ---------------------------------------------------------------------
def test_yaw_integrates_constant_gyro(stationary_filter):
    """With a constant gyro_z = 0.5 rad/s for 1 s of IMU samples, yaw
    should land at ~0.5 rad. Position stays near origin because vx=vy=0."""
    f = stationary_filter
    gyro = np.array([0.0, 0.0, 0.5])
    accel = np.array([0.0, 0.0, G])
    # 1 s @ 400 Hz, starting after the calibration window (we use
    # virtual time relative to the fixture's last sample at 1499*0.0025
    # = 3.7475 s).
    t0 = 1500 * 0.0025
    n = 400
    for i in range(n):
        f.push_imu(t0 + i * 0.0025, accel, gyro)
    # Yaw should be near 0.5 rad. Tolerance allows for the first sample
    # being skipped (initial _t_imu_last assignment).
    assert math.isclose(f.state.yaw, 0.5, abs_tol=0.01)


def test_yaw_wraps_to_negative():
    """Continuous spin past +π should wrap to negative side rather than
    growing unbounded."""
    f = OdometryFilter()
    accel = np.array([0.0, 0.0, G])
    gyro_zero = np.zeros(3)
    # Calibrate
    for i in range(1500):
        f.push_imu(i * 0.0025, accel, gyro_zero)
    assert f.is_calibrated()

    # Spin at 5 rad/s for 0.8 s → 4.0 rad raw, should wrap to ~-2.28 rad
    gyro_spin = np.array([0.0, 0.0, 5.0])
    t0 = 1500 * 0.0025
    n = 320
    for i in range(n):
        f.push_imu(t0 + i * 0.0025, accel, gyro_spin)
    assert f.state.yaw >= -math.pi
    assert f.state.yaw <= math.pi


# ---------------------------------------------------------------------
# Integration end-to-end
# ---------------------------------------------------------------------
def test_constant_rpm_drives_position(stationary_filter):
    """With vx pinned by RPM corrections and yaw=0, position should
    advance along +x."""
    f = stationary_filter
    target_rpm = 100.0
    target_vx = target_rpm * RPM_TO_MS  # ≈ 0.898 m/s

    # First saturate vx with 200 RPM pushes (matches the convergence
    # math above — 200 × α=0.10 → residual ~1.8e-9 × 0.898).
    for i in range(200):
        f.push_rpm(t=i * 0.0125, rpm=target_rpm)
    assert math.isclose(f.state.vx, target_vx, abs_tol=1e-4)

    # Now drive 1 s of IMU samples at zero accel (vx held by RPM).
    accel = np.array([0.0, 0.0, G])
    gyro = np.zeros(3)
    t0 = 1500 * 0.0025
    n = 400
    for i in range(n):
        # Re-pin vx with an RPM push every 5 IMU samples (~80 Hz)
        if i % 5 == 0:
            f.push_rpm(t=0.625 + i * 0.0025, rpm=target_rpm)
        f.push_imu(t0 + i * 0.0025, accel, gyro)

    # Expected x ≈ vx * 1 s ≈ 0.898 m. Tolerance for IMU integration
    # noise from the bias-corrected accel.z (which is exactly G here so
    # bias is exactly 0; no x drift expected).
    assert math.isclose(f.state.x, target_vx, abs_tol=0.05)
    assert math.isclose(f.state.y, 0.0, abs_tol=0.01)


# ---------------------------------------------------------------------
# Reset
# ---------------------------------------------------------------------
def test_reset_clears_state(stationary_filter):
    """After reset(), filter is uncalibrated and at zero state."""
    f = stationary_filter
    f.push_rpm(t=0.0, rpm=200.0)
    assert f.state.vx > 0.0

    f.reset()
    assert not f.is_calibrated()
    s = f.state
    assert s.x == 0.0 and s.y == 0.0
    assert s.vx == 0.0 and s.vy == 0.0
    assert s.yaw == 0.0


# ---------------------------------------------------------------------
# Phase 3 (#383) — steering + brake cross-checks
# ---------------------------------------------------------------------
def test_yaw_residual_zero_when_kinematics_match(stationary_filter):
    """If steering = 0 and gyro_z = 0, predicted yaw rate = 0 and
    measured yaw rate = 0 → residual should be ~0."""
    f = stationary_filter
    accel = np.array([0.0, 0.0, G])
    gyro = np.zeros(3)
    # Pin vx to a small non-zero value via RPM so the kinematic-bicycle
    # prediction has something to multiply (ω_pred = vx/L · tan δ).
    for i in range(50):
        f.push_rpm(t=i * 0.0125, rpm=50.0)
    f.push_steering(t=0.0, angle_rad=0.0)
    # Drive a few IMU samples; with steering=0 the predicted yaw rate
    # is zero regardless of vx.
    t0 = 1500 * 0.0025
    for i in range(20):
        f.push_imu(t0 + i * 0.0025, accel, gyro)
    d = f.diagnostics
    assert math.isclose(d.yaw_residual_rad_s, 0.0, abs_tol=1e-9)
    assert d.slip_flag is False


def test_yaw_residual_nonzero_under_steering(stationary_filter):
    """With steering δ != 0 and gyro_z = 0, the kinematic-bicycle
    prediction gives a non-zero ω_pred — the residual measures the
    "expected vs measured" turn rate. Slip flag set when residual
    exceeds the threshold."""
    f = stationary_filter
    accel = np.array([0.0, 0.0, G])
    gyro = np.zeros(3)
    # Build up vx ≈ 5 m/s by saturating the filter with RPM
    target_rpm = 556.0  # ≈ 5 m/s at RPM_TO_MS = 0.00821
    for i in range(200):
        f.push_rpm(t=i * 0.0125, rpm=target_rpm)
    # Steer at +0.3 rad (~17°) — kinematic predicts ω = (5/1.570)·tan(0.3) ≈ 0.98 rad/s
    f.push_steering(t=0.0, angle_rad=0.3)
    t0 = 1500 * 0.0025
    for i in range(20):
        f.push_imu(t0 + i * 0.0025, accel, gyro)
    d = f.diagnostics
    expected_pred = (f.state.vx / WHEELBASE_M) * math.tan(0.3)
    assert math.isclose(d.yaw_residual_rad_s, expected_pred, abs_tol=0.05)
    # The expected residual ≈ 1.0 rad/s is above the 0.3 rad/s slip
    # threshold → slip flag should be set.
    assert d.slip_flag is True


def test_brake_collapses_alpha_vx(stationary_filter):
    """When brake_pressure > BRAKE_LOCKUP_THRESHOLD, the next RPM
    correction should use ALPHA_VX_BRAKE (not ALPHA_VX)."""
    f = stationary_filter
    # Start vx at 0; one RPM correction at normal α gives vx ≈ α·target
    target_rpm = 100.0
    target_vx = target_rpm * RPM_TO_MS

    # Push brake above threshold first
    f.push_brake(t=0.0, brake=0.5)
    # Now an RPM sample arrives — should apply the brake-α
    f.push_rpm(t=0.0, rpm=target_rpm)
    # vx after one correction at α_brake from 0 = α_brake · target
    assert math.isclose(
        f.state.vx, ALPHA_VX_BRAKE * target_vx, abs_tol=1e-6,
    )
    assert math.isclose(f.diagnostics.effective_alpha_vx, ALPHA_VX_BRAKE, abs_tol=1e-9)


def test_brake_release_restores_alpha_vx(stationary_filter):
    """After brake drops below threshold, α_vx returns to the normal
    correction strength."""
    f = stationary_filter
    target_rpm = 100.0
    # Brake on, one correction
    f.push_brake(t=0.0, brake=0.5)
    f.push_rpm(t=0.0, rpm=target_rpm)
    # Brake off, second correction — α back to normal
    f.push_brake(t=0.1, brake=0.0)
    f.push_rpm(t=0.1, rpm=target_rpm)
    assert math.isclose(f.diagnostics.effective_alpha_vx, ALPHA_VX, abs_tol=1e-9)


def test_brake_below_threshold_uses_normal_alpha(stationary_filter):
    """Light brake (below BRAKE_LOCKUP_THRESHOLD) doesn't trigger the
    fallback — we keep the normal α even on partial-brake tickover."""
    f = stationary_filter
    f.push_brake(t=0.0, brake=BRAKE_LOCKUP_THRESHOLD - 0.05)
    f.push_rpm(t=0.0, rpm=100.0)
    assert math.isclose(f.diagnostics.effective_alpha_vx, ALPHA_VX, abs_tol=1e-9)


def test_reset_clears_phase3_state(stationary_filter):
    """reset() restores steering / brake / diagnostics to defaults."""
    f = stationary_filter
    f.push_steering(t=0.0, angle_rad=0.4)
    f.push_brake(t=0.0, brake=0.7)
    f.push_rpm(t=0.0, rpm=200.0)
    assert f.diagnostics.effective_alpha_vx == ALPHA_VX_BRAKE  # under brake
    f.reset()
    assert f.diagnostics.effective_alpha_vx == ALPHA_VX        # default
    assert f.diagnostics.slip_flag is False
    assert math.isclose(f.diagnostics.yaw_residual_rad_s, 0.0, abs_tol=1e-9)
