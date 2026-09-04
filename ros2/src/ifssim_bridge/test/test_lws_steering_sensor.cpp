// Unit tests for the Bosch LWS sensor model in
// include/lws_steering_sensor.h. Pins datasheet-aligned behaviour:
// quantization step, range clamping, per-session frozen
// nonlinearity bias, direction-reversal hysteresis, and PRNG seeding
// determinism.
//
// These tests are intentionally header-only consumers — they don't
// touch ROS, so they run as plain gtest binaries with no ament
// fixtures.

#include <gtest/gtest.h>

#include <cmath>

#include "lws_steering_sensor.h"

using ifssim_bridge::LwsParams;
using ifssim_bridge::LwsSteeringSensor;
using ifssim_bridge::kDeg2Rad;
using ifssim_bridge::kRad2Deg;
using ifssim_bridge::kLwsRangeDeg;
using ifssim_bridge::kLwsResolutionDeg;

namespace {

// Build a sensor with all noise sources off — useful when we want to
// isolate one effect at a time.
LwsParams quiet_params(double seed = 42) {
  LwsParams p;
  p.fixed_nonlinearity_bias_deg = 0.0;
  p.noise_std_deg = 0.0;
  p.hysteresis_deg = 0.0;
  p.seed = static_cast<uint32_t>(seed);
  return p;
}

}  // namespace


// ----- Quantization -----

TEST(LwsSteeringSensor, QuantizesToResolutionStep) {
  // Inputs at fractions of 0.1° should snap to the nearest step.
  LwsSteeringSensor s(quiet_params());
  const double in_deg = 12.34;
  const double out_rad = s.measure(in_deg * kDeg2Rad);
  const double out_deg = out_rad * kRad2Deg;
  // 12.34 rounds to 12.3 (nearest 0.1° step).
  EXPECT_NEAR(out_deg, 12.3, 1e-6);
}


TEST(LwsSteeringSensor, ZeroResolutionDisablesQuantization) {
  LwsParams p = quiet_params();
  p.resolution_deg = 0.0;
  LwsSteeringSensor s(p);
  // 12.34° should come back exactly (no quantization, no noise, no bias).
  const double out_deg = s.measure(12.34 * kDeg2Rad) * kRad2Deg;
  EXPECT_NEAR(out_deg, 12.34, 1e-9);
}


// ----- Range clamping -----

TEST(LwsSteeringSensor, ClampsAtHardwareRange) {
  LwsSteeringSensor s(quiet_params());
  // ±900° should clamp to ±780° (datasheet hardware range).
  const double pos = s.measure(900.0 * kDeg2Rad) * kRad2Deg;
  const double neg = s.measure(-900.0 * kDeg2Rad) * kRad2Deg;
  EXPECT_NEAR(pos,  kLwsRangeDeg, kLwsResolutionDeg);
  EXPECT_NEAR(neg, -kLwsRangeDeg, kLwsResolutionDeg);
}


// ----- Nonlinearity bias -----

TEST(LwsSteeringSensor, FrozenBiasShiftsAllSamplesByTheSameAmount) {
  LwsParams p = quiet_params();
  p.fixed_nonlinearity_bias_deg = 1.5;
  LwsSteeringSensor s(p);
  EXPECT_NEAR(s.nonlinearity_bias_deg(), 1.5, 1e-9);

  const double a_deg = s.measure(0.0)  * kRad2Deg;
  const double b_deg = s.measure(10.0 * kDeg2Rad) * kRad2Deg;
  const double c_deg = s.measure(-5.0 * kDeg2Rad) * kRad2Deg;
  EXPECT_NEAR(a_deg,   1.5, kLwsResolutionDeg + 1e-6);
  EXPECT_NEAR(b_deg,  11.5, kLwsResolutionDeg + 1e-6);
  EXPECT_NEAR(c_deg,  -3.5, kLwsResolutionDeg + 1e-6);
}


TEST(LwsSteeringSensor, RandomBiasStaysWithinDatasheetBound) {
  LwsParams p;
  p.nonlinearity_deg = 2.5;
  p.seed = 12345;  // deterministic
  LwsSteeringSensor s(p);
  EXPECT_GE(s.nonlinearity_bias_deg(), -2.5);
  EXPECT_LE(s.nonlinearity_bias_deg(),  2.5);
}


TEST(LwsSteeringSensor, SameSeedYieldsSameBias) {
  LwsParams p;
  p.seed = 7;
  LwsSteeringSensor a(p), b(p);
  EXPECT_DOUBLE_EQ(a.nonlinearity_bias_deg(), b.nonlinearity_bias_deg());
}


// ----- Hysteresis -----

TEST(LwsSteeringSensor, HysteresisLagsOnDirectionReversal) {
  LwsParams p = quiet_params();
  p.hysteresis_deg = 5.0;
  LwsSteeringSensor s(p);

  // Sweep right (positive) — last sample sets dir = +1.
  for (double deg = 0.0; deg <= 20.0; deg += 1.0) {
    s.measure(deg * kDeg2Rad);
  }
  // Reverse: now sweep back left. First sample after reversal
  // should be biased by +5° (lag against the new -x direction).
  const double m_deg = s.measure(19.0 * kDeg2Rad) * kRad2Deg;
  // Expected ≈ 19.0 + 5.0 = 24.0° before quantization.
  EXPECT_GT(m_deg, 23.0);
  EXPECT_LT(m_deg, 25.0);
}


TEST(LwsSteeringSensor, HysteresisDecaysAsMotionContinues) {
  LwsParams p = quiet_params();
  p.hysteresis_deg = 5.0;
  LwsSteeringSensor s(p);
  // Set up: sweep right then reverse.
  for (double deg = 0.0; deg <= 20.0; deg += 1.0) s.measure(deg * kDeg2Rad);
  s.measure(19.0 * kDeg2Rad);  // direction reverse — lag loaded at +5°
  // Continue sweeping left — lag should decay back toward zero.
  // After ~5° of leftward travel, residual lag should be < 0.5°.
  for (double deg = 19.0; deg >= 14.0; deg -= 1.0) s.measure(deg * kDeg2Rad);
  const double m_deg = s.measure(13.0 * kDeg2Rad) * kRad2Deg;
  EXPECT_NEAR(m_deg, 13.0, 0.5);
}


TEST(LwsSteeringSensor, NoHysteresisOnMonotonicSweep) {
  LwsParams p = quiet_params();
  p.hysteresis_deg = 5.0;
  LwsSteeringSensor s(p);
  // Pure increasing sweep — no direction change, no lag.
  double prev = -1.0;
  for (double deg = 0.0; deg <= 50.0; deg += 1.0) {
    const double m = s.measure(deg * kDeg2Rad) * kRad2Deg;
    EXPECT_NEAR(m, deg, kLwsResolutionDeg + 1e-6);
    EXPECT_GE(m, prev);
    prev = m;
  }
}


// ----- Determinism + reset -----

TEST(LwsSteeringSensor, RuntimeResetClearsHysteresisButKeepsBias) {
  LwsParams p = quiet_params();
  p.fixed_nonlinearity_bias_deg = 1.0;
  p.hysteresis_deg = 5.0;
  LwsSteeringSensor s(p);
  // Load hysteresis.
  for (double deg = 0.0; deg <= 10.0; deg += 1.0) s.measure(deg * kDeg2Rad);
  s.measure(9.0 * kDeg2Rad);  // reverse — lag loaded
  s.reset_runtime_state();
  // After reset, a fresh measurement at 0° should equal the bias alone.
  const double m_deg = s.measure(0.0) * kRad2Deg;
  EXPECT_NEAR(m_deg, 1.0, kLwsResolutionDeg + 1e-6);
}


// ----- 100 Hz cadence sanity (no model state assertion — just that
// repeated identical inputs yield bias-aligned consistent outputs) ----

TEST(LwsSteeringSensor, ConsistentOutputOnStationaryInput) {
  LwsParams p = quiet_params();
  p.fixed_nonlinearity_bias_deg = 0.7;
  LwsSteeringSensor s(p);
  const double in_rad = 5.0 * kDeg2Rad;
  const double a = s.measure(in_rad);
  const double b = s.measure(in_rad);
  const double c = s.measure(in_rad);
  EXPECT_DOUBLE_EQ(a, b);
  EXPECT_DOUBLE_EQ(b, c);
}
