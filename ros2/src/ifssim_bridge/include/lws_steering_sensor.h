// LWS — Bosch Motorsport Steering Wheel Angle Sensor model.
//
// Mirrors the published sensor characteristics from
// docs/Data Sheet_191153675_Steering_Wheel_Angle_Sensor_LWS.pdf:
//
//   Range                ±780°  (±13.61 rad)
//   Resolution           0.1°
//   Nonlinearity         ±2.5°
//   Hysteresis           0 to 5°
//   Update rate          100 Hz (10 ms)
//   CAN ID 0x2B0         LWS_Standard
//
// Why the sim needs this:
//   The bridge has been publishing `/fsds/steering_angle` by echoing
//   the autonomy's commanded δ — a perfect, instant, noise-free
//   signal. The real DV pipeline reads Bosch LWS over CAN via uDV;
//   the sensed angle is quantized, has a bias, lags through
//   hysteresis, and arrives at a fixed 100 Hz cadence. Treating the
//   commanded value as the measurement would have the EKF (issue #462
//   follow-up) over-trust it during the upcoming steering-angle
//   measurement update — which is the entire reason we model the
//   sensor here.
//
// What this class is NOT:
//   * Not a full CAN frame producer. The byte-level fidelity of
//     LWS_Standard (0x2B0) — TRIM/CAL/OK bits, byte order, 16-bit LSB
//     packing — belongs at the uDV→DV-PC boundary when that's real
//     hardware. In sim we publish the post-decode floating-point
//     value directly.
//   * Not a model of the steering rack mechanics. The mapping from
//     steering wheel angle to road-wheel angle (steering ratio) is
//     applied at the call site, not here. This class only models the
//     sensor's behaviour on whatever steering-wheel angle it's asked
//     to measure.
//
// Header-only on purpose so the unit tests in
// ros2/src/ifssim_bridge/test/test_lws_steering_sensor.cpp can
// instantiate it without dragging in any ROS plumbing.

#ifndef IFSSIM_BRIDGE__LWS_STEERING_SENSOR_H_
#define IFSSIM_BRIDGE__LWS_STEERING_SENSOR_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace ifssim_bridge {

constexpr double kLwsResolutionDeg     = 0.1;    // datasheet "Absolute physical resolution"
constexpr double kLwsRangeDeg          = 780.0;  // datasheet "Measuring range"
constexpr double kLwsNonlinearityDeg   = 2.5;    // datasheet "Nonlinearity (±)"
constexpr double kLwsHysteresisDeg     = 5.0;    // datasheet "Hysteresis (max)"
constexpr double kLwsRateHz            = 100.0;  // datasheet "CAN update rate"

constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kRad2Deg = 180.0 / M_PI;

// Tunable parameters. The defaults match the datasheet's worst-case
// values; the bridge exposes them as ROS parameters so they can be
// dialled down for healthy-sensor scenarios or up for fault injection.
struct LwsParams {
  // Per-session systematic bias. Drawn uniformly in
  // [-nonlinearity_deg, +nonlinearity_deg] at construction.
  double nonlinearity_deg = kLwsNonlinearityDeg;

  // Direction-reversal hysteresis (deg). On a direction change the
  // measurement lags by this much, then decays exponentially toward
  // zero as motion continues in the new direction.
  double hysteresis_deg = kLwsHysteresisDeg;

  // Below-quantization noise floor. The datasheet doesn't specify
  // this explicitly but the sensor isn't bit-perfect; a small zero-
  // mean Gaussian below the 0.1° step keeps the EKF from over-
  // trusting consecutive identical samples.
  double noise_std_deg = 0.02;

  // Quantization step in degrees. 0 disables quantization (useful for
  // isolating bias/hysteresis behaviour in tests).
  double resolution_deg = kLwsResolutionDeg;

  // Optional fixed nonlinearity bias for reproducible tests. NaN means
  // "draw randomly from [-nonlinearity_deg, +nonlinearity_deg]".
  double fixed_nonlinearity_bias_deg = std::numeric_limits<double>::quiet_NaN();

  // Seed for the internal PRNG. 0 means "use std::random_device".
  uint32_t seed = 0;
};


class LwsSteeringSensor {
 public:
  // Constructs the sensor and freezes its per-session nonlinearity
  // bias. Pass `params.fixed_nonlinearity_bias_deg = X` for tests
  // that need to assert on a known bias; otherwise it's randomized.
  explicit LwsSteeringSensor(const LwsParams & params = LwsParams{})
  : params_(params)
  {
    if (params_.seed != 0) {
      rng_.seed(params_.seed);
    } else {
      std::random_device rd;
      rng_.seed(rd());
    }
    if (std::isfinite(params_.fixed_nonlinearity_bias_deg)) {
      bias_deg_ = params_.fixed_nonlinearity_bias_deg;
    } else {
      std::uniform_real_distribution<double> d(
        -params_.nonlinearity_deg, params_.nonlinearity_deg);
      bias_deg_ = d(rng_);
    }
  }

  // The session-frozen nonlinearity bias (deg). Useful for tests and
  // for surfacing on a diagnostic topic in production.
  double nonlinearity_bias_deg() const noexcept { return bias_deg_; }

  // Apply the sensor model to one commanded steering-wheel angle.
  //   true_angle_rad — the perfect underlying steering wheel angle
  //                    the rack would yield in the absence of the
  //                    sensor (sim: commanded δ × steering_ratio).
  // Returns the measured value the LWS would report on CAN.
  //
  // Lifecycle: stateful — call once per virtual sample at the LWS
  // sample rate. Hysteresis tracks the direction of the underlying
  // signal across calls.
  double measure(double true_angle_rad)
  {
    const double true_deg = true_angle_rad * kRad2Deg;

    // 1. Clamp to hardware range. The real LWS reports saturation
    //    (0x7FFF) past ±780°; in sim we silently clamp because the
    //    consumer-facing topic is a Float32, not a CAN byte stream.
    const double clamped_deg = std::clamp(true_deg, -kLwsRangeDeg, kLwsRangeDeg);

    // 2. Direction tracking. Hysteresis kicks in on the tick AFTER
    //    the underlying signal reverses. Tiny oscillations below the
    //    resolution threshold don't count — otherwise sensor noise
    //    in tests would re-trigger hysteresis every sample.
    const double delta = clamped_deg - last_true_deg_;
    const double dir = (std::abs(delta) > params_.resolution_deg)
      ? std::copysign(1.0, delta) : last_dir_;
    if (dir != 0.0 && last_dir_ != 0.0 && dir != last_dir_) {
      // Direction reversed — load the hysteresis lag in the opposite
      // sign of the new motion. We will return to zero as motion
      // continues in `dir`.
      hyst_deg_ = -dir * params_.hysteresis_deg;
    } else {
      // Same direction (or first sample). Decay the lag toward zero.
      // Decay rate chosen so that ~1° of motion roughly halves the
      // lag — slow enough that brief blips don't reset it instantly,
      // fast enough that a sustained sweep clears it within ~5° of
      // travel (typical FS one-turn lock-to-lock sweeps ±90° at the
      // wheel many times per lap).
      constexpr double kHystHalfLifeDeg = 1.0;
      const double decay = std::pow(0.5, std::abs(delta) / kHystHalfLifeDeg);
      hyst_deg_ *= decay;
    }
    last_true_deg_ = clamped_deg;
    if (dir != 0.0) last_dir_ = dir;

    // 3. Apply systematic bias + hysteresis + zero-mean noise.
    const double noise_deg = (params_.noise_std_deg > 0.0)
      ? std::normal_distribution<double>(0.0, params_.noise_std_deg)(rng_)
      : 0.0;
    double measured_deg = clamped_deg + bias_deg_ + hyst_deg_ + noise_deg;

    // 4. Quantize. Round-to-nearest matches the LWS micro's A/D
    //    converter behaviour better than floor.
    if (params_.resolution_deg > 0.0) {
      measured_deg = std::round(measured_deg / params_.resolution_deg)
                   * params_.resolution_deg;
    }
    return measured_deg * kDeg2Rad;
  }

  // Reset internal state — direction tracking, hysteresis lag, etc.
  // The frozen nonlinearity bias is preserved (it's a sensor property,
  // not a runtime state). Useful when reusing one sensor across test
  // cases.
  void reset_runtime_state() noexcept {
    last_true_deg_ = 0.0;
    last_dir_ = 0.0;
    hyst_deg_ = 0.0;
  }

 private:
  LwsParams params_;
  std::mt19937 rng_;

  // Per-session frozen systematic bias (deg).
  double bias_deg_ = 0.0;

  // Direction-tracking state for hysteresis.
  double last_true_deg_ = 0.0;
  double last_dir_ = 0.0;       // 0 until first non-trivial step
  double hyst_deg_ = 0.0;        // current lag offset
};

}  // namespace ifssim_bridge

#endif  // IFSSIM_BRIDGE__LWS_STEERING_SENSOR_H_
