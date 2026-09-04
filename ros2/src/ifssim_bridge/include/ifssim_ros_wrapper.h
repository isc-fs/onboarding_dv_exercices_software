#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <fs_msgs/msg/control_command.hpp>
#include <fs_msgs/msg/go_signal.hpp>
#include <fs_msgs/msg/finished_signal.hpp>
#include <fs_msgs/msg/track.hpp>
#include <fs_msgs/msg/extra_info.hpp>
#include <fs_msgs/srv/reset.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "lws_steering_sensor.h"
#include "tcp_client.h"
#include "udp_receiver.h"  // For frame struct definitions

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <map>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

/**
 * IFSSIM ROS2 Wrapper — sensor over TCP, LiDAR over UDP.
 *
 * Architecture (2 TCP connections + one UDP listener):
 *   1. Sensor stream (`streamSensors`, TCP push, port 41452)
 *      → IMU at 400Hz, GSS/TF/Odom at 100Hz, GPS at 10Hz
 *   2. LiDAR stream (UDP push from FSDSUdpBroadcaster, port 51453)
 *      → PointCloud2 at ~10Hz. UdpReceiver handles chunk reassembly.
 *      LiDAR is UDP-only since #322 (#321's TCP/UDS soft-deprecation).
 *   3. Command client (TCP req/resp, port 41451 RPC)
 *      → control commands, settings, referee queries
 *
 * Camera sensors were removed in perf/strip-cameras — the legacy camera
 * client + cameraTimerCb + CompressedImage publishers are gone.
 */
class IFSSIMRosWrapper
{
public:
    IFSSIMRosWrapper(
        std::shared_ptr<rclcpp::Node> node,
        const std::string& host,
        int port,
        double timeout_sec);

    ~IFSSIMRosWrapper();

private:
    void initializeConnection();
    void initializePublishers();
    void initializeSubscribers();
    void initializeTimers();
    void startStreaming();
    void triggerReconnect();  // Called by stream threads on disconnect

    // Streaming threads. The TCP `lidarStreamThread` was removed in
    // #322 (LiDAR moved to chunked UDP via UdpReceiver) and
    // reinstated in PR-#482: the UDP path's wedge modes (userspace
    // UDP proxy losing port bindings, WSL2 kernel rcvbuf overflow
    // for fragmented datagrams) are structurally unfixable cross-
    // platform on Docker Desktop, and TCP avoids both problems by
    // construction. UdpReceiver is still in the tree as an
    // env-var-gated fallback (IFSSIM_LIDAR_TRANSPORT=udp) — see
    // startStreaming() for the switch.
    void sensorStreamThread();
    void sensorPublishThread();  // Drains the single-slot buffer, calls onSensorFrame
    void lidarStreamThread();    // PR-#482: TCP recv loop, reads stream-header + payload
    void lidarPublishThread();   // Drains the single-slot buffer, calls onLidarFrame

    // Stream data handlers
    void onSensorFrame(const SensorFrame& frame);
    void onLidarFrame(const LidarChunkHeader& header, const float* points);

    // Timer callbacks (TCP command client, low frequency)
    void goSignalTimerCb();
    void extraInfoTimerCb();
    void trackPublishCb();
    void staticTfCb();
    void tireLoadsTimerCb();

    // Subscriber callbacks
    void controlCommandCb(const fs_msgs::msg::ControlCommand::SharedPtr msg);
    void ebsRequestCb(const std_msgs::msg::Empty::SharedPtr msg);
    void ebsResetCb(const std_msgs::msg::Empty::SharedPtr msg);
    void resetSrvCb(
        const std::shared_ptr<fs_msgs::srv::Reset::Request> request,
        std::shared_ptr<fs_msgs::srv::Reset::Response> response);

    // Node
    std::shared_ptr<rclcpp::Node> node_;

    // TCP clients
    std::unique_ptr<TcpClient> client_;          // Commands + referee queries

    // Streaming sockets (raw, not TcpClient — held open).
    // `lidar_stream_fd_` + `lidar_thread_` are populated only when
    // IFSSIM_LIDAR_TRANSPORT=tcp (default, see startStreaming). The
    // fallback path (=udp) leaves both at default and starts
    // udp_receiver_ instead. Sensors always ride TCP — they don't
    // have the fragmentation pathology LiDAR did.
    std::atomic<int> sensor_stream_fd_{-1};
    std::atomic<int> lidar_stream_fd_{-1};
    std::thread sensor_thread_;
    std::thread lidar_thread_;
    std::atomic<bool> streaming_{false};
    std::mutex reconnect_mutex_;  // Ensures only one thread reconnects at a time

    // Which LiDAR transport this bridge instance is using. Set once
    // at startStreaming() from IFSSIM_LIDAR_TRANSPORT and never
    // changed. Plumbed to triggerReconnect() so it knows whether to
    // reopen the TCP LiDAR socket or leave the UDP receiver running.
    enum class LidarTransport { Tcp, Udp };
    LidarTransport lidar_transport_ = LidarTransport::Tcp;

    // Sensor producer-consumer split — same pattern as the LiDAR split
    // below. Before this, the 400 Hz sensor recv thread published IMU
    // (400 Hz), GSS/TF/Odom (100 Hz), GPS (10 Hz) inline, so any DDS
    // backpressure (slow subscriber, full RELIABLE buffer) blocked the
    // recv loop and the kernel TCP buffer filled within ~milliseconds.
    // Plugin's SendAll then hit its 1 s timeout, tore the stream down,
    // and ALL sensor topics fell to ~0 Hz under pipeline load. Symptom
    // captured: with pipeline running, /imu and /gps each reported
    // "topic does not appear to be published" for 10 s. Move publishes
    // to a dedicated thread; recv thread now only pushes the latest
    // SensorFrame into a single-slot buffer and immediately re-enters
    // recv. Frame drops are acceptable: at 400 Hz a missed sample is
    // 2.5 ms of IMU.
    std::mutex sensor_pub_mutex_;
    std::condition_variable sensor_pub_cv_;
    std::optional<SensorFrame> sensor_pending_;
    std::thread sensor_pub_thread_;

    // LiDAR producer-consumer split. The recv thread used to call
    // publish() inline, which under sustained pipeline-subscriber load
    // would block long enough for the kernel TCP recv buffer to fill ⇒
    // plugin's send buffer fills ⇒ plugin's SendAll hits its 1 s timeout
    // ⇒ stream tear-down ⇒ reconnect cascade. By moving publish() to a
    // dedicated thread the recv thread never blocks on ROS work — it
    // pushes the latest frame into a single-slot buffer (dropping any
    // unconsumed older frame) and immediately re-enters recv. Drops are
    // intentional: LiDAR is a streaming firehose, latency matters more
    // than every-frame delivery.
    struct PendingLidarFrame {
        LidarChunkHeader header;
        std::vector<float> points;
    };
    std::mutex lidar_pub_mutex_;
    std::condition_variable lidar_pub_cv_;
    std::optional<PendingLidarFrame> lidar_pending_;
    std::thread lidar_pub_thread_;

    // Connection params
    std::string host_;
    int port_;
    double timeout_sec_;

    // Frame IDs — match original FSDS simulator convention
    std::string map_frame_id_ = "odom";
    std::string vehicle_frame_id_ = "fsds/FSCar";

    // Publishers
    // /clock — sim time source for the whole pipeline (use_sim_time). Driven
    // from the SensorFrame's sim timestamp on every sensor frame (the highest-
    // rate, always-on stream). The bridge itself does NOT run use_sim_time —
    // it's the clock *source*, so it builds rclcpp::Time straight from the wire
    // sim ns and must avoid the chicken-and-egg of waiting on its own /clock.
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    // Last sim time published on /clock — /clock must be non-decreasing
    // WITHIN a sim session. A step backwards larger than
    // kSimTimeRewindThresholdNs is not jitter but a sim-time rewind: IFSSIM
    // was restarted or the level reloaded while the bridge stayed up (UE game
    // time restarts at 0 on level load). onSensorFrame() then resets this and
    // the IMU clamp and lets /clock jump back; onLidarFrame() does the same
    // for its own clamp. Without that, /clock would stay silent until the new
    // session's sim time overtook the old one, and every use_sim_time node's
    // timers (AS tick, reconciler, controller) would sit frozen for as long as
    // the previous session lasted — "session started" but the car never moves.
    rclcpp::Time last_clock_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    static constexpr int64_t kSimTimeRewindThresholdNs = 500000000;  // 0.5 s
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gss_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_rpm_pub_;
    // /fsds/steering_angle — actual front-wheel angle in radians.
    // Post-#462: this is the Bosch-LWS-modelled steering wheel angle
    // (with quantization, nonlinearity bias, hysteresis, 100 Hz
    // cadence) divided by `steering_ratio_` to recover the road-wheel
    // angle that consumers expect. The "perfect commanded δ × max_rad"
    // path is gone — the autonomy now sees a sensor-faithful signal.
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_angle_pub_;
    // /fsds/lws/steering_wheel_angle_rad — sensor-faithful steering
    // wheel angle (NOT the road-wheel angle). Mirrors what uDV would
    // forward off the Bosch LWS CAN frame; the road-wheel-angle topic
    // above is the post-conversion view of the same measurement.
    // Both topics publish from the same 100 Hz timer with the same
    // underlying LwsSteeringSensor::measure() call to guarantee
    // they're consistent.
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lws_steering_wheel_pub_;
    // /fsds/brake_pressure — commanded brake authority [0, 1],
    // published from SensorFrame.brake (controls echo from UE5).
    // Phase 3 (#383) input to OdometryFilter for slip detection
    // (high brake → distrust RPM-derived vx).
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_pressure_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tire_loads_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    // Optional subsampled LiDAR cloud for visualisation tools. Browser-
    // based viewers (Foxglove web, Lichtblick web) deserialise + WebGL-
    // upload the full 1.5 MB/scan stream on the JS thread, which lands at
    // 30-40 % CPU on a tab. This publisher emits every Nth point to a
    // companion topic so a viz session can subscribe to /lidar/Lidar1/viz
    // and leave /lidar/Lidar1 (full density) for the autonomy stack. Only
    // created when `lidar_viz_decimation` parameter > 1 (default 0 = off,
    // production runs unaffected).
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_viz_pub_;
    // 0 (default) = disabled, no viz publisher created. >= 2 = publish
    // every Nth point on /lidar/Lidar1/viz alongside the full cloud.
    uint32_t lidar_viz_decimation_ = 0;
    rclcpp::Publisher<fs_msgs::msg::GoSignal>::SharedPtr go_signal_pub_;
    rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr finished_signal_pub_;
    rclcpp::Publisher<fs_msgs::msg::ExtraInfo>::SharedPtr extra_info_pub_;
    rclcpp::Publisher<fs_msgs::msg::Track>::SharedPtr track_pub_;

    // Transient: previous referee.finished value, for edge-triggered publish
    bool last_finished_state_ = false;

    // Transient: previous referee.laps value, used together with
    // last_finished_state_ to detect a session restart and auto-clear
    // ebs_triggered_ — the only reliable in-bridge signal that the next
    // setCarControls belongs to a new run. See extraInfoTimerCb() for
    // the edge-detection logic.
    uint32_t last_laps_state_ = 0;

    // Monotonic-stamp guards for IMU and LiDAR. GLIM (and any LiDAR-IMU
    // SLAM pipeline) rejects samples whose timestamp ≤ the last accepted
    // sample's timestamp. The bridge's `node_->now()` snapshot in the
    // sensor and lidar publish threads can race occasionally — at 400 Hz
    // IMU we observed ~5-10 ms rewinds in 2026-04-26 step-2 verification,
    // which caused GLIM to reject every subsequent IMU sample after one
    // outlier-future sample landed first. Clamp each stream's published
    // stamp to be strictly greater than the previous one (bump by 1 ns
    // when the natural `now()` would regress). Same clock domain for both
    // streams (wall-clock from container), so no cross-stream alignment
    // is needed beyond per-stream monotonicity.
    rclcpp::Time last_imu_stamp_   = rclcpp::Time(0, 0, RCL_ROS_TIME);
    rclcpp::Time last_lidar_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // Subscribers
    rclcpp::Subscription<fs_msgs::msg::ControlCommand>::SharedPtr control_cmd_sub_;
    // /signal/ebs — autonomy-initiated emergency stop. On first message the
    // bridge applies a full-brake command then disables api_control so no
    // subsequent setCarControls can release the brake (real-car EBS analog).
    //
    // /signal/ebs_reset — release the latch. Published by the control node
    // on init so a fresh session always starts with controls accepted, even
    // if the previous session ended with a latched EBS. Without this, the
    // bridge would silently drop every setCarControls until dv_pipeline_stack was
    // restarted (the flag had no reset path).
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr ebs_request_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr ebs_reset_sub_;
    bool ebs_triggered_ = false;
    rclcpp::Service<fs_msgs::srv::Reset>::SharedPtr reset_srv_;

    // TF
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timers
    rclcpp::TimerBase::SharedPtr go_signal_timer_;
    rclcpp::TimerBase::SharedPtr extra_info_timer_;
    rclcpp::TimerBase::SharedPtr track_publish_timer_;
    rclcpp::TimerBase::SharedPtr static_tf_timer_;
    rclcpp::TimerBase::SharedPtr tire_loads_timer_;

    // Config
    std::string mission_name_ = "trackdrive";
    std::string track_name_ = "A";
    bool competition_mode_ = false;
    // UDP receiver — owns the sensor + LiDAR UDP listener threads.
    // start() called unconditionally from initializeConnection (#322
    // retired the TCP and UDS LiDAR transports; UDP is the only path
    // now). The LiDAR callback synthesises a LidarChunkHeader and
    // calls onLidarFrame so the publish path stays as it was.
    UdpReceiver udp_receiver_;

    // Sensor mount offsets (ROS body frame, metres). Queried once at
    // connection time via `getSensorOffset <name>` so the static TFs the
    // bridge publishes follow settings.json instead of stale hardcoded
    // numbers.
    struct Vec3 { double x = 0.0; double y = 0.0; double z = 0.0; bool valid = false; };
    Vec3 lidar_offset_;

    // Spawn position captured once at connection time via simGetVehiclePose.
    // resetSrvCb teleports back here; position-only (no quaternion) to avoid
    // the ENU↔UE5 ~90° yaw drift described in feedback_reset_orientation.md.
    struct HomePose { double x = 0.0; double y = 0.0; double z = 0.3; bool valid = false; };
    HomePose home_pose_;

    // Helper: query getSensorOffset for a single sensor, parse into Vec3.
    Vec3 querySensorOffset(const std::string& name);

    // Per-sensor publish rates (divisors of the 400Hz sensor stream):
    //   IMU  → publish every frame    (400 Hz)
    //   GSS / TF / Odom → every 4    (100 Hz)
    //   GPS  → every 40              ( 10 Hz)
    uint64_t sensor_frame_count_ = 0;

    // Noise params
    double gps_position_noise_std_ = 0.0;
    double imu_accel_noise_std_ = 0.0;
    double imu_gyro_noise_std_ = 0.0;
    double gss_velocity_noise_std_ = 0.0;
    // Maximum front-wheel angle (radians). The plugin sends steering
    // as a normalized [-1, 1] axis input; we convert at publish time
    // for /fsds/steering_angle so downstream consumers see SI units.
    // 0.5 rad ≈ 28.6° matches the IFS-08 URDF rack limit in
    // pipeline/coche_urdf/urdf/ifs_08.urdf.
    //
    // TODO(braking-steering): authoritative ISC_IFS_08.xlsx MONO sheet
    // gives turning radius 4.5 m + wheelbase 1.570 m → max δ ≈ 0.336
    // rad. Changing this default will reduce controller authority —
    // held until Sandra confirms + controller speed/lookahead is
    // re-tuned. Tracked in issue #462.
    double max_steering_angle_rad_ = 0.5;

    // Steering ratio (steering wheel angle / road-wheel angle). The
    // Bosch LWS measures the wheel column rotation; the
    // OdometryFilter consumes a road-wheel angle δ. This ratio bridges
    // them on both publish paths.
    // TODO(braking-steering): no explicit ratio in the IFS-08 model
    // package. 9.35 is derived from ±180° lock-to-lock at the wheel +
    // max road δ = 0.336 rad (quick-rack motorsport convention).
    // Update when Sandra confirms the rack design.
    double steering_ratio_ = 9.35;

    // LWS sensor model — frozen per-session bias + hysteresis state.
    // Constructed in initializePublishers() once parameters are read.
    std::unique_ptr<ifssim_bridge::LwsSteeringSensor> lws_sensor_;

    // Latest commanded steering δ_road (rad) cached from the sensor
    // stream. The 100 Hz LWS publish timer reads this, applies the
    // LWS error model, and publishes both /lws/* and /steering_angle.
    // Decoupling lets the LWS topics keep their 10 ms cadence even if
    // the underlying UDP SensorFrame is slow / bursty.
    std::atomic<double> latest_steering_cmd_rad_{0.0};

    // 100 Hz LWS publish timer. Kept as a member so the destructor
    // can release the rclcpp::Timer cleanly.
    rclcpp::TimerBase::SharedPtr lws_publish_timer_;
    void lwsPublishTimerCb();

    void parseNoiseSettings(const std::string& settings_json);

    // Helper: open a raw TCP socket and send a command
    int openStreamSocket(const std::string& command);
};
