/**
 * IFSSIM ROS2 Wrapper — TCP push model.
 *
 * Sensor data streams continuously from the sim over persistent TCP connections.
 * No polling. The sim pushes binary frames at engine tick rate (~100Hz).
 * Control commands and referee queries use traditional TCP request-response.
 * Camera sensors were removed in perf/strip-cameras.
 */

#include "ifssim_ros_wrapper.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <fs_msgs/msg/cone.hpp>
#include <sstream>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <limits>

#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

using namespace std::chrono_literals;

IFSSIMRosWrapper::IFSSIMRosWrapper(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& host, int port, double timeout_sec)
    : node_(node), host_(host), port_(port), timeout_sec_(timeout_sec)
{
    mission_name_ = node_->declare_parameter<std::string>("mission_name", "trackdrive");
    track_name_ = node_->declare_parameter<std::string>("track_name", "A");
    competition_mode_ = node_->declare_parameter<bool>("competition_mode", false);
    // Maximum front-wheel angle (radians). The UE5 plugin sends
    // steering as a normalized [-1, 1] axis input through
    // SensorFrame.steering; we convert to radians at the publish
    // site so /fsds/steering_angle is in SI units (the contract
    // sim_supervisor's OdometryFilter expects post-#383). 0.5 rad
    // matches the IFS-08 URDF rack limit; raise via launch arg if
    // the plugin's max-axis-to-angle mapping changes.
    max_steering_angle_rad_ = node_->declare_parameter<double>(
        "max_steering_angle_rad", 0.5);

    // ----- LWS (Bosch Steering Wheel Angle Sensor) model — #462 -----
    // The bridge publishes the post-decode floating-point view that
    // DV-PC would see after uDV forwards the CAN frame. Parameters
    // expose every datasheet figure so a yaml override can match
    // healthy / faulty sensor scenarios without recompiling.
    steering_ratio_ = node_->declare_parameter<double>(
        "steering_ratio", 9.35);
    const double lws_publish_hz = node_->declare_parameter<double>(
        "lws_publish_hz", ifssim_bridge::kLwsRateHz);  // 100 Hz default
    {
        ifssim_bridge::LwsParams lp;
        lp.nonlinearity_deg = node_->declare_parameter<double>(
            "lws_nonlinearity_deg", ifssim_bridge::kLwsNonlinearityDeg);
        lp.hysteresis_deg = node_->declare_parameter<double>(
            "lws_hysteresis_deg", ifssim_bridge::kLwsHysteresisDeg);
        lp.noise_std_deg = node_->declare_parameter<double>(
            "lws_noise_std_deg", 0.02);
        lp.resolution_deg = node_->declare_parameter<double>(
            "lws_resolution_deg", ifssim_bridge::kLwsResolutionDeg);
        // NaN sentinel = "draw randomly within ±nonlinearity_deg".
        lp.fixed_nonlinearity_bias_deg = node_->declare_parameter<double>(
            "lws_fixed_nonlinearity_bias_deg",
            std::numeric_limits<double>::quiet_NaN());
        const int seed_param = node_->declare_parameter<int>("lws_seed", 0);
        lp.seed = static_cast<uint32_t>(seed_param);
        lws_sensor_ = std::make_unique<ifssim_bridge::LwsSteeringSensor>(lp);
        RCLCPP_INFO(node_->get_logger(),
            "LWS sensor model: ratio=%.2f, frozen bias=%.3f deg, %.0f Hz",
            steering_ratio_, lws_sensor_->nonlinearity_bias_deg(),
            lws_publish_hz);
    }
    // Period clamp: parameters out of plausible range are bugs.
    const auto lws_period_ms = std::chrono::milliseconds(
        static_cast<int>(std::round(1000.0 / std::max(1.0, lws_publish_hz))));
    lws_publish_timer_ = node_->create_wall_timer(
        lws_period_ms,
        std::bind(&IFSSIMRosWrapper::lwsPublishTimerCb, this));
    // `lidar_transport` and `lidar_uds_path` parameters were removed in
    // #322. LiDAR is now always UDP via UdpReceiver — #321 marked the
    // TCP and UDS paths soft-deprecated, this PR follows through.

    // Opt-in subsampled LiDAR cloud for visualisation. 0 disables it
    // entirely (production default; no extra publisher, no per-frame
    // subsample work). Set via the `LIDAR_VIZ_DECIMATION` env var in
    // docker-compose.yml or as a launch parameter override. See
    // onLidarFrame() for the publish path and REFERENCE.md §7
    // for the operator-facing rationale.
    {
        const int dec = node_->declare_parameter<int>("lidar_viz_decimation", 0);
        lidar_viz_decimation_ = (dec >= 2) ? static_cast<uint32_t>(dec) : 0;
        if (dec > 0 && dec < 2) {
            RCLCPP_WARN(node_->get_logger(),
                "lidar_viz_decimation=%d clamped to disabled (need >=2 for any actual subsampling)",
                dec);
        }
    }

    initializeConnection();
    initializePublishers();
    initializeSubscribers();
    initializeTimers();
    startStreaming();
}

IFSSIMRosWrapper::~IFSSIMRosWrapper()
{
    streaming_ = false;
    // Close both stream TCP fds so the blocking recv() in
    // sensorStreamThread / lidarStreamThread wakes up. UdpReceiver
    // (used only on the UDP-fallback path) shuts itself down through
    // its own destructor.
    int sfd = sensor_stream_fd_.exchange(-1);
    if (sfd >= 0) close(sfd);
    int lfd = lidar_stream_fd_.exchange(-1);
    if (lfd >= 0) close(lfd);
    // Wake the publish-threads out of their condition_variable waits so
    // they can observe streaming_=false and exit.
    sensor_pub_cv_.notify_all();
    lidar_pub_cv_.notify_all();
    if (sensor_thread_.joinable()) sensor_thread_.join();
    if (lidar_thread_.joinable()) lidar_thread_.join();
    if (sensor_pub_thread_.joinable()) sensor_pub_thread_.join();
    if (lidar_pub_thread_.joinable()) lidar_pub_thread_.join();
}

int IFSSIMRosWrapper::openStreamSocket(const std::string& command)
{
    // Pre-#322 this had a leading AF_UNIX branch that opened
    // /tmp/ifssim_streams/lidar.sock when `command == "streamLidar"`
    // and `lidar_uds_path_` was set, falling back to TCP on failure.
    // The UDS LiDAR sender on the plugin side was retired in #322;
    // the only surviving caller is the sensor stream which always
    // wanted TCP anyway, so the function collapses to the TCP path.
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) <= 0) {
        struct hostent* he = gethostbyname(host_.c_str());
        if (!he) { close(sock); return -1; }
        memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    }

    if (::connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(sock);
        return -1;
    }

    // Disable Nagle for low latency
    int flag = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    // Bump SO_RCVBUF to 16 MB on stream sockets. Kernel default is
    // ~256 KB on most platforms, which only holds ~1/8 of a single
    // LiDAR scan at the Hesai ATX_S01 datasheet rate (174 k pts × 12
    // bytes = ~2 MB/scan). When the recv buffer fills, the TCP window
    // closes and the plugin's send blocks; if the publish thread on
    // either side momentarily stalls (foxglove fan-out, downstream
    // backpressure, GC pause), the buffer-stuck timeout in
    // FFSDSRpcServer::SendAll fires and the plugin tears down the
    // stream. Larger RCVBUF gives ~8 scans of headroom — enough to
    // soak up a 100–200 ms publish-thread hiccup without disconnect.
    // Linux may need `sysctl net.core.rmem_max` raised above 16 MB
    // to accept the request; the kernel silently caps to rmem_max.
    int rcvbuf = 16 * 1024 * 1024;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) != 0) {
        // Best-effort; the kernel may cap below the request. Not fatal.
        // The original 256 KB default is what we had before — log so the
        // operator notices if rmem_max is the limit.
        RCLCPP_WARN(node_->get_logger(),
                    "openStreamSocket: SO_RCVBUF setsockopt failed; using kernel default");
    }

    // Send the stream command
    std::string msg = command + "\n";
    send(sock, msg.c_str(), msg.size(), 0);

    // Read exactly "OK\n" (3 bytes) — do NOT read more or we'll consume the start
    // of the first sensor frame (sent at 400Hz immediately after the ACK).
    char buf[4];
    size_t total = 0;
    while (total < 3) {
        ssize_t n = recv(sock, buf + total, 3 - total, 0);
        if (n <= 0) { close(sock); return -1; }
        total += n;
    }
    buf[3] = '\0';

    return sock;
}

void IFSSIMRosWrapper::initializeConnection()
{
    client_ = std::make_unique<TcpClient>();
    if (!client_->connect(host_, port_, timeout_sec_)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to connect command client to %s:%d", host_.c_str(), port_);
        return;
    }

    client_->sendBool("enableApiControl");

    // Camera discovery removed in perf/strip-cameras — cameras no longer
    // exist on the sim side. The dedicated camera TCP client is gone too.

    if (client_->sendBool("ping")) {
        RCLCPP_INFO(node_->get_logger(), "IFSSIM connected (TCP push model)");
    }

    std::string settings = client_->sendCommand("getSettingsString");
    parseNoiseSettings(settings);

    // Cache sensor mount offsets from the plugin so static TFs don't
    // drift out of sync with settings.json. Queried once; offsets are
    // constructor-time constants on the sim side and won't change until
    // the sim is restarted.
    lidar_offset_ = querySensorOffset("lidar");

    // Capture spawn position for the /reset service. Must happen after the
    // track is loaded (the vehicle is already placed at the start gate by
    // the time the bridge connects), so this is the correct reset target.
    std::string pose_resp = client_->sendCommand("simGetVehiclePose");
    if (!pose_resp.empty() && pose_resp.find("\"error\"") == std::string::npos) {
        home_pose_.x = client_->parseDouble(pose_resp, "x");
        home_pose_.y = client_->parseDouble(pose_resp, "y");
        home_pose_.z = client_->parseDouble(pose_resp, "z");
        home_pose_.valid = true;
        RCLCPP_INFO(node_->get_logger(),
            "Home pose captured: (%.3f, %.3f, %.3f) ENU", home_pose_.x, home_pose_.y, home_pose_.z);
    } else {
        RCLCPP_WARN(node_->get_logger(), "simGetVehiclePose failed — /reset will be a no-op");
    }
}

IFSSIMRosWrapper::Vec3 IFSSIMRosWrapper::querySensorOffset(const std::string& name)
{
    Vec3 v;
    if (!client_ || !client_->isConnected()) return v;
    std::string resp = client_->sendCommand("getSensorOffset " + name);
    if (resp.empty() || resp.find("\"error\"") != std::string::npos) {
        RCLCPP_WARN(node_->get_logger(),
            "getSensorOffset(%s) failed: %s — static TF for this sensor will be skipped",
            name.c_str(), resp.empty() ? "no response" : resp.c_str());
        return v;
    }
    v.x = client_->parseDouble(resp, "x");
    v.y = client_->parseDouble(resp, "y");
    v.z = client_->parseDouble(resp, "z");
    v.valid = true;
    RCLCPP_INFO(node_->get_logger(),
        "Sensor '%s' mount: (%.3f, %.3f, %.3f) m (REP-103 body frame)",
        name.c_str(), v.x, v.y, v.z);
    return v;
}

void IFSSIMRosWrapper::initializePublishers()
{
    // High-rate sensors use BEST_EFFORT QoS for the same reason /lidar/Lidar1
    // does (see comment below). With the default RELIABLE keep_last(10), a
    // single slow subscriber stalled the publish thread → kernel TCP recv
    // buffer filled → plugin SendAll hit its 1 s timeout → stream tear-down,
    // and /imu / /gps fell to 0 Hz under pipeline load. Sensor topics are a
    // "latest sample wins" stream by nature; drops are correct, backpressure
    // is not. GPS at 10 Hz is the marginal case — kept BEST_EFFORT for
    // consistency since its subscribers (none currently reliable-only) can
    // tolerate the rare drop.
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    // /clock — sim-time source for the pipeline (use_sim_time). ClockQoS is
    // RELIABLE + TRANSIENT_LOCAL so a node spinning up late still latches the
    // most recent time instead of blocking at t=0 until the next tick.
    clock_pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::ClockQoS());
    gps_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("gps", sensor_qos);
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu", sensor_qos);
    gss_pub_ = node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("gss", sensor_qos);
    // /motor_rpm — engine RPM straight from Chaos VehicleMovement
    // (FSDSVehiclePawn::GetCarState → SensorFrame.rpm). Real-car parity:
    // IFS-08 inverter publishes motor RPM on CAN at 100 Hz. cone_slam
    // consumes it as a body-frame longitudinal velocity factor:
    //   v_x = rpm × (2π × WheelRadius / GearRatio) / 60
    // For IFS-08 (WheelRadius=0.228 m, GearRatio=2.909): v ≈ rpm × 0.00821 m/s.
    motor_rpm_pub_ = node_->create_publisher<std_msgs::msg::Float32>("motor_rpm", sensor_qos);
    // /steering_angle — front-wheel angle (rad), post-LWS sensor model.
    // Now driven by the 100 Hz lws_publish_timer_ rather than the
    // per-UDP-frame path so the cadence matches the real CAN sensor
    // (datasheet "100 Hz / 10 ms"). The underlying signal is the
    // commanded δ cached in latest_steering_cmd_rad_ from the sensor
    // stream; the timer multiplies it by steering_ratio_ to get the
    // steering wheel angle the LWS would see, runs the sensor model,
    // then publishes both the road-wheel-angle view here and the
    // sensor-faithful wheel-angle view on /lws/*. See #462.
    steering_angle_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
        "steering_angle", sensor_qos);
    lws_steering_wheel_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
        "lws/steering_wheel_angle_rad", sensor_qos);
    // /brake_pressure — commanded brake authority [0, 1], echoed from
    // the autonomy's last ControlCommand.brake. Phase 3 (#383) input
    // to OdometryFilter — when brake > threshold, RPM-derived v_x is
    // unreliable (drive wheels can lock) and the complementary filter
    // collapses α_vx toward zero, falling back to IMU integration.
    brake_pressure_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
        "brake_pressure", sensor_qos);
    // /tire_loads — vertical load Fz at each wheel [N], order [FL, FR, RL, RR].
    // Sourced from Chaos's per-wheel SpringForce via the plugin RPC, so this
    // is the same Fz the wheel solver is using to compute grip this tick.
    // Foxglove visualises Float32MultiArray as a small bar chart out of the
    // box; the autonomy can read it for grip-aware velocity targets.
    tire_loads_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        "tire_loads", sensor_qos);
    // /lidar/Lidar1 uses BEST_EFFORT QoS (rather than the default RELIABLE
    // keep_last(10)) so a slow subscriber — most notably the numba-JIT
    // cone-detection node during its first ~15 s of warmup, but also any
    // foxglove_bridge consumer that pauses to render a frame — drops
    // messages locally instead of backpressuring the bridge's lidar_thread.
    // Without this, the lidar_thread blocks inside `publish()`, the kernel
    // TCP recv buffer fills, the *plugin's* TCP send buffer fills, the
    // plugin's SendAll hits its 1 s timeout, the plugin closes the stream,
    // and the bridge sees a dead socket → reconnect cascade. Net effect on
    // a real autocross run: LiDAR drops from 10 Hz to ~1 Hz the moment
    // pipeline subscribers come online. Sensor data is fundamentally a
    // best-effort stream — drops are fine, backpressure is not.
    auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort();
    lidar_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lidar/Lidar1", lidar_qos);
    if (lidar_viz_decimation_ >= 2) {
        // Same QoS as the full cloud — BEST_EFFORT lets a slow tab drop
        // frames instead of backpressuring the bridge.
        lidar_viz_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "lidar/Lidar1/viz", lidar_qos);
        RCLCPP_INFO(node_->get_logger(),
            "/lidar/Lidar1/viz enabled — every %u-th point published alongside the full cloud",
            lidar_viz_decimation_);
    }
    go_signal_pub_ = node_->create_publisher<fs_msgs::msg::GoSignal>("signal/go", 10);
    // Published edge-triggered when the referee's bFinished flips false->true;
    // consumers (e.g. the control node) brake the car to end the event cleanly.
    finished_signal_pub_ = node_->create_publisher<fs_msgs::msg::FinishedSignal>("signal/finished", 10);

    // Camera publishers removed in perf/strip-cameras — see header comment.

    if (!competition_mode_) {
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("testing_only/odom", sensor_qos);
        auto qos = rclcpp::QoS(1).transient_local();
        track_pub_ = node_->create_publisher<fs_msgs::msg::Track>("testing_only/track", qos);
        extra_info_pub_ = node_->create_publisher<fs_msgs::msg::ExtraInfo>("testing_only/extra_info", 10);
    }

    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    RCLCPP_INFO(node_->get_logger(), "Publishers initialized (competition_mode: %s)",
        competition_mode_ ? "true" : "false");
}

void IFSSIMRosWrapper::initializeSubscribers()
{
    control_cmd_sub_ = node_->create_subscription<fs_msgs::msg::ControlCommand>(
        "control_command", 10,
        std::bind(&IFSSIMRosWrapper::controlCommandCb, this, std::placeholders::_1));

    // Latched QoS so a late-joining publisher that already fired its EBS
    // signal still triggers a handler on connect. Only one in-flight msg
    // needed — EBS is one-shot.
    auto ebs_qos = rclcpp::QoS(1).transient_local();
    ebs_request_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "signal/ebs", ebs_qos,
        std::bind(&IFSSIMRosWrapper::ebsRequestCb, this, std::placeholders::_1));

    // /signal/ebs_reset — explicit release of the EBS latch. Without this
    // the bridge's `ebs_triggered_` had no way to flip back to false after
    // a session ended with the autonomous-stop logic firing — every
    // subsequent Start Session would silently drop setCarControls until
    // the user restarted dv_pipeline_stack. The control node publishes this once
    // on init, so a fresh session always starts with controls accepted.
    ebs_reset_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "signal/ebs_reset", ebs_qos,
        std::bind(&IFSSIMRosWrapper::ebsResetCb, this, std::placeholders::_1));

    reset_srv_ = node_->create_service<fs_msgs::srv::Reset>(
        "reset",
        std::bind(&IFSSIMRosWrapper::resetSrvCb, this, std::placeholders::_1, std::placeholders::_2));
}

void IFSSIMRosWrapper::initializeTimers()
{
    // Camera timer removed in perf/strip-cameras.
    go_signal_timer_ = node_->create_wall_timer(1000ms, std::bind(&IFSSIMRosWrapper::goSignalTimerCb, this));
    static_tf_timer_ = node_->create_wall_timer(1000ms, std::bind(&IFSSIMRosWrapper::staticTfCb, this));
    // /tire_loads at 20 Hz — fast enough that the autonomy controller
    // (40 Hz) sees each Fz update at most one tick stale, slow enough
    // that we don't flood the bridge's command client (this fetches via
    // RPC on the same TCP connection as setCarControls / getCarState).
    tire_loads_timer_ = node_->create_wall_timer(
        50ms, std::bind(&IFSSIMRosWrapper::tireLoadsTimerCb, this));

    if (!competition_mode_) {
        extra_info_timer_ = node_->create_wall_timer(1000ms, std::bind(&IFSSIMRosWrapper::extraInfoTimerCb, this));
        track_publish_timer_ = node_->create_wall_timer(5000ms, std::bind(&IFSSIMRosWrapper::trackPublishCb, this));
    }
}

void IFSSIMRosWrapper::startStreaming()
{
    // Open sensor stream (TCP, always — sensors don't have the
    // fragmentation pathology LiDAR did, the ~40 KB/s rate isn't
    // bandwidth-bound, and the TCP push has been the production
    // sensor transport since the plugin was first written).
    sensor_stream_fd_ = openStreamSocket("streamSensors");
    if (sensor_stream_fd_ >= 0) {
        RCLCPP_INFO(node_->get_logger(), "Sensor stream connected");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open sensor stream");
    }

    // LiDAR transport selection. Default TCP (PR-#482); UDP available
    // as an escape hatch via the IFSSIM_LIDAR_TRANSPORT environment
    // variable for the case where the modern Docker Desktop loopback
    // turns out to still bite throughput on a specific OS/version
    // combination we haven't seen yet. UDP keeps the
    // chunked-reassembly path that #322 introduced; TCP is the
    // streaming pattern from PR-#482.
    {
        const char* env_lit = std::getenv("IFSSIM_LIDAR_TRANSPORT");
        std::string env = env_lit ? env_lit : "";
        // Lowercase compare so "TCP"/"Tcp"/"tcp" all work.
        for (auto& c : env) c = (char)std::tolower((unsigned char)c);
        lidar_transport_ = (env == "udp") ? LidarTransport::Udp : LidarTransport::Tcp;
    }

    if (lidar_transport_ == LidarTransport::Tcp) {
        lidar_stream_fd_ = openStreamSocket("streamLidar");
        if (lidar_stream_fd_ >= 0) {
            RCLCPP_INFO(node_->get_logger(), "LiDAR transport: TCP (streamLidar, PR-#482)");
        } else {
            RCLCPP_ERROR(node_->get_logger(),
                "LiDAR transport: TCP requested but openStreamSocket failed — "
                "will retry via reconnect path");
        }
    } else {
        // Fallback path: same UDP receiver wiring the production path
        // used pre-#482. Keep the chunked-UDP receiver thread + chunk-
        // reassembly callback alive only on the UDP branch so the
        // hot path doesn't pay for an unused listener.
        udp_receiver_.setLidarCallback(
            [this](int32_t total_points, int32_t channels,
                   int64_t lag_ns, int64_t sim_capture_ns,
                   const std::vector<float>& points) {
                LidarChunkHeader hdr{};
                hdr.magic = LIDAR_MAGIC;
                hdr.chunk_index = 0;
                hdr.total_chunks = 1;
                hdr.frame_id = 0;
                hdr.points_in_chunk = total_points;
                hdr.total_points = total_points;
                hdr.channels = channels;
                hdr.lag_ns = lag_ns;
                hdr.sim_capture_ns = sim_capture_ns;
                {
                    std::lock_guard<std::mutex> lock(lidar_pub_mutex_);
                    lidar_pending_ = PendingLidarFrame{hdr, points};
                }
                lidar_pub_cv_.notify_one();
            });
        // sensor_port=0 disables UdpReceiver's sensor listener; LiDAR
        // port 41500 chosen for cross-OS Docker Desktop compatibility
        // (non-adjacent to 41452, below Windows' 49152 dynamic range).
        // See FSDSGameMode.cpp for the port-choice trail.
        udp_receiver_.start(0, 41500);

        // PR-#482: the sim's UDP LiDAR broadcaster defaults OFF since
        // the production transport is now TCP. Explicitly turn it on
        // for the UDP-fallback branch so the chunked-UDP listener
        // actually receives data. (On the TCP branch above, we
        // intentionally leave the broadcaster off so the sim doesn't
        // spuriously send ~1.5 MB/s of LiDAR packets nobody's
        // listening for.)
        if (client_ && client_->isConnected()) {
            const std::string resp = client_->sendCommand("enableLidarUdpBroadcast");
            if (resp.find("true") == std::string::npos) {
                RCLCPP_WARN(node_->get_logger(),
                    "UDP fallback: enableLidarUdpBroadcast returned %s — "
                    "sim may not be sending UDP LiDAR; check sim build "
                    "predates PR-#482", resp.c_str());
            }
        }

        RCLCPP_INFO(node_->get_logger(),
            "LiDAR transport: UDP fallback (IFSSIM_LIDAR_TRANSPORT=udp, listening on 41500)");
    }

    streaming_ = true;

    sensor_thread_     = std::thread(&IFSSIMRosWrapper::sensorStreamThread,  this);
    sensor_pub_thread_ = std::thread(&IFSSIMRosWrapper::sensorPublishThread, this);
    if (lidar_transport_ == LidarTransport::Tcp) {
        lidar_thread_  = std::thread(&IFSSIMRosWrapper::lidarStreamThread,   this);
    }
    lidar_pub_thread_  = std::thread(&IFSSIMRosWrapper::lidarPublishThread,  this);

    // Kick off reconnect if either TCP stream failed (UE5 not in
    // Play mode yet is the common case). UDP receiver listens
    // passively and doesn't gate the reconnect decision.
    const bool tcp_lidar_failed =
        lidar_transport_ == LidarTransport::Tcp && lidar_stream_fd_ < 0;
    if (sensor_stream_fd_ < 0 || tcp_lidar_failed) {
        std::thread(&IFSSIMRosWrapper::triggerReconnect, this).detach();
    }
}

// =============================================================================
// Streaming threads — read continuous binary data from sim
// =============================================================================

static bool readExact(int fd, void* buf, size_t len)
{
    size_t total = 0;
    while (total < len) {
        ssize_t n = recv(fd, (char*)buf + total, len - total, 0);
        if (n <= 0) return false;
        total += n;
    }
    return true;
}

void IFSSIMRosWrapper::sensorStreamThread()
{
    RCLCPP_INFO(node_->get_logger(), "Sensor stream thread started");

    while (streaming_) {
        int fd = sensor_stream_fd_.load();
        if (fd < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        SensorFrame frame;
        if (!readExact(fd, &frame, sizeof(frame))) {
            RCLCPP_WARN(node_->get_logger(), "Sensor stream disconnected");
            int expected = fd;
            if (sensor_stream_fd_.compare_exchange_strong(expected, -1)) close(fd);
            triggerReconnect();
            continue;
        }

        if (frame.magic != SENSOR_MAGIC) continue;

        // Hand off to the publish thread. Single-slot buffer with
        // drop-oldest semantics — if the consumer hasn't yet drained the
        // previous frame, we overwrite it. The mutex is held only long
        // enough to swap the std::optional; publish() runs entirely
        // outside the lock on the consumer side. See the header comment
        // for the failure mode this prevents.
        {
            std::lock_guard<std::mutex> lock(sensor_pub_mutex_);
            sensor_pending_ = frame;
        }
        sensor_pub_cv_.notify_one();
    }
}

void IFSSIMRosWrapper::sensorPublishThread()
{
    RCLCPP_INFO(node_->get_logger(), "Sensor publish thread started");

    while (streaming_) {
        SensorFrame frame;
        {
            std::unique_lock<std::mutex> lock(sensor_pub_mutex_);
            sensor_pub_cv_.wait(lock, [this] {
                return !streaming_ || sensor_pending_.has_value();
            });
            if (!streaming_) break;
            frame = *sensor_pending_;
            sensor_pending_.reset();
        }

        // publish() runs OUTSIDE the lock so the recv thread can keep
        // pushing new frames. Whatever time DDS / subscriber back-
        // pressure costs is purely consumer-side; the recv loop never
        // sees it.
        onSensorFrame(frame);
    }

    RCLCPP_INFO(node_->get_logger(), "Sensor publish thread exiting");
}

// PR-#482: TCP LiDAR recv loop reinstated. Mirrors sensorStreamThread's
// architectural pattern (raw socket fd, readExact for partial recvs,
// drop-oldest single-slot handoff to the publish thread). Header file
// has the long failure-mode trail that motivated bringing TCP back
// after #322 retired it.
//
// Wire frame layout (matches plugin's StreamLidar):
//
//   [FFSDSLidarStreamHeader  24 B]   (magic / frame_id / channels / total_points / lag_ns)
//   [TotalPoints × 4 floats × 4 B]   (x, y, z, intensity)
//
// One wire frame per scan. No chunking — TCP is a byte stream, the
// kernel handles segmentation. readExact handles partial recvs on
// our side; the plugin's SendAll handles partial sends on its side.
void IFSSIMRosWrapper::lidarStreamThread()
{
    RCLCPP_INFO(node_->get_logger(), "LiDAR stream thread started (TCP, PR-#482)");

    // Local stream-header layout. Hand-rolled to avoid pulling the
    // plugin's UE5-Build headers into the ROS build; we control the
    // wire format and the static_assert on the plugin side guards
    // size compatibility (see FFSDSLidarStreamHeader). If you grow
    // the wire header on the plugin side, mirror it here AND bump a
    // wire-version field — old bridges cannot parse new headers.
    #pragma pack(push, 1)
    struct WireHeader {
        uint32_t magic;        // 0x4C494452 "LIDR"
        uint32_t frame_id;
        int32_t  channels;
        int32_t  total_points;
        int64_t  lag_ns;
        int64_t  sim_capture_ns;  // absolute UE sim capture time (ns), Option 2
    };
    #pragma pack(pop)
    static_assert(sizeof(WireHeader) == 32,
        "Bridge wire header out of sync with plugin's FFSDSLidarStreamHeader");

    // Reusable point-cloud receive buffer. Sized for the worst-case
    // Hesai ATX_S01 scan plus headroom so we don't reallocate every
    // scan. ~174 k points × 16 B/point + header = ~2.8 MB worst case;
    // 4 MB gives ~40 % headroom.
    std::vector<float> points;
    points.reserve(174000 * 4);

    while (streaming_) {
        int fd = lidar_stream_fd_.load();
        if (fd < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        WireHeader hdr;
        if (!readExact(fd, &hdr, sizeof(hdr))) {
            RCLCPP_WARN(node_->get_logger(), "LiDAR stream disconnected (header recv failed)");
            int expected = fd;
            if (lidar_stream_fd_.compare_exchange_strong(expected, -1)) close(fd);
            triggerReconnect();
            continue;
        }
        if (hdr.magic != LIDAR_MAGIC) {
            // Out-of-band garbage on the stream — desync.  Tearing
            // down the connection is the only safe recovery: we
            // can't reliably skip forward to the next header in a
            // byte stream without a framing escape. Reconnect will
            // re-send the "streamLidar\n" handshake and the plugin
            // will start fresh from the first scan.
            RCLCPP_WARN(node_->get_logger(),
                "LiDAR stream: bad magic 0x%08x — desync, reconnecting",
                hdr.magic);
            int expected = fd;
            if (lidar_stream_fd_.compare_exchange_strong(expected, -1)) close(fd);
            triggerReconnect();
            continue;
        }
        if (hdr.total_points <= 0 || hdr.total_points > 10'000'000) {
            // Sanity bound — drops obviously corrupt frames before
            // we resize() with a multi-GB count. 10 M points is
            // ~57× the Hesai ATX_S01's PointsPerScan; well clear of
            // any plausible real frame.
            RCLCPP_WARN(node_->get_logger(),
                "LiDAR stream: implausible total_points=%d — desync, reconnecting",
                hdr.total_points);
            int expected = fd;
            if (lidar_stream_fd_.compare_exchange_strong(expected, -1)) close(fd);
            triggerReconnect();
            continue;
        }

        points.resize(static_cast<size_t>(hdr.total_points) * 4);
        const size_t payload_bytes = points.size() * sizeof(float);
        if (!readExact(fd, points.data(), payload_bytes)) {
            RCLCPP_WARN(node_->get_logger(), "LiDAR stream disconnected (payload recv failed)");
            int expected = fd;
            if (lidar_stream_fd_.compare_exchange_strong(expected, -1)) close(fd);
            triggerReconnect();
            continue;
        }

        // Hand off to publish thread. Synthesise a LidarChunkHeader
        // from the stream header so onLidarFrame's signature
        // (transport-agnostic by design) doesn't need to change.
        // chunk_index/total_chunks/points_in_chunk are unused by
        // onLidarFrame in production; we set them to 0/1/total
        // for self-documenting "this is a single-frame delivery"
        // semantics.
        LidarChunkHeader pub_hdr{};
        pub_hdr.magic = LIDAR_MAGIC;
        pub_hdr.chunk_index = 0;
        pub_hdr.total_chunks = 1;
        pub_hdr.frame_id = hdr.frame_id;
        pub_hdr.points_in_chunk = hdr.total_points;
        pub_hdr.total_points = hdr.total_points;
        pub_hdr.channels = hdr.channels;
        pub_hdr.lag_ns = hdr.lag_ns;
        pub_hdr.sim_capture_ns = hdr.sim_capture_ns;

        {
            std::lock_guard<std::mutex> lock(lidar_pub_mutex_);
            lidar_pending_ = PendingLidarFrame{pub_hdr, points};
        }
        lidar_pub_cv_.notify_one();
    }

    RCLCPP_INFO(node_->get_logger(), "LiDAR stream thread exiting");
}

void IFSSIMRosWrapper::lidarPublishThread()
{
    RCLCPP_INFO(node_->get_logger(), "LiDAR publish thread started");

    while (streaming_) {
        PendingLidarFrame frame;
        {
            std::unique_lock<std::mutex> lock(lidar_pub_mutex_);
            lidar_pub_cv_.wait(lock, [this] {
                return !streaming_ || lidar_pending_.has_value();
            });
            if (!streaming_) break;
            frame = std::move(*lidar_pending_);
            lidar_pending_.reset();
        }

        // publish() runs OUTSIDE the lock so the recv thread can keep
        // pushing new frames into the slot. Whatever time this takes
        // (PointCloud2 serialization + DDS fan-out + downstream subscriber
        // back-pressure) is purely consumer-side; the recv loop never
        // sees it.
        onLidarFrame(frame.header, frame.points.data());
    }

    RCLCPP_INFO(node_->get_logger(), "LiDAR publish thread exiting");
}

void IFSSIMRosWrapper::triggerReconnect()
{
    std::lock_guard<std::mutex> lock(reconnect_mutex_);

    // Another thread may have already completed the reconnect while
    // we waited. Gate on whichever streams this transport actually
    // uses: sensors always (TCP), LiDAR only on the TCP path (UDP
    // listens passively, no per-connection state to rebuild).
    const bool need_sensor = sensor_stream_fd_ < 0;
    const bool need_lidar  =
        lidar_transport_ == LidarTransport::Tcp && lidar_stream_fd_ < 0;
    if (!need_sensor && !need_lidar) return;

    RCLCPP_INFO(node_->get_logger(), "Reconnecting to IFSSIM (level reset?)...");

    // Only recreate the command client if it's actually broken. A transient
    // stream glitch (partial LiDAR frame, brief sensor recv hiccup) does NOT
    // mean UE5 is gone — the streams use *separate* TCP connections from
    // the command client. Tearing down and rebuilding `client_` on every
    // stream hiccup was breaking control: each reconnect dropped any
    // in-flight setCarControls and re-issued enableApiControl, while
    // `controlCommandCb` reads `client_` lock-free.
    //
    // Symptom that surfaced today: with a 10 Hz LiDAR push and the plugin
    // occasionally closing its push socket mid-frame, this function fired
    // ~once per second. Each fire tore down the command client for ~30 ms,
    // so a meaningful fraction of `setCarControls` were silently dropped.
    // The car drove like the controls were lagging by 1 s.
    if (!client_ || !client_->isConnected()) {
        // Belt-and-suspenders: clear any latched EBS state when we reconnect
        // the *command* client. The primary release path is
        // /signal/ebs_reset published by the control node on init, but if
        // the control node crashed without publishing — or if UE5 itself was
        // restarted — we don't want a stale flag to keep dropping
        // setCarControls forever. Doing this here (instead of on every
        // stream-only reconnect) prevents the EBS flag from being cleared
        // mid-session by spurious LiDAR-stream reconnects.
        ebs_triggered_ = false;

        int attempt = 0;
        while (streaming_) {
            client_ = std::make_unique<TcpClient>();
            if (client_->connect(host_, port_, 3.0)) {
                client_->sendBool("enableApiControl");
                RCLCPP_INFO(node_->get_logger(), "Command client reconnected (attempt %d)", ++attempt);
                // UE5 was restarted — car is back at spawn. Re-capture home pose
                // so /reset targets the new session's start position, not the
                // stale one from the previous session.
                std::string pose_resp = client_->sendCommand("simGetVehiclePose");
                if (!pose_resp.empty() && pose_resp.find("\"error\"") == std::string::npos) {
                    home_pose_.x = client_->parseDouble(pose_resp, "x");
                    home_pose_.y = client_->parseDouble(pose_resp, "y");
                    home_pose_.z = client_->parseDouble(pose_resp, "z");
                    home_pose_.valid = true;
                    RCLCPP_INFO(node_->get_logger(),
                        "Home pose re-captured after reconnect: (%.3f, %.3f, %.3f) ENU",
                        home_pose_.x, home_pose_.y, home_pose_.z);
                }
                break;
            }
            RCLCPP_INFO(node_->get_logger(), "Reconnect attempt %d failed, retrying in 2s...", ++attempt);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    // Reconnect sensor stream
    while (streaming_ && sensor_stream_fd_ < 0) {
        int fd = openStreamSocket("streamSensors");
        if (fd >= 0) {
            sensor_stream_fd_.store(fd);
            RCLCPP_INFO(node_->get_logger(), "Sensor stream reconnected");
        } else {
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    // Reconnect LiDAR stream on the TCP path. The UDP fallback's
    // listener is passive and self-recovers when the plugin sends
    // the next datagram, so it doesn't need explicit reconnect.
    if (lidar_transport_ == LidarTransport::Tcp) {
        while (streaming_ && lidar_stream_fd_ < 0) {
            int fd = openStreamSocket("streamLidar");
            if (fd >= 0) {
                lidar_stream_fd_.store(fd);
                RCLCPP_INFO(node_->get_logger(), "LiDAR stream reconnected (TCP)");
            } else {
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }
    }
}

// =============================================================================
// Frame handlers — publish ROS2 topics
// =============================================================================

void IFSSIMRosWrapper::onSensorFrame(const SensorFrame& f)
{
    // Capture clock = the plugin's UE game/sim time (ns), NOT the bridge's
    // wall clock. Stamping sensor messages and /clock from this is the whole
    // fix for the "clock stretch": every consumer with use_sim_time now
    // integrates on sim seconds (the ~0.77×-real physics clock) instead of
    // wall seconds, so EKF distance and SLAM heading stop over-counting ~30%.
    // The bridge itself stays on wall time (it's the /clock source — see
    // clock_pub_ comment in the header), so node_->now() is unused here.
    rclcpp::Time now(static_cast<int64_t>(f.timestamp), RCL_ROS_TIME);
    ++sensor_frame_count_;

    // Sim-time rewind — IFSSIM restarted / level reloaded while the bridge
    // stayed up (see last_clock_stamp_ in the header). Frames arrive in order
    // on one TCP stream, so a step back beyond the jitter threshold can only
    // mean the plugin's game clock started over. Reset the per-session
    // guards so /clock follows the new sim time immediately instead of going
    // silent, and so the IMU clamp stops pinning stamps to old_time + 1 ns.
    // Downstream, rcl timers re-arm on a backward jump and tf2 buffers clear.
    // Both guards live on this (sensor) thread; the LiDAR clamp is reset on
    // its own thread in onLidarFrame().
    if (last_clock_stamp_.nanoseconds() > 0 &&
        now.nanoseconds() + kSimTimeRewindThresholdNs < last_clock_stamp_.nanoseconds())
    {
        RCLCPP_WARN(node_->get_logger(),
            "Sim time rewound %.1f s → %.1f s (IFSSIM restart / level reload?) "
            "— re-basing /clock and sensor stamp guards on the new session",
            last_clock_stamp_.seconds(), now.seconds());
        last_clock_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        last_imu_stamp_   = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    // Drive /clock from the sim stamp. /clock must be non-decreasing; the
    // game tick can repeat a sim ns across consecutive 400 Hz frames, so only
    // publish when time actually advanced (a repeat is a no-op, not a rewind).
    if (now > last_clock_stamp_) {
        rosgraph_msgs::msg::Clock clk;
        clk.clock = now;
        clock_pub_->publish(clk);
        last_clock_stamp_ = now;
    }

    // GPS — 10 Hz (every 40 frames of the 400 Hz stream)
    if (sensor_frame_count_ % 40 == 0)
    {
        sensor_msgs::msg::NavSatFix msg;
        msg.header.stamp = now;
        msg.header.frame_id = "fsds/GPS";
        msg.latitude = f.latitude;
        msg.longitude = f.longitude;
        msg.altitude = f.altitude;
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        double gps_var = gps_position_noise_std_ * gps_position_noise_std_;
        msg.position_covariance = {gps_var,0,0, 0,gps_var,0, 0,0,gps_var};
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        gps_pub_->publish(msg);
    }

    // IMU
    {
        // Monotonic guard — see last_imu_stamp_ comment in the header. GLIM
        // rejects any IMU sample whose stamp ≤ the previously-accepted one;
        // bump by 1 ns when node_->now() would regress so the publish stream
        // is strictly increasing.
        rclcpp::Time imu_stamp = now;
        if (last_imu_stamp_.nanoseconds() > 0 && imu_stamp <= last_imu_stamp_) {
            imu_stamp = last_imu_stamp_ + rclcpp::Duration::from_nanoseconds(1);
        }
        last_imu_stamp_ = imu_stamp;

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = imu_stamp;
        msg.header.frame_id = "fsds/IMU";
        // Convert UE5 body frame (left-handed: X=fwd, Y=right, Z=up) to
        // ROS REP-103 body frame (right-handed: X=fwd, Y=left, Z=up).
        // The UDP broadcaster forwards FSDSImuSensor's body-frame outputs
        // (accel was already body-framed in the sensor; gyro was made
        // body-framed by the same sensor file as of 2026-04-27).
        //
        // Linear acceleration is a polar vector — under the Y-axis basis
        // change det(R)=-1 it transforms as v → (vx, -vy, vz).
        //
        // Angular velocity is a pseudovector (axial vector). Under the
        // same basis change with det(R)=-1 the transformation gains an
        // extra det factor: ω → (-ωx, ωy, -ωz). Only the polar transform
        // (Y-flip) was applied originally; that produced the right
        // gravity vector but mirrored yaw direction, which the
        // 2026-04-27 Phase 2 drive made obvious — fast_LIMO integrated
        // turns the wrong way.
        //
        // The orientation quat is already ENU-converted via UEQuatToENU
        // upstream and needs no further flip here.
        msg.linear_acceleration.x =  f.accel_x;
        msg.linear_acceleration.y = -f.accel_y;
        msg.linear_acceleration.z =  f.accel_z;
        msg.angular_velocity.x = -f.gyro_x;
        msg.angular_velocity.y =  f.gyro_y;
        msg.angular_velocity.z = -f.gyro_z;
        msg.orientation.x = f.orient_x;
        msg.orientation.y = f.orient_y;
        msg.orientation.z = f.orient_z;
        msg.orientation.w = f.orient_w;
        msg.orientation_covariance = {1e-6,0,0, 0,1e-6,0, 0,0,1e-6};
        double gyro_var = imu_gyro_noise_std_ * imu_gyro_noise_std_;
        msg.angular_velocity_covariance = {gyro_var,0,0, 0,gyro_var,0, 0,0,gyro_var};
        double accel_var = imu_accel_noise_std_ * imu_accel_noise_std_;
        msg.linear_acceleration_covariance = {accel_var,0,0, 0,accel_var,0, 0,0,accel_var};
        imu_pub_->publish(msg);
    }

    // GSS + TF + Odom — 100 Hz (every 4 frames of the 400 Hz stream)
    if (sensor_frame_count_ % 4 == 0)
    {
        // GSS
        {
            geometry_msgs::msg::TwistWithCovarianceStamped msg;
            msg.header.stamp = now;
            msg.header.frame_id = vehicle_frame_id_;
            msg.twist.twist.linear.x = f.gss_vx;
            msg.twist.twist.linear.y = f.gss_vy;
            msg.twist.twist.linear.z = f.gss_vz;
            double gss_var = gss_velocity_noise_std_ * gss_velocity_noise_std_;
            msg.twist.covariance[0] = gss_var;
            msg.twist.covariance[7] = gss_var;
            msg.twist.covariance[14] = gss_var;
            gss_pub_->publish(msg);
        }

        // Motor RPM — straight from Chaos. Real IFS-08 publishes the
        // same field on CAN at 100 Hz; matching rate keeps sim/real
        // parity for the cone_slam velocity factor.
        {
            std_msgs::msg::Float32 msg;
            msg.data = f.rpm;
            motor_rpm_pub_->publish(msg);
        }

        // Cache the latest commanded steering δ (road-wheel angle, rad).
        // The 100 Hz lws_publish_timer_ reads this, runs it through the
        // LWS sensor model, and publishes both /lws/* and /steering_angle.
        // Decoupling the sensor cadence from the UDP frame cadence
        // matches the real car (CAN @ 100 Hz independent of whatever
        // IMU/RPM rates uDV forwards). See #462.
        latest_steering_cmd_rad_.store(
            static_cast<double>(f.steering) * max_steering_angle_rad_,
            std::memory_order_relaxed);

        // /brake_pressure — commanded brake authority [0, 1] echoed
        // from SensorFrame.brake. Proxy for hydraulic line pressure
        // in the absence of a Chaos brake-fluid model: when the
        // autonomy commands brake > threshold the drive wheels can
        // lock, RPM-derived v_x is no longer reliable, and the
        // OdometryFilter should fall back to IMU-only integration.
        // #383.
        {
            std_msgs::msg::Float32 msg;
            msg.data = f.brake;
            brake_pressure_pub_->publish(msg);
        }

        // TF odom → fsds/FSCar — REMOVED in PR #3 of the GLIM rebuild.
        // GLIM (LiDAR-IMU SLAM) owns odom → base_link now. Odometria_perfecta
        // continues to publish odom → fsds/FSCar from /fsds/testing_only/odom
        // until step 4 of the rebuild renames pipeline frame references and
        // step 5 deletes Odometria_perfecta entirely. The bridge no longer
        // needs to publish a duplicate sim-GT TF — and doing so would conflict
        // with GLIM's odom frame (multiple writers to the same TF parent
        // cause non-deterministic last-writer-wins behavior in TF2).

        // Odom (testing only). Both pose and twist are sourced from the
        // vehicle pawn's clean kinematics — no GSS sensor in the loop —
        // so this topic is fully ground-truth. The twist fill closes
        // issue #315 (was previously default-zero, which misled
        // consumers that expected GT velocity here; gt_pose_relay.py
        // had to finite-difference pose locally as a workaround).
        if (odom_pub_) {
            nav_msgs::msg::Odometry msg;
            msg.header.stamp = now;
            msg.header.frame_id = map_frame_id_;
            msg.child_frame_id = vehicle_frame_id_;
            msg.pose.pose.position.x = f.pos_x;
            msg.pose.pose.position.y = f.pos_y;
            msg.pose.pose.position.z = f.pos_z;
            msg.pose.pose.orientation.x = f.pose_qx;
            msg.pose.pose.orientation.y = f.pose_qy;
            msg.pose.pose.orientation.z = f.pose_qz;
            msg.pose.pose.orientation.w = f.pose_qw;
            double pos_var = gps_position_noise_std_ * gps_position_noise_std_;
            msg.pose.covariance[0] = pos_var;
            msg.pose.covariance[7] = pos_var;
            msg.pose.covariance[14] = pos_var;
            msg.pose.covariance[21] = 1e-6;
            msg.pose.covariance[28] = 1e-6;
            msg.pose.covariance[35] = 1e-6;
            // Clean ground-truth body-frame velocity (#315) + angular
            // velocity (closing the #315 follow-up gap). Same source —
            // pawn root component, no sensor in the loop. Tiny
            // covariance — this is GT, not a measurement; downstream
            // consumers that weight by inverse covariance still see
            // a finite floor without claiming literal zero uncertainty.
            msg.twist.twist.linear.x  = f.gt_vel_body_x;
            msg.twist.twist.linear.y  = f.gt_vel_body_y;
            msg.twist.twist.linear.z  = f.gt_vel_body_z;
            msg.twist.twist.angular.x = f.gt_ang_vel_body_x;
            msg.twist.twist.angular.y = f.gt_ang_vel_body_y;
            msg.twist.twist.angular.z = f.gt_ang_vel_body_z;
            constexpr double gt_var = 1e-9;
            msg.twist.covariance[0]  = gt_var;
            msg.twist.covariance[7]  = gt_var;
            msg.twist.covariance[14] = gt_var;
            msg.twist.covariance[21] = gt_var;
            msg.twist.covariance[28] = gt_var;
            msg.twist.covariance[35] = gt_var;
            odom_pub_->publish(msg);
        }
    }
}

void IFSSIMRosWrapper::onLidarFrame(const LidarChunkHeader& header, const float* points)
{
    int total_points = header.total_points;
    if (total_points <= 0) return;

    // Capture-time stamping. Preferred path (Option 2): the plugin tags each
    // scan with the ABSOLUTE UE sim time of capture (sim_capture_ns). Stamp
    // header.stamp straight from it — immune to GPU readback + transport +
    // DDS-backlog latency, which is what made the old lag_ns scheme drift
    // upward over a long run (lag only covered plugin capture-to-send, not
    // the rest of the pipeline). This is the same sim clock /clock runs on,
    // so under use_sim_time the LiDAR aligns with odom/IMU by construction.
    //
    // Fallback (old plugin build, sim_capture_ns == 0): the legacy #238
    // scheme — subtract the capture-to-send lag from the bridge's wall clock.
    // NOTE this fallback mixes clock domains (wall now() vs sim stamps
    // elsewhere); it only exists for back-compat with pre-Option-2 plugins.
    rclcpp::Time lidar_stamp;
    if (header.sim_capture_ns > 0) {
        lidar_stamp = rclcpp::Time(header.sim_capture_ns, RCL_ROS_TIME);
        // Sim-time rewind (IFSSIM restart / level reload) — same detection
        // as onSensorFrame(), applied to this stream's own clamp because it
        // lives on the LiDAR publish thread. Without this the clamp below
        // would pin every scan to old_time + 1 ns after a sim restart.
        if (last_lidar_stamp_.nanoseconds() > 0 &&
            lidar_stamp.nanoseconds() + kSimTimeRewindThresholdNs
                < last_lidar_stamp_.nanoseconds())
        {
            RCLCPP_WARN(node_->get_logger(),
                "LiDAR sim time rewound %.1f s → %.1f s — re-basing stamp guard",
                last_lidar_stamp_.seconds(), lidar_stamp.seconds());
            last_lidar_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
    } else {
        lidar_stamp = node_->now();
        if (header.lag_ns > 0) {
            lidar_stamp = lidar_stamp - rclcpp::Duration::from_nanoseconds(header.lag_ns);
        }
    }
    // Monotonic guard — same rationale as the IMU clamp in onSensorFrame.
    // GLIM expects strictly increasing timestamps on /lidar/Lidar1.
    if (last_lidar_stamp_.nanoseconds() > 0 && lidar_stamp <= last_lidar_stamp_) {
        lidar_stamp = last_lidar_stamp_ + rclcpp::Duration::from_nanoseconds(1);
    }
    last_lidar_stamp_ = lidar_stamp;

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = lidar_stamp;
    msg.header.frame_id = "fsds/Lidar";
    msg.height = 1;
    msg.width = total_points;
    msg.is_dense = true;
    msg.is_bigendian = false;

    // XYZ + intensity PointCloud2 (#255). Per-point payload is 16 B —
    // (x, y, z, intensity) FLOAT32. Intensity ∈ [0, 1] is computed by
    // the GPU decode shader / CPU LineTrace path:
    //   intensity = ρ_905 × cos(θ_inc) × (R_ref / range)²
    // (Hesai ATX-S01 working principle, see FSDSLidarDecode.usf and
    // FSDSLidarSensor.cpp::PerformScan.)
    //
    // Pre-#255 was XYZ-only (12 B/point); the timestamp field that lived
    // here even earlier was for fast_LIMO's HESAI handler, which has
    // since been replaced upstream — current consumers read x/y/z and
    // (now) intensity.
    //
    // Per-point construction with PointCloud2Iterator was the dominant
    // cost on this thread at the datasheet pts/s rate; the source data
    // is already a packed (x,y,z,intensity) float32 array, so the full
    // point payload is a single memcpy of total_points × 16 bytes.
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(4,
        "x",         1, sensor_msgs::msg::PointField::FLOAT32,
        "y",         1, sensor_msgs::msg::PointField::FLOAT32,
        "z",         1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(total_points);

    // NOTE: points are passed through verbatim — the downstream pipeline
    // (cone detection / SLAM) was written against UE's left-handed axis
    // convention, so "correcting" to REP-103 by negating Y here breaks
    // cone clustering. Leave as-is for compatibility.
    std::memcpy(msg.data.data(), points,
                static_cast<size_t>(total_points) * 4 * sizeof(float));

    lidar_pub_->publish(msg);

    // Optional /lidar/Lidar1/viz — every Nth point as a separate cloud
    // for browser-based visualisers (Foxglove web, Lichtblick web) that
    // burn 30-40 % CPU deserialising the full 1.5 MB/scan stream.
    // Off by default (lidar_viz_decimation_ == 0); when enabled, the
    // autonomy stack still gets the full /lidar/Lidar1 cloud, only
    // viz tools subscribe to /viz. Header (stamp, frame_id) is
    // identical so the two clouds line up frame-for-frame.
    if (lidar_viz_pub_ && lidar_viz_decimation_ >= 2) {
        const uint32_t N = lidar_viz_decimation_;
        const int viz_count = (total_points + static_cast<int>(N) - 1) / static_cast<int>(N);
        if (viz_count > 0) {
            sensor_msgs::msg::PointCloud2 viz;
            viz.header        = msg.header;
            viz.height        = 1;
            viz.width         = static_cast<uint32_t>(viz_count);
            viz.is_dense      = true;
            viz.is_bigendian  = false;

            sensor_msgs::PointCloud2Modifier mod(viz);
            mod.setPointCloud2Fields(4,
                "x",         1, sensor_msgs::msg::PointField::FLOAT32,
                "y",         1, sensor_msgs::msg::PointField::FLOAT32,
                "z",         1, sensor_msgs::msg::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
            mod.resize(viz_count);

            // Stride-copy: every Nth (x,y,z,intensity) quartet. Kept
            // simple and deterministic — drop random / voxel-grid
            // sampling are nice-to-haves but stride is enough to make
            // a browser tab usable and adds no per-frame allocation
            // beyond the cloud itself.
            float* dst = reinterpret_cast<float*>(viz.data.data());
            for (int i = 0, src_idx = 0; i < viz_count; ++i, src_idx += static_cast<int>(N) * 4) {
                dst[i*4 + 0] = points[src_idx + 0];
                dst[i*4 + 1] = points[src_idx + 1];
                dst[i*4 + 2] = points[src_idx + 2];
                dst[i*4 + 3] = points[src_idx + 3];
            }

            lidar_viz_pub_->publish(viz);
        }
    }
}

// =============================================================================
// TCP timer callbacks (low frequency)
// =============================================================================

void IFSSIMRosWrapper::goSignalTimerCb()
{
    fs_msgs::msg::GoSignal msg;
    msg.header.stamp = node_->now();
    msg.mission = mission_name_;
    msg.track = track_name_;
    go_signal_pub_->publish(msg);
}

void IFSSIMRosWrapper::lwsPublishTimerCb()
{
    // 100 Hz LWS publish — runs the cached commanded δ_road through
    // the Bosch LWS sensor model and emits two views of the same
    // measurement:
    //   * /lws/steering_wheel_angle_rad — sensor-faithful column angle
    //     (what uDV would forward off CAN, in rad rather than the raw
    //     LWS bit layout).
    //   * /steering_angle — road-wheel angle (= LWS / steering_ratio),
    //     consumer contract for OdometryFilter and any future EKF.
    //
    // Both publish from the SAME measure() call so they stay
    // bit-consistent — a downstream EKF that uses both can't get them
    // out of sync. See #462.
    if (!lws_sensor_) return;
    const double cmd_road_rad =
        latest_steering_cmd_rad_.load(std::memory_order_relaxed);
    const double cmd_sw_rad = cmd_road_rad * steering_ratio_;
    const double meas_sw_rad = lws_sensor_->measure(cmd_sw_rad);
    {
        std_msgs::msg::Float32 m;
        m.data = static_cast<float>(meas_sw_rad);
        lws_steering_wheel_pub_->publish(m);
    }
    {
        std_msgs::msg::Float32 m;
        m.data = static_cast<float>(meas_sw_rad / steering_ratio_);
        steering_angle_pub_->publish(m);
    }
}

void IFSSIMRosWrapper::tireLoadsTimerCb()
{
    // Pulls Chaos's per-wheel SpringForce via the plugin RPC and publishes
    // it as a 4-element Float32MultiArray on /tire_loads. Layout label is
    // "wheels" with the documented order [FL, FR, RL, RR]; clients should
    // index by name not by position, but they should know the order from
    // the dim label too.
    if (!client_ || !client_->isConnected()) return;
    std::string resp = client_->sendCommand("getTireLoads truth");
    if (resp.empty() || resp.find("\"error\"") != std::string::npos) return;

    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "wheels";  // FL, FR, RL, RR
    msg.layout.dim[0].size = 4;
    msg.layout.dim[0].stride = 4;
    msg.layout.data_offset = 0;
    msg.data.resize(4);
    msg.data[0] = static_cast<float>(client_->parseDouble(resp, "FL"));
    msg.data[1] = static_cast<float>(client_->parseDouble(resp, "FR"));
    msg.data[2] = static_cast<float>(client_->parseDouble(resp, "RL"));
    msg.data[3] = static_cast<float>(client_->parseDouble(resp, "RR"));
    tire_loads_pub_->publish(msg);
}

void IFSSIMRosWrapper::extraInfoTimerCb()
{
    if (!client_ || !client_->isConnected()) return;
    std::string resp = client_->sendCommand("getRefereeState");
    if (resp.empty()) return;

    fs_msgs::msg::ExtraInfo msg;
    msg.doo_counter = (uint32_t)client_->parseDouble(resp, "doo_counter");
    msg.laps = (uint32_t)client_->parseDouble(resp, "laps");
    extra_info_pub_->publish(msg);

    // Detect finished edge (false -> true) and notify downstream.
    // Robust JSON key-level parse — previously a raw substring search for
    // `"finished":true`, which was fragile to key reordering and to any
    // other field whose string value happened to contain that literal.
    bool finished_now = client_->parseBool(resp, "finished");
    if (finished_now && !last_finished_state_ && finished_signal_pub_) {
        fs_msgs::msg::FinishedSignal fin;
        fin.header.stamp = node_->now();
        finished_signal_pub_->publish(fin);
        RCLCPP_INFO(node_->get_logger(), "Event finished — published /signal/finished");
    }

    // Detect session restart: finished true → false, OR the lap counter
    // went backwards (referee was reset by a fresh setEvent call mid-
    // session, before the previous one ever flipped finished=true).
    // Either case means the next setCarControls on the wire belongs to a
    // *new* run, so any latched ebs_triggered_ from the previous run is
    // stale and must be cleared. This is the bridge-side counterpart to
    // the control node's publish-on-init of /signal/ebs_reset, which
    // races DDS discovery after pipeline restarts; with both in place
    // the latch will reliably clear regardless of which signal lands
    // first.
    uint32_t laps_now = (uint32_t)client_->parseDouble(resp, "laps");
    const bool finished_edge = !finished_now && last_finished_state_;
    const bool lap_rewind = laps_now < last_laps_state_;
    if ((finished_edge || lap_rewind) && ebs_triggered_) {
        ebs_triggered_ = false;
        RCLCPP_INFO(node_->get_logger(),
            "Session restart detected (%s) — EBS latch cleared",
            finished_edge ? "finished true→false" : "lap counter rewound");
    }

    last_finished_state_ = finished_now;
    last_laps_state_ = laps_now;
}

void IFSSIMRosWrapper::trackPublishCb()
{
    if (!client_ || !client_->isConnected()) return;
    std::string resp = client_->sendCommand("getRefereeState");
    if (resp.empty()) return;

    fs_msgs::msg::Track msg;
    size_t arr_start = resp.find("\"cone_positions\":[");
    if (arr_start == std::string::npos) return;
    arr_start = resp.find('[', arr_start);
    size_t arr_end = resp.find(']', arr_start);
    if (arr_end == std::string::npos) return;

    std::string arr = resp.substr(arr_start + 1, arr_end - arr_start - 1);
    if (arr.empty()) return;

    size_t pos = 0;
    while (pos < arr.size()) {
        size_t obj_start = arr.find('{', pos);
        if (obj_start == std::string::npos) break;
        size_t obj_end = arr.find('}', obj_start);
        if (obj_end == std::string::npos) break;
        std::string obj = arr.substr(obj_start, obj_end - obj_start + 1);
        pos = obj_end + 1;

        auto pf = [&obj](const std::string& key) -> double {
            size_t kpos = obj.find("\"" + key + "\":");
            if (kpos == std::string::npos) return 0.0;
            return std::stod(obj.substr(kpos + key.size() + 3));
        };

        fs_msgs::msg::Cone cone;
        cone.location.x = pf("x");
        cone.location.y = pf("y");
        cone.color = (uint8_t)pf("color");
        msg.track.push_back(cone);
    }

    if (!msg.track.empty()) track_pub_->publish(msg);
}

void IFSSIMRosWrapper::staticTfCb()
{
    auto now = node_->now();

    // Sensor static transforms — base_link → fsds/{IMU,Lidar,GPS}.
    //
    // GLIM (LiDAR-IMU SLAM) owns the odom→base_link dynamic transform; the
    // bridge owns the static base_link→sensor chain. GLIM uses these to
    // compute T_lidar_imu for scan undistortion and to express its output
    // in the body frame.
    //
    // Identity transforms — UE5 already pre-transforms LiDAR points and IMU
    // readings into the vehicle frame before sending them to the bridge.
    // Applying the settings.json offsets (Lidar1.X/Y/Z = 0.5/0/0.9) here
    // would double-apply them and place sensor data at the wrong location.
    //
    // TODO real-car: when the actual IFS-08 sends LiDAR points in the
    // sensor's own frame, replace these with the real CAD offsets, source
    // from getSensorOffset RPC (cached at initializeConnection).
    auto publishIdentityStatic = [&](const std::string& parent, const std::string& child) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = parent;
        tf.child_frame_id = child;
        tf.transform.rotation.w = 1.0;  // identity (translation defaults to zero)
        static_tf_broadcaster_->sendTransform(tf);
    };

    publishIdentityStatic("base_link", "fsds/IMU");
    publishIdentityStatic("base_link", "fsds/Lidar");
    publishIdentityStatic("base_link", "fsds/GPS");

    // Camera static TFs and the camera sensors they referred to were
    // both removed in perf/strip-cameras — see header comment.
}

void IFSSIMRosWrapper::parseNoiseSettings(const std::string& settings)
{
    auto pf = [&settings](const std::string& key) -> double {
        size_t pos = settings.find("\"" + key + "\"");
        if (pos == std::string::npos) return 0.0;
        pos = settings.find(':', pos);
        if (pos == std::string::npos) return 0.0;
        pos++;
        while (pos < settings.size() && settings[pos] == ' ') pos++;
        try { return std::stod(settings.substr(pos)); } catch (...) { return 0.0; }
    };

    // settings.json values are in SI (m, m/s, m/s², rad/s). The accel
    // conversion that used to divide by 100 here was compensating for an
    // older cm/s² convention inside the plugin — settings are now SI on
    // both sides (the plugin ×100 bumps them to its internal cm/s² accel
    // signal; the IMU RPC still emits m/s² via UEVelocityToENU).
    gps_position_noise_std_ = pf("GpsPositionNoiseStd");
    imu_accel_noise_std_ = pf("AccelNoiseStd");
    imu_gyro_noise_std_ = pf("GyroNoiseStd");
    gss_velocity_noise_std_ = pf("VelocityNoiseStd");

    RCLCPP_INFO(node_->get_logger(), "Noise: GPS=%.3fm, IMU accel=%.4f gyro=%.4f, GSS=%.3f",
        gps_position_noise_std_, imu_accel_noise_std_, imu_gyro_noise_std_, gss_velocity_noise_std_);
}

// =============================================================================
// Subscriber callbacks
// =============================================================================

void IFSSIMRosWrapper::controlCommandCb(const fs_msgs::msg::ControlCommand::SharedPtr msg)
{
    if (!client_ || !client_->isConnected()) return;
    // Drop any commands after EBS has latched — real-car EBS can't be
    // overridden by the AS until it's released, and the sim's analog is
    // api_control disabled. Keeping the if-check here as well means a late
    // publisher won't slip a post-EBS setCarControls through the client
    // before the disableApiControl call has propagated.
    if (ebs_triggered_) return;
    // setVehicleCommand is the renamed-for-clarity successor of
    // setCarControls — same wire format (throttle steering regen). The
    // 3rd arg has always semantically been regen demand on the IFS-08
    // (folded into EMRAX motor command, no hydraulic friction brake);
    // the new name reflects that. ControlCommand.brake → regen.
    std::ostringstream cmd;
    cmd << "setVehicleCommand " << msg->throttle << " " << msg->steering << " " << msg->brake;
    client_->sendCommand(cmd.str());
}

void IFSSIMRosWrapper::ebsRequestCb(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    if (ebs_triggered_ || !client_ || !client_->isConnected()) return;
    ebs_triggered_ = true;
    // Route EBS through the dedicated handbrake channel. The older
    // `setCarControls 0 0 1 + disableApiControl` sequence was silently
    // undone every tick by UE5's axis-input system (keyboard brake axis
    // reads 0 → overwrites CurrentControls.Brake), delivering only
    // drag decel (~2 m/s²) instead of full brake (~11 m/s²). ActivateEbs
    // locks all input channels and clamps handbrake=true.
    client_->sendCommand("activateEbs");
    RCLCPP_INFO(node_->get_logger(), "EBS engaged — handbrake latched, all inputs locked");
}

void IFSSIMRosWrapper::ebsResetCb(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    if (!ebs_triggered_) return;  // already cleared, nothing to do
    ebs_triggered_ = false;
    RCLCPP_INFO(node_->get_logger(), "EBS latch released — control commands re-enabled");
}

void IFSSIMRosWrapper::resetSrvCb(
    const std::shared_ptr<fs_msgs::srv::Reset::Request> request,
    std::shared_ptr<fs_msgs::srv::Reset::Response> response)
{
    (void)request;
    // Command client can silently die (send failure sets connected_=false) without
    // triggering triggerReconnect, which is only fired by stream disconnects. Attempt
    // one inline reconnect so /reset works even after a crash-induced command client drop.
    if (!client_ || !client_->isConnected()) {
        RCLCPP_WARN(node_->get_logger(), "/reset: command client down, attempting reconnect");
        client_ = std::make_unique<TcpClient>();
        if (!client_->connect(host_, port_, 3.0)) {
            RCLCPP_ERROR(node_->get_logger(), "/reset: reconnect failed");
            response->success = false;
            return;
        }
        client_->sendBool("enableApiControl");
        // Re-capture home pose since we reconnected (sim may have restarted)
        std::string pose_resp = client_->sendCommand("simGetVehiclePose");
        if (!pose_resp.empty() && pose_resp.find("\"error\"") == std::string::npos) {
            home_pose_.x = client_->parseDouble(pose_resp, "x");
            home_pose_.y = client_->parseDouble(pose_resp, "y");
            home_pose_.z = client_->parseDouble(pose_resp, "z");
            home_pose_.valid = true;
        }
    }
    if (!home_pose_.valid) {
        RCLCPP_WARN(node_->get_logger(), "/reset called but home pose was never captured");
        response->success = false;
        return;
    }
    // Latch handbrake before teleporting so the car can't roll on arrival.
    client_->sendCommand("activateEbs");
    // Prefer the start gate captured by the last loadTrack (always the right
    // reset target after a track change), including its track-aligned heading.
    // Fall back to the startup spawn pose (position only) if no track is loaded.
    double rx = home_pose_.x, ry = home_pose_.y, rz = home_pose_.z;
    bool has_gate_rot = false;
    double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
    std::string gate = client_->sendCommand("getStartGatePose");
    if (!gate.empty() && gate.find("\"error\"") == std::string::npos) {
        rx = client_->parseDouble(gate, "x");
        ry = client_->parseDouble(gate, "y");
        rz = client_->parseDouble(gate, "z");
        if (gate.find("\"qw\"") != std::string::npos) {
            qw = client_->parseDouble(gate, "qw");
            qx = client_->parseDouble(gate, "qx");
            qy = client_->parseDouble(gate, "qy");
            qz = client_->parseDouble(gate, "qz");
            has_gate_rot = true;
        }
    }
    char cmd[192];
    if (has_gate_rot) {
        // Pass the track-aligned heading captured at loadTrack so /reset always
        // ends up facing down the track (orange cones in front), not whatever
        // direction the car happened to be facing when reset was called.
        snprintf(cmd, sizeof(cmd),
                 "simSetVehiclePose %.4f %.4f %.4f %.6f %.6f %.6f %.6f",
                 rx, ry, rz, qw, qx, qy, qz);
    } else {
        snprintf(cmd, sizeof(cmd),
                 "simSetVehiclePose %.4f %.4f %.4f", rx, ry, rz);
    }
    client_->sendCommand(cmd);
    response->success = true;
}
