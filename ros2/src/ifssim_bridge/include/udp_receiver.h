#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <functional>
#include <thread>
#include <atomic>

/**
 * Binary frame structs — must match UE5 FSDSUdpBroadcaster.h exactly.
 */
#pragma pack(push, 1)

struct SensorFrame
{
    uint32_t magic;
    uint32_t frame_id;
    // UE game/sim time at capture, in ns (the ~0.77×-real physics clock).
    // Bridge writes this into header.stamp and drives /clock from it.
    // Matches FFSDSSensorFrame::Timestamp.
    uint64_t timestamp;
    // Wall-clock at capture, in ns — latency/health metrics only, never
    // used for integration. Matches FFSDSSensorFrame::ExternalTimestamp.
    uint64_t external_timestamp;

    // GPS
    double latitude, longitude;
    float altitude;

    // IMU (body frame)
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float orient_x, orient_y, orient_z, orient_w;

    // GSS (body frame, m/s)
    float gss_vx, gss_vy, gss_vz;

    // Odom/Pose (ENU meters)
    float pos_x, pos_y, pos_z;
    float pose_qx, pose_qy, pose_qz, pose_qw;
    float speed, rpm;

    // Referee
    int32_t doo_counter, oc_counter, lap_count;

    // Controls echo
    float throttle, steering, brake;

    // Ground-truth body-frame velocity, clean (no GSS sensor noise) — #315.
    // Plugin populates from VehiclePawn->GetVelocity() rotated into body
    // frame (same source as pos/orient above, no sensor in the loop).
    // Bridge sources /testing_only/odom's twist.linear from these fields
    // so the GT topic is symmetric (clean GT pose AND clean GT twist);
    // /gss continues to publish the noisy GSS sensor values.
    float gt_vel_body_x, gt_vel_body_y, gt_vel_body_z; // m/s, body frame

    // Ground-truth body-frame angular velocity (rad/s). Plugin populates
    // from RootComponent->GetPhysicsAngularVelocityInRadians() rotated
    // into body frame — same math as the IMU sensor, so this is the
    // noise-free, bias-free truth that the noisy IMU gyro tracks.
    // Bridge writes these into /testing_only/odom's twist.angular,
    // closing the gap left by #315 (twist.linear only).
    float gt_ang_vel_body_x, gt_ang_vel_body_y, gt_ang_vel_body_z;
};

struct LidarChunkHeader
{
    uint32_t magic;
    uint16_t chunk_index;
    uint16_t total_chunks;
    uint32_t frame_id;
    int32_t points_in_chunk;
    int32_t total_points;
    int32_t channels;
    // Capture-to-send lag in nanoseconds — how long ago this scan was
    // physically captured, measured by the publisher at packing time
    // as (now_cycles - LidarSensor->LastTimestamp). Bridge stamps the
    // ROS message at `node_->now() - lag_ns` so the header.stamp
    // reflects the real capture moment of the scan, regardless of
    // GPU readback latency. Issue #238 — closes the gap left by #232
    // (publisher LastTimestamp was capture-time but the wire format
    // dropped it on the floor).
    int64_t lag_ns;
    // Absolute UE game/sim time of scan capture, in ns (Option 2). When
    // >0 the bridge stamps header.stamp straight from this — immune to the
    // readback/transport/backlog latency that made lag_ns drift upward over
    // a long run. Falls back to now()-lag_ns when 0 (old plugin build).
    // Matches FFSDSLidarChunkHeader::SimCaptureNs.
    int64_t sim_capture_ns;
};

#pragma pack(pop)

static constexpr uint32_t SENSOR_MAGIC = 0x49465353; // "IFSS"
static constexpr uint32_t LIDAR_MAGIC  = 0x4C494452; // "LIDR"

/**
 * UDP Receiver — listens for sensor and LiDAR broadcasts from the IFSSIM UE5 plugin.
 * Runs two listener threads (one per port).
 */
class UdpReceiver
{
public:
    using SensorCallback = std::function<void(const SensorFrame&)>;
    using LidarCallback = std::function<void(int32_t total_points, int32_t channels,
                                              int64_t lag_ns, int64_t sim_capture_ns,
                                              const std::vector<float>& points)>;

    UdpReceiver();
    ~UdpReceiver();

    void start(int sensor_port = 41452, int lidar_port = 41453);
    void stop();

    void setSensorCallback(SensorCallback cb) { sensor_cb_ = cb; }
    void setLidarCallback(LidarCallback cb) { lidar_cb_ = cb; }

    bool isRunning() const { return running_; }

private:
    void sensorListenerThread(int port);
    void lidarListenerThread(int port);

    SensorCallback sensor_cb_;
    LidarCallback lidar_cb_;

    std::thread sensor_thread_;
    std::thread lidar_thread_;
    std::atomic<bool> running_{false};

    // LiDAR chunk reassembly
    struct LidarFrame {
        uint32_t frame_id = 0;
        int32_t total_points = 0;
        int32_t channels = 0;
        int32_t total_chunks = 0;
        int32_t chunks_received = 0;
        bool delivered = false;
        // Capture-to-send lag in ns (#238) — preserved from the first
        // chunk of the frame; subsequent chunks of the same frame carry
        // (essentially) the same value. Wrapper subtracts this from
        // node_->now() at publish time to recover the capture-time stamp.
        int64_t lag_ns = 0;
        // Absolute sim capture time (ns) — preserved from the first chunk,
        // same as lag_ns. Bridge prefers this over lag_ns when >0.
        int64_t sim_capture_ns = 0;
        std::vector<float> points;
    };
    LidarFrame pending_lidar_;
};
