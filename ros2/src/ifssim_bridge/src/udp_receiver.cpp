#include "udp_receiver.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

UdpReceiver::UdpReceiver() {}

UdpReceiver::~UdpReceiver()
{
    stop();
}

void UdpReceiver::start(int sensor_port, int lidar_port)
{
    if (running_) return;
    running_ = true;

    // port=0 disables the corresponding listener thread. Used by the
    // bridge in udp-LiDAR mode where sensors stay on TCP — saves a
    // thread that would otherwise sit blocked in recv() forever.
    if (sensor_port > 0) {
        sensor_thread_ = std::thread(&UdpReceiver::sensorListenerThread,
                                     this, sensor_port);
    }
    if (lidar_port > 0) {
        lidar_thread_ = std::thread(&UdpReceiver::lidarListenerThread,
                                    this, lidar_port);
    }
}

void UdpReceiver::stop()
{
    running_ = false;
    if (sensor_thread_.joinable()) sensor_thread_.join();
    if (lidar_thread_.joinable()) lidar_thread_.join();
}

void UdpReceiver::sensorListenerThread(int port)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "IFSSIM UDP: Failed to create sensor socket" << std::endl;
        return;
    }

    // Allow reuse and set receive timeout
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Increase receive buffer for high-frequency data
    int rcvbuf = 1024 * 1024; // 1MB
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "IFSSIM UDP: Failed to bind sensor port " << port << std::endl;
        close(sock);
        return;
    }

    std::cout << "IFSSIM UDP: Listening for sensors on port " << port << std::endl;

    while (running_) {
        SensorFrame frame;
        ssize_t n = recv(sock, &frame, sizeof(frame), 0);

        if (n == sizeof(frame) && frame.magic == SENSOR_MAGIC) {
            if (sensor_cb_) {
                sensor_cb_(frame);
            }
        }
        // Timeout or wrong size — just loop
    }

    close(sock);
}

void UdpReceiver::lidarListenerThread(int port)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "IFSSIM UDP: Failed to create LiDAR socket" << std::endl;
        return;
    }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Large receive buffer for point cloud chunks. At 1.74 M pts/s and
    // PointsPerChunk=700, a single 10 Hz scan is ~250 chunks × 8 KB ≈
    // 2 MB. The plugin sends an entire scan in one tick burst, so the
    // bridge's recv loop has ~100 ms to drain before the next burst.
    // 4 MB was tight (drops observed under load); 16 MB gives ~7 scans
    // of headroom — enough to ride out a publish-thread hiccup or a
    // foxglove fan-out spike. Linux honours the request up to
    // `sysctl net.core.rmem_max` (we run inside Docker so it's the VM's
    // sysctl, not the host's — typically 4 MB on Docker Desktop, but
    // setsockopt above 4 MB silently caps rather than failing).
    int rcvbuf = 16 * 1024 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "IFSSIM UDP: Failed to bind LiDAR port " << port << std::endl;
        close(sock);
        return;
    }

    std::cout << "IFSSIM UDP: Listening for LiDAR on port " << port << std::endl;

    // Max chunk: header + 5000 points * 3 floats * 4 bytes = ~60KB
    std::vector<uint8_t> buffer(65536);

    while (running_) {
        ssize_t n = recv(sock, buffer.data(), buffer.size(), 0);

        if (n < (ssize_t)sizeof(LidarChunkHeader)) continue;

        LidarChunkHeader* header = (LidarChunkHeader*)buffer.data();
        if (header->magic != LIDAR_MAGIC) continue;

        int data_offset = sizeof(LidarChunkHeader);
        // Per-point payload is 4 floats (x, y, z, intensity) since #255.
        int expected_data = header->points_in_chunk * 4 * sizeof(float);
        if (n < data_offset + expected_data) continue;

        // New frame? Deliver whatever we'd assembled of the previous one
        // (UDP loses chunks under load — without this, a single missing
        // chunk would mean the entire scan is silently dropped) and reset
        // state for the new frame.
        //
        // Reset only on frame_id transition. The earlier `|| chunk_index
        // == 0` reset was a hazard under packet reordering: if Docker
        // Desktop's gvisor reorders chunks within a single send burst
        // (the plugin emits ~250 datagrams in a tight loop), receiving
        // chunk 0 last for a given frame would clobber the partially-
        // assembled state for that *same* frame. frame_id alone is the
        // unambiguous boundary — every chunk in one scan shares it.
        // Drop chunks belonging to a frame older than the one currently
        // being assembled — Docker Desktop's UDP forwarder can reorder
        // datagrams across the host/VM boundary, and accepting a stale
        // chunk would (a) corrupt the in-flight scan and (b) trigger the
        // transition path, which would then wrongly fire partial deliveries
        // on every back-and-forth. Frame IDs are monotonic on the plugin
        // side (FrameCounter is post-incremented per sensor tick), so a
        // strictly-less header frame_id is unambiguously stale.
        if (pending_lidar_.frame_id != 0
            && (int32_t)(header->frame_id - pending_lidar_.frame_id) < 0) {
            continue;
        }
        if (header->frame_id != pending_lidar_.frame_id) {
            if (!pending_lidar_.delivered
                && pending_lidar_.chunks_received > 0
                && lidar_cb_) {
                lidar_cb_(pending_lidar_.total_points, pending_lidar_.channels,
                          pending_lidar_.lag_ns, pending_lidar_.sim_capture_ns,
                          pending_lidar_.points);
            }
            pending_lidar_.frame_id = header->frame_id;
            pending_lidar_.total_points = header->total_points;
            pending_lidar_.channels = header->channels;
            pending_lidar_.total_chunks = header->total_chunks;
            pending_lidar_.chunks_received = 0;
            pending_lidar_.delivered = false;
            pending_lidar_.lag_ns = header->lag_ns;
            pending_lidar_.sim_capture_ns = header->sim_capture_ns;
            pending_lidar_.points.assign(header->total_points * 4, 0.0f);  // x,y,z,intensity — #255
        }
        if (pending_lidar_.delivered) {
            // Late chunk for an already-delivered frame: drop.
            continue;
        }

        // Copy chunk data into correct position. PointsPerChunk is the
        // uniform chunk stride the plugin uses (last chunk may be shorter,
        // tracked by points_in_chunk). MUST match PointsPerChunk in the
        // plugin's FSDSUdpBroadcaster.cpp::BroadcastLidarFrame — since
        // #255 both hardcode 500 pts/chunk so each 16-B/point datagram
        // fits under macOS's default `net.inet.udp.maxdgram` of 9216 B
        // (500×16 + 24 ≈ 8024 B). Pre-#255 was 700 pts/chunk @ 12 B.
        // Linux defaults are much higher; chunk size is sized for the
        // most restrictive host.
        constexpr int LIDAR_UDP_POINTS_PER_CHUNK = 500;
        int point_offset = header->chunk_index * LIDAR_UDP_POINTS_PER_CHUNK * 4;
        float* src = (float*)(buffer.data() + data_offset);
        int floats_count = header->points_in_chunk * 4;

        if (point_offset + floats_count <= (int)pending_lidar_.points.size()) {
            memcpy(pending_lidar_.points.data() + point_offset, src, floats_count * sizeof(float));
        }

        pending_lidar_.chunks_received++;

        // All chunks received? Deliver, then mark delivered so subsequent
        // late/duplicate chunks for this same frame_id don't re-fire the
        // callback (the publish thread would otherwise see a flood of
        // identical PointCloud2 messages and /lidar/Lidar1 hz would
        // explode well above the LiDAR's true rate).
        if (pending_lidar_.chunks_received >= pending_lidar_.total_chunks) {
            if (lidar_cb_) {
                lidar_cb_(pending_lidar_.total_points, pending_lidar_.channels,
                          pending_lidar_.lag_ns, pending_lidar_.sim_capture_ns,
                          pending_lidar_.points);
            }
            pending_lidar_.delivered = true;
        }
    }

    close(sock);
}
