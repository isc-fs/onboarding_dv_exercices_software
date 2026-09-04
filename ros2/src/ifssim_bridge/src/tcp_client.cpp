#include "tcp_client.h"

#ifdef _WIN32
  #define WIN32_LEAN_AND_MEAN
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "Ws2_32.lib")
  typedef SOCKET socket_t;
  #define INVALID_SOCK INVALID_SOCKET
  #define CLOSE_SOCKET closesocket
  static bool wsa_initialized = false;
  static void init_wsa() {
      if (!wsa_initialized) {
          WSADATA wsa;
          WSAStartup(MAKEWORD(2, 2), &wsa);
          wsa_initialized = true;
      }
  }
#else
  #include <sys/socket.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  typedef int socket_t;
  #define INVALID_SOCK (-1)
  #define CLOSE_SOCKET close
  static void init_wsa() {}
#endif

#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <netdb.h>

TcpClient::TcpClient() {}

TcpClient::~TcpClient()
{
    disconnect();
}

bool TcpClient::connect(const std::string& host, int port, double timeout_sec)
{
    std::lock_guard<std::mutex> lock(mutex_);
    init_wsa();

    socket_t sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == INVALID_SOCK) {
        std::cerr << "IFSSIM Bridge: Failed to create socket" << std::endl;
        return false;
    }
    socket_fd_ = (decltype(socket_fd_))sock;

    // Set timeout
#ifdef _WIN32
    DWORD timeout_ms = (DWORD)(timeout_sec * 1000);
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_ms, sizeof(timeout_ms));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout_ms, sizeof(timeout_ms));
#else
    struct timeval tv;
    tv.tv_sec = (int)timeout_sec;
    tv.tv_usec = (int)((timeout_sec - tv.tv_sec) * 1000000);
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
#endif

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);

    // Try inet_pton first (IP address), fall back to hostname resolution
    if (inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr) <= 0) {
        struct hostent* he = gethostbyname(host.c_str());
        if (!he) {
            std::cerr << "IFSSIM Bridge: Invalid address: " << host << std::endl;
            CLOSE_SOCKET(sock);
            socket_fd_ = (decltype(socket_fd_))INVALID_SOCK;
            return false;
        }
        memcpy(&server_addr.sin_addr, he->h_addr_list[0], he->h_length);
    }

    if (::connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "IFSSIM Bridge: Connection failed to " << host << ":" << port << std::endl;
        CLOSE_SOCKET(sock);
        socket_fd_ = (decltype(socket_fd_))INVALID_SOCK;
        return false;
    }

    connected_ = true;
    std::cout << "IFSSIM Bridge: Connected to " << host << ":" << port << std::endl;
    return true;
}

void TcpClient::disconnect()
{
    std::lock_guard<std::mutex> lock(mutex_);
    socket_t sock = (socket_t)socket_fd_;
    if (sock != INVALID_SOCK) {
        CLOSE_SOCKET(sock);
        socket_fd_ = (decltype(socket_fd_))INVALID_SOCK;
    }
    connected_ = false;
}

bool TcpClient::isConnected() const
{
    return connected_;
}

std::string TcpClient::sendCommand(const std::string& command)
{
    std::lock_guard<std::mutex> lock(mutex_);
    socket_t sock = (socket_t)socket_fd_;

    if (!connected_ || sock == INVALID_SOCK) return "";

    std::string msg = command + "\n";
    if (send(sock, msg.c_str(), (int)msg.size(), 0) < 0) {
        connected_ = false;
        return "";
    }

    // Read until we have a complete response (loop to handle TCP fragmentation and large payloads)
    // A complete RPC response is newline-terminated; we buffer up to
    // `kMaxResponseBytes` before giving up on the framing assumption.
    // Bumped from the original 512 KB to 2 MB because the referee's
    // getRefereeState payload grows linearly with cone count — a dense
    // autocross layout can cross 600 KB and the old cap would silently
    // truncate mid-JSON, leaving callers with malformed input.
    constexpr size_t kMaxResponseBytes = 2 * 1024 * 1024;
    std::string response;
    response.reserve(65536);
    char buffer[16384];
    bool truncated = false;
    while (true) {
        int bytes = recv(sock, buffer, sizeof(buffer) - 1, 0);
        if (bytes <= 0) {
            connected_ = false;
            return "";
        }
        response.append(buffer, bytes);
        if (response.back() == '\n' || response.back() == '\r') break;
        if (response.size() > kMaxResponseBytes) {
            truncated = true;
            break;
        }
    }
    if (truncated) {
        // Log once per incident so downstream callers know why parsing failed.
        // stderr so the bridge's RCLCPP_* output isn't the only place this
        // shows up — TcpClient has no logger handle.
        std::fprintf(stderr,
            "[tcp_client] response exceeded %zu bytes without a terminating newline; "
            "returning truncated data. Command was: %s\n",
            kMaxResponseBytes, command.c_str());
    }

    while (!response.empty() && (response.back() == '\n' || response.back() == '\r'))
        response.pop_back();

    return response;
}

bool TcpClient::sendBool(const std::string& command)
{
    return sendCommand(command) == "true";
}

double TcpClient::parseDouble(const std::string& json, const std::string& key)
{
    // Simple JSON value parser: find "key":value
    std::string search = "\"" + key + "\":";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return 0.0;

    pos += search.size();
    // Skip whitespace
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) pos++;

    std::string value_str;
    while (pos < json.size() && json[pos] != ',' && json[pos] != '}') {
        value_str += json[pos++];
    }

    try {
        return std::stod(value_str);
    } catch (...) {
        return 0.0;
    }
}

bool TcpClient::parseBool(const std::string& json, const std::string& key)
{
    std::string search = "\"" + key + "\":";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return false;

    pos += search.size();
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) pos++;

    // Compare against the literal 'true'; anything else (false, null,
    // numeric, or EOF) returns false.
    return json.compare(pos, 4, "true") == 0;
}

std::string TcpClient::sendBinaryCommand(const std::string& command, std::vector<uint8_t>& outData)
{
    std::lock_guard<std::mutex> lock(mutex_);
    socket_t sock = (socket_t)socket_fd_;

    if (!connected_ || sock == INVALID_SOCK) return "";

    std::string msg = command + "\n";
    if (send(sock, msg.c_str(), (int)msg.size(), 0) < 0) {
        connected_ = false;
        return "";
    }

    // Read header line (e.g., "PTS:1234\n")
    std::string header;
    char c;
    while (recv(sock, &c, 1, 0) == 1) {
        if (c == '\n') break;
        header += c;
    }

    if (header.empty()) return "";

    // Parse the byte count from header
    size_t colonPos = header.find(':');
    if (colonPos == std::string::npos) return header;

    int dataSize = 0;
    try {
        std::string prefix = header.substr(0, colonPos);
        int count = std::stoi(header.substr(colonPos + 1));

        // PTS handler (`getLidarDataBinary` reply, 4 floats per point)
        // was removed in #322 along with the rest of the TCP-LiDAR
        // path on the plugin side — `sendBinaryCommand` is only ever
        // called for `simGetImageBinary` now.
        if (prefix == "IMG") {
            dataSize = count; // raw byte count
        }
    } catch (...) {
        return header;
    }

    // Read binary data
    if (dataSize > 0) {
        outData.resize(dataSize);
        int totalRead = 0;
        while (totalRead < dataSize) {
            int bytesRead = recv(sock, (char*)outData.data() + totalRead, dataSize - totalRead, 0);
            if (bytesRead <= 0) break;
            totalRead += bytesRead;
        }
        outData.resize(totalRead);
    }

    return header;
}
