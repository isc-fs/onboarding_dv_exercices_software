#pragma once

#include <string>
#include <vector>
#include <mutex>

/**
 * Simple TCP client for connecting to the IFSSIM RPC server.
 * Protocol: send "command [args]\n", receive "json_response\n"
 * Cross-platform: uses Winsock2 on Windows, POSIX sockets on Linux.
 */
class TcpClient
{
public:
    TcpClient();
    ~TcpClient();

    bool connect(const std::string& host, int port, double timeout_sec = 5.0);
    void disconnect();
    bool isConnected() const;

    /** Send a command and receive the response */
    std::string sendCommand(const std::string& command);

    /** Convenience: send and parse a simple boolean response */
    bool sendBool(const std::string& command);

    /** Convenience: send and parse a float response from JSON */
    double parseDouble(const std::string& json, const std::string& key);

    /** Parse a JSON boolean field by key. Returns true if the value is
     *  the literal `true`, false otherwise (including missing key). Robust
     *  to JSON key reordering and whitespace — unlike a naive substring
     *  match on `"key":true` which breaks the moment the plugin reorders
     *  its output fields. */
    bool parseBool(const std::string& json, const std::string& key);

    /** Send a binary command and receive header + binary data
     * Protocol: send "command\n", receive "HEADER:value\n" + raw bytes
     * Returns the header line and fills outData with binary payload */
    std::string sendBinaryCommand(const std::string& command, std::vector<uint8_t>& outData);

private:
#ifdef _WIN32
    unsigned long long socket_fd_ = ~0ULL; // INVALID_SOCKET
#else
    int socket_fd_ = -1;
#endif
    bool connected_ = false;
    std::mutex mutex_;
};
