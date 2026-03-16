/**
 * @file heart_beat.cpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief Supports automatic connection and communication functions without having to set the IP of each component in a local network environment
 * @version 1.1
 * @date 2025-5-19
 *
 * @copyright Copyright AU (c) 2025
 *
 */

#include <arpa/inet.h>
#include <unistd.h>
#include <ifaddrs.h>
#include <sys/socket.h>
#include <netdb.h>

#include "flat/Heartbeat_generated.h"
#include "flat/RequestConnection_generated.h"
#include "flat/ResponseConnection_generated.h"

//#include "util/HexDump.hpp"
#include "util/conversion.hpp"
#include "util/yamlParser.hpp"

#include "au_4d_radar.hpp"
#include "heart_beat.hpp"
#include "util/crc32.hpp"

#define RECEIVE_PORT 59152 // Users can use UDP ports in the range 49152-65535.
#define SEND_PORT 59153
#define BUFFER_SIZE 1460

/*
Message Format
MessageType (4 bytes) + CRC32 (4 bytes) + Payload Length (2 bytes) + Payload Body
MessageType: 4 bytes
CRC32: 4 bytes (payload_body only, not include payload_length)
payload_length: 2 bytes
payload_body: flatbuffers
*/

#define MSG_TYPE_OFFSET 4  // offset for MessageType(4bytes)
#define PAYLOAD_LEN_OFFSET 8 // offset header(4 bytes) + CRC32(4 bytes)
#define PAYLOAD_OFFSET 10  // offset header(4 bytes) + CRC32(4 bytes) + payload_length(2 bytes)


enum MessageType {
    REQUEST_CONNECTION  = 0x41551001,
    RESPONSE_CONNECTION = 0x41551002,
    HEARTBEAT_MESSAGE   = 0x41551003
};


namespace au_4d_radar
{

Heartbeat::Heartbeat(device_au_radar_node* node)
    : recv_sockfd(-1), send_sockfd(-1), running(false), radar_node_(node) {}

Heartbeat::~Heartbeat() {
    stop();
}

void Heartbeat::start() {
    if (initialize()) {
        receiverThread = std::thread(&Heartbeat::handleClientMessages, this);
    }
}

void Heartbeat::stop() {
    running = false;
    if (recv_sockfd >= 0) {
        close(recv_sockfd);
        recv_sockfd = -1;
    }

    if (send_sockfd >= 0) {
        close(send_sockfd);
        send_sockfd = -1;
    }

    if (receiverThread.joinable()) {
        receiverThread.join();
    }
}

bool Heartbeat::initialize() {
    clientHostname = YamlParser::readHostname("client_hostname");

    heartbeat_check_timer_ = radar_node_->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&Heartbeat::checkConnectionLoss, this));

    return true;
}

void Heartbeat::checkConnectionLoss() {
    const uint64_t now = static_cast<uint64_t>(std::time(nullptr));

    for (const auto& [hostname, last_ts] : last_heartbeat_ts_) {
        // RCLCPP_DEBUG(radar_node_->get_logger(), "heart_beat:: hostname: %s now: %lu time: %lu", hostname.c_str(), now, last_ts);
        if ((now > last_ts) && (now - last_ts >= 5)) {
            time_t last_time = static_cast<time_t>(last_ts);
            struct tm* timeinfo = localtime(&last_time);
            char time_str[16];
            strftime(time_str, sizeof(time_str), "%H:%M:%S", timeinfo);
            RCLCPP_DEBUG(radar_node_->get_logger(),"Connection lost: %s (last heartbeat at %s)", hostname.c_str(), time_str);
        }
    }
}

void Heartbeat::handleClientMessages() {

    while (running) {
        std::vector<uint8_t> buffer(BUFFER_SIZE);
        socklen_t len = sizeof(recv_server_addr);
        int n = recvfrom(recv_sockfd, buffer.data(), buffer.size(), MSG_DONTWAIT, (struct sockaddr *)&recv_server_addr, &len);
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(rclcpp::get_logger("Heartbeat"), "recvfrom failed");
            }
            usleep(1000);
            continue;
        } else if (n < 10 || n > BUFFER_SIZE) {
            RCLCPP_ERROR(rclcpp::get_logger("Heartbeat"), "message size exceeds buffer size");
            continue;
        }
        // MessageType (4 bytes) + CRC32 (4 bytes) + Payload Length (2 bytes) + Payload Body
        uint32_t messageType = Conversion::be_to_u32(buffer.data());
        if (messageType == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("Heartbeat"), "Invalid MessageType received");
            continue;
        }

        uint16_t payloadLength = Conversion::be_to_u16(&buffer[PAYLOAD_LEN_OFFSET]);
        if (payloadLength != n - PAYLOAD_OFFSET) {
            RCLCPP_ERROR(rclcpp::get_logger("Heartbeat"), "Payload length mismatch");
            continue;
        }

        uint32_t receivedCrc32 = Conversion::be_to_u32(&buffer[MSG_TYPE_OFFSET]);
        uint32_t calculatedCrc32 = crc32(&buffer[PAYLOAD_OFFSET], payloadLength);
        if (receivedCrc32 != calculatedCrc32) {
            RCLCPP_ERROR(rclcpp::get_logger("Heartbeat"), "CRC32 mismatch");
            continue;
        }

        std::string receivedIp = inAddrToString(recv_server_addr.sin_addr.s_addr);

        switch (messageType) {
            case MessageType::REQUEST_CONNECTION:
                processRequestConnection(buffer.data(), receivedIp, len);
                break;
            case MessageType::HEARTBEAT_MESSAGE:
                processHeartbeatMessage(buffer.data(), receivedIp);
                break;
            default:
                RCLCPP_DEBUG(radar_node_->get_logger(), "heart_beat:: Unknown message type: %08x receivedIp: %s", messageType, receivedIp.c_str());
                break;
        }
    }
}

void Heartbeat::processHeartbeatMessage(const uint8_t* buffer, const std::string& receivedIp) {
    auto Heartbeat = AU::GetHeartbeat(&buffer[PAYLOAD_OFFSET]);
    if (!Heartbeat) {
        RCLCPP_ERROR(rclcpp::get_logger("Heartbeat"), "Failed to decode Heartbeat message.");
        return;
    }

    std::string hostname = Heartbeat->client_hostname()->str();
    if (clientHostname.starts_with(hostname.substr(0, 7))) {
        mon_msgs::msg::RadarHealth radar_health_msg;
        time_t raw_time = static_cast<time_t>(Heartbeat->timestamp());
        struct tm* timeinfo = localtime(&raw_time);
        char time_str[64];
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", timeinfo);
        setClientIp(hostname, receivedIp);

        radar_health_msg.client_hostname = hostname;
        radar_health_msg.status = Heartbeat->status();
        radar_health_msg.tv_sec = Heartbeat->timestamp();
        last_heartbeat_ts_[hostname] = Heartbeat->timestamp();

        RCLCPP_DEBUG(radar_node_->get_logger(), "heart_beat:: hostname: %s status: %u time: %s",
                     radar_health_msg.client_hostname.c_str(), radar_health_msg.status, time_str);
#ifdef DEBUG_BUILD
        auto temps = Heartbeat->temp_tx_rfes();
        if (temps && temps->size() >= 4) {
            RCLCPP_DEBUG(radar_node_->get_logger(), "heartbeat(%s) temperature a53_core = %.2f, tx_rfes = %.2f, %.2f, %.2f, %.2f",
                hostname.c_str(), Heartbeat->temp_a53_cores(), temps->Get(0), temps->Get(1), temps->Get(2), temps->Get(3));
        } else {
            RCLCPP_DEBUG(radar_node_->get_logger(), "heartbeat(%s) txTemp_rfes not available or too short", hostname.c_str());
        }

        auto rfeErrorState = Heartbeat->rfe_error_state();
        if (rfeErrorState && rfeErrorState->size() >= 4) {
            RCLCPP_DEBUG(radar_node_->get_logger(), "heartbeat(%s) rfeErrorState = 0x%08x, 0x%08x, 0x%08x, 0x%08x",
                hostname.c_str(), rfeErrorState->Get(0), rfeErrorState->Get(1), rfeErrorState->Get(2), rfeErrorState->Get(3));
        } else {
            RCLCPP_DEBUG(radar_node_->get_logger(), "heartbeat(%s) rfeErrorState not available or too short", hostname.c_str());
        }
#endif
        radar_node_->publishHeartbeat(radar_health_msg);
    }
}

Heartbeat& Heartbeat::getInstance(device_au_radar_node* node) {
    static Heartbeat instance(node);
    return instance;
}

}