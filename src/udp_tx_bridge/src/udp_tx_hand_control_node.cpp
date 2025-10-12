#include <rclcpp/rclcpp.hpp>
#include <bridge_msgs/msg/hand_control_command.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstring>
#include <string>

namespace proto {
  // "#hand_control$" = 14 bytes → 16바이트로 패딩
  static constexpr char HEADER_RAW[] = "#hand_control$";
  static constexpr size_t HEADER_FIXED_LEN = 16;
  static constexpr size_t PAYLOAD_LEN = 1 /*mode*/ + 4 /*distance*/ + 4 /*height*/; // 9B
}

class UdpTxHandControlNode : public rclcpp::Node {
public:
  UdpTxHandControlNode() : Node("udp_tx_hand_control") {
    remote_ip_ = declare_parameter<std::string>("remote_ip",   "172.23.0.1"); // WSL→Windows
    remote_port_ = declare_parameter<int>("remote_port", 7901);  // 필요 시 변경

    sockfd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
      RCLCPP_FATAL(get_logger(), "socket() failed");
      rclcpp::shutdown();
      return;
    }
    std::memset(&dst_, 0, sizeof(dst_));
    dst_.sin_family = AF_INET;
    dst_.sin_port = htons(static_cast<uint16_t>(remote_port_));
    if (::inet_pton(AF_INET, remote_ip_.c_str(), &dst_.sin_addr) != 1) {
      RCLCPP_FATAL(get_logger(), "inet_pton() failed for ip=%s", remote_ip_.c_str());
      rclcpp::shutdown();
      return;
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    sub_ = create_subscription<bridge_msgs::msg::HandControlCommand>(
      "/hand_control", qos,
      std::bind(&UdpTxHandControlNode::onCmd, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "udp_tx_hand_control → %s:%d (header=16B, payload=9B)",
      remote_ip_.c_str(), remote_port_);
  }

  ~UdpTxHandControlNode() override {
    if (sockfd_ >= 0) ::close(sockfd_);
  }

private:
  void onCmd(const bridge_msgs::msg::HandControlCommand & m) {
    // [16B header][1B mode][4B distance][4B height]
    std::array<uint8_t, proto::HEADER_FIXED_LEN + proto::PAYLOAD_LEN> buf{};
    // header(16B, zero-pad)
    const size_t src_len = std::strlen(proto::HEADER_RAW);
    const size_t copy_len = std::min(src_len, proto::HEADER_FIXED_LEN);
    std::memcpy(buf.data(), proto::HEADER_RAW, copy_len);

    // payload (LE)
    buf[proto::HEADER_FIXED_LEN + 0] = static_cast<uint8_t>(m.mode);
    std::memcpy(buf.data() + proto::HEADER_FIXED_LEN + 1, &m.distance, sizeof(float));
    std::memcpy(buf.data() + proto::HEADER_FIXED_LEN + 5, &m.height,   sizeof(float));

    const ssize_t sent = ::sendto(
      sockfd_, buf.data(), buf.size(), 0,
      reinterpret_cast<sockaddr*>(&dst_), sizeof(dst_));
    if (sent != static_cast<ssize_t>(buf.size())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "sendto() partial/failed: sent=%zd expected=%zu", sent, buf.size());
    }
  }

  std::string remote_ip_;
  int remote_port_{7901};
  int sockfd_{-1};
  sockaddr_in dst_{};
  rclcpp::Subscription<bridge_msgs::msg::HandControlCommand>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpTxHandControlNode>());
  rclcpp::shutdown();
  return 0;
}
