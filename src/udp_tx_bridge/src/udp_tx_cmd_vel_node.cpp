#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstring>
#include <string>

namespace proto {
// 고정 헤더(16B 패딩). 레거시 규격: "#Turtlebot_cmd$" + pad
static constexpr char HEADER_RAW[] = "#Turtlebot_cmd$";
static constexpr size_t HEADER_FIXED_LEN = 16;    // 헤더를 16바이트로 맞춤
static constexpr size_t PAYLOAD_LEN = 8;          // float32 v(4) + float32 w(4)
} // namespace proto

class UdpTxCmdVelNode : public rclcpp::Node {
public:
  UdpTxCmdVelNode() : Node("udp_tx_cmd_vel") {
    remote_ip_   = declare_parameter<std::string>("remote_ip",   "172.23.0.1"); // WSL→Windows 기본
    remote_port_ = declare_parameter<int>("remote_port", 7601);

    // UDP 소켓 준비
    sockfd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
      RCLCPP_FATAL(get_logger(), "socket() failed");
      rclcpp::shutdown();
      return;
    }
    std::memset(&dst_, 0, sizeof(dst_));
    dst_.sin_family = AF_INET;
    dst_.sin_port   = htons(static_cast<uint16_t>(remote_port_));
    if (::inet_pton(AF_INET, remote_ip_.c_str(), &dst_.sin_addr) != 1) {
      RCLCPP_FATAL(get_logger(), "inet_pton() failed for ip=%s", remote_ip_.c_str());
      rclcpp::shutdown();
      return;
    }

    // QoS: 최신 명령 1개만 (BestEffort)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos,
      std::bind(&UdpTxCmdVelNode::onTwist, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "udp_tx_cmd_vel started → %s:%d (header=16B, payload=8B)",
      remote_ip_.c_str(), remote_port_);
  }

  ~UdpTxCmdVelNode() override {
    if (sockfd_ >= 0) { ::close(sockfd_); }
  }

private:
  void onTwist(const geometry_msgs::msg::Twist & msg) {
    // 버퍼 구성: [16B header][8B payload]
    std::array<uint8_t, proto::HEADER_FIXED_LEN + proto::PAYLOAD_LEN> buf{};
    // 1) 헤더(16B) — 원 문자열을 복사하고, 남는 부분은 0 패딩
    {
      const size_t src_len = std::strlen(proto::HEADER_RAW);
      const size_t copy_len = std::min(src_len, proto::HEADER_FIXED_LEN);
      std::memcpy(buf.data(), proto::HEADER_RAW, copy_len);
      // 나머지는 이미 0으로 초기화되어 있음
    }
    // 2) 페이로드 — float32 LE로 v,w
    const float v = static_cast<float>(msg.linear.x);
    const float w = static_cast<float>(msg.angular.z);
    std::memcpy(buf.data() + proto::HEADER_FIXED_LEN + 0, &v, sizeof(float));
    std::memcpy(buf.data() + proto::HEADER_FIXED_LEN + 4, &w, sizeof(float));

    // 송신
    const ssize_t sent = ::sendto(
      sockfd_, buf.data(), buf.size(), 0,
      reinterpret_cast<sockaddr*>(&dst_), sizeof(dst_));
    if (sent != static_cast<ssize_t>(buf.size())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000 /*ms*/,
        "sendto() partial/failed: sent=%zd expected=%zu", sent, buf.size());
    }
  }

  // params
  std::string remote_ip_;
  int         remote_port_{7601};

  // UDP
  int sockfd_{-1};
  sockaddr_in dst_{};

  // sub
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpTxCmdVelNode>());
  rclcpp::shutdown();
  return 0;
}
