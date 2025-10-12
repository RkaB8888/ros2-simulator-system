// src/udp_tx_bridge/src/udp_tx_cmd_vel_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include <cstdint>

namespace proto {
// 시뮬레이터가 기대하는 정확한 프레이밍 (이전 파이썬 브릿지 기준)
// [ "#Turtlebot_cmd$" (15B) ]
// [ int32 LE: payload len = 8 ]
// [ int32 LE: 0 ] x 3 (aux)
// [ float32 LE: linear, float32 LE: angular ] (총 8B)
// [ "\r\n" (0x0D 0x0A) ]

static constexpr char   HEADER_RAW[]   = "#Turtlebot_cmd$";
static constexpr size_t HEADER_LEN     = 15;      // 패딩 없음!
static constexpr uint32_t PAYLOAD_LEN  = 8;       // float32 * 2
static constexpr uint8_t TAIL_CRLF[2]  = {0x0D, 0x0A};
} // namespace proto

class UdpTxCmdVelNode : public rclcpp::Node {
public:
  UdpTxCmdVelNode() 
  : Node("udp_tx_cmd_vel", 
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    // 1) 필수 파라미터 (YAML에서만 제공)
    if (!this->get_parameter("remote_ip", remote_ip_) ||
        !this->get_parameter("remote_port", remote_port_) ||
        remote_ip_.empty() ||
        remote_port_ <= 0 || remote_port_ > 65535) {
      RCLCPP_FATAL(get_logger(),
        "Missing/invalid required parameters: remote_ip/remote_port. "
        "Pass a valid YAML config (e.g., bridge_bringup/config/system.<env>.yaml).");
      rclcpp::shutdown();
      return;
    }

    // 2) 실행 중 파라미터 변경 차단
    param_cb_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>&) {
        (void)this;
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = false;
        res.reason = "runtime parameter changes disabled";
        return res;
      });

    // 3) UDP 소켓 준비 (검증 통과 후)
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

    // 4) 구독 생성
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos,
      std::bind(&UdpTxCmdVelNode::onTwist, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "udp_tx_cmd_vel → %s:%d (framing: header(15) + len(4) + aux(12) + payload(8) + CRLF)",
      remote_ip_.c_str(), remote_port_);
  }

  ~UdpTxCmdVelNode() override {
    if (sockfd_ >= 0) { ::close(sockfd_); }
  }

private:
  static inline void append_bytes(std::vector<uint8_t>& out, const void* src, size_t n) {
    const auto* p = static_cast<const uint8_t*>(src);
    out.insert(out.end(), p, p + n);
  }

  void onTwist(const geometry_msgs::msg::Twist & msg) {
    // payload (float32 LE)
    const float linear  = static_cast<float>(msg.linear.x);
    const float angular = static_cast<float>(msg.angular.z);


    // 패킷 구성
    std::vector<uint8_t> buf;
    buf.reserve(proto::HEADER_LEN + 4 + 12 + proto::PAYLOAD_LEN + 2);

    // 1) 헤더(15B, 패딩 없음)
    append_bytes(buf, proto::HEADER_RAW, proto::HEADER_LEN);
    
    // 2) 길이 필드(int32 LE = 8)
    const uint32_t len_le = 8;         // x86_64 리틀엔디안 가정
    append_bytes(buf, &len_le, sizeof(len_le));

    // 3) aux 3개 (int32 LE 0,0,0)
    const uint32_t zero = 0;
    append_bytes(buf, &zero, sizeof(zero));
    append_bytes(buf, &zero, sizeof(zero));
    append_bytes(buf, &zero, sizeof(zero));

    // 4) payload: float32 LE (linear, angular)
    append_bytes(buf, &linear,  sizeof(float));
    append_bytes(buf, &angular, sizeof(float));

    // 5) CRLF
    append_bytes(buf, proto::TAIL_CRLF, sizeof(proto::TAIL_CRLF));

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
  int         remote_port_;

  // UDP
  int         sockfd_{-1};
  sockaddr_in dst_{};

  // sub & param-callback
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpTxCmdVelNode>());
  rclcpp::shutdown();
  return 0;
}
