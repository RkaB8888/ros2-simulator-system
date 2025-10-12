// src/udp_tx_bridge/src/udp_tx_iot_control_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <bridge_msgs/msg/iot_control.hpp>
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
  // 레거시 프레이밍 (과거 Python bridge 기준):
  // [ "#Appliances$"(12B) ][ int32 LE len=17 ][ int32 0 ][ int32 0 ][ int32 0 ]
  // [ uint8[17] payload ][ "\r\n" ]
  static constexpr char   HEADER_RAW[]  = "#Appliances$";
  static constexpr size_t HEADER_LEN    = 12;   // 패딩 없음
  static constexpr uint32_t LEN_BYTES   = 17;   // payload 크기
  static constexpr uint8_t TAIL_CRLF[2] = {0x0D, 0x0A};
} // namespace proto

class UdpTxIotControlNode : public rclcpp::Node {
public:
  UdpTxIotControlNode() 
  : Node("udp_tx_iot_control",
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

    // 2) 실행 중 파라미터 변경 차단 — 안전을 위해 비활성화
    param_cb_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>&) {
        (void)this;
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = false;
        res.reason = "runtime parameter changes disabled";
        return res;
      });
    
    // 3) UDP 소켓/목적지 준비 (검증 통과 후)
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
    sub_ = create_subscription<bridge_msgs::msg::IotControl>(
      "/iot_control", qos,
      std::bind(&UdpTxIotControlNode::onCmd, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "udp_tx_iot_control → %s:%d (framing: '#Appliances$'(12) + len(4=17) + aux(12) + payload(17) + CRLF)",
      remote_ip_.c_str(), remote_port_);
  }

  ~UdpTxIotControlNode() override {
    if (sockfd_ >= 0) ::close(sockfd_);
  }

private:
  static inline void append_bytes(std::vector<uint8_t>& out, const void* src, size_t n) {
    const auto* p = static_cast<const uint8_t*>(src);
    out.insert(out.end(), p, p + n);
  }

  void onCmd(const bridge_msgs::msg::IotControl & m) {
    // payload: 정확히 17바이트가 와야 함
    if (m.data.size() != proto::LEN_BYTES) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "IotControl payload size invalid: got=%zu expected=%u",
        m.data.size(), proto::LEN_BYTES);
      return;
    }

    std::vector<uint8_t> buf;
    buf.reserve(proto::HEADER_LEN + 4 + 12 + proto::LEN_BYTES + 2);

    // 1) 헤더(12B, 패딩 없음)
    append_bytes(buf, proto::HEADER_RAW, proto::HEADER_LEN);

    // 2) 길이(int32 LE = 17)
    const uint32_t len_le = proto::LEN_BYTES;
    append_bytes(buf, &len_le, sizeof(len_le));

    // 3) aux 3개 (int32 LE 0,0,0)
    const uint32_t zero = 0;
    append_bytes(buf, &zero, 4);
    append_bytes(buf, &zero, 4);
    append_bytes(buf, &zero, 4);

    // 4) payload (uint8[17])
    append_bytes(buf, m.data.data(), proto::LEN_BYTES);

    // 5) CRLF
    append_bytes(buf, proto::TAIL_CRLF, sizeof(proto::TAIL_CRLF));

    // 송신
    const ssize_t sent = ::sendto(
      sockfd_, buf.data(), buf.size(), 0,
      reinterpret_cast<sockaddr*>(&dst_), sizeof(dst_));
    if (sent != static_cast<ssize_t>(buf.size())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "sendto() partial/failed: sent=%zd expected=%zu", sent, buf.size());
    }
  }

  // params
  std::string remote_ip_;
  int         remote_port_;

  // UDP
  int         sockfd_{-1};
  sockaddr_in dst_{};

  // subs & callbacks
  rclcpp::Subscription<bridge_msgs::msg::IotControl>::SharedPtr sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpTxIotControlNode>());
  rclcpp::shutdown();
  return 0;
}
