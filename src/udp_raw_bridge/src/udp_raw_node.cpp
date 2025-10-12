// src/udp_raw_bridge/src/udp_raw_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <vector>
#include <cstring>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

using boost::asio::ip::udp;

class UdpRawNode : public rclcpp::Node {
public:
  UdpRawNode() : Node("udp_raw_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)), io_(), sock_(io_) {
    // 1) 파라미터: 기본값 없이 선언 → 없거나 비정상이면 즉시 종료(fail-fast)
    if (!this->get_parameter("listen_port", listen_port_) ||
        !this->get_parameter("topic_name",  topic_name_)   ||
        listen_port_ <= 0 || listen_port_ > 65535          ||
        topic_name_.empty()) {
      RCLCPP_FATAL(get_logger(),
        "Missing/invalid required parameters: listen_port/topic_name. "
        "Pass a valid YAML (e.g., bridge_bringup/config/system.<env>.yaml).");
      rclcpp::shutdown();
      return;
    }

    // (선택) 실행 중 파라미터 변경 차단
    param_cb_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>&) {
        (void)this;
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = false;
        res.reason = "runtime parameter changes disabled";
        return res;
      });
    
    // 2) UDP 소켓 바인딩 (검증 통과 후)
    boost::system::error_code ec;
    udp::endpoint ep(udp::v4(), static_cast<uint16_t>(listen_port_));
    sock_.open(udp::v4(), ec);
    if (ec) {
      RCLCPP_FATAL(get_logger(), "socket.open() failed: %s", ec.message().c_str());
      rclcpp::shutdown();
      return;
    }
    sock_.set_option(boost::asio::socket_base::reuse_address(true), ec);
    sock_.bind(ep, ec);
    if (ec) {
      RCLCPP_FATAL(get_logger(), "bind(%d) failed: %s", listen_port_, ec.message().c_str());
      rclcpp::shutdown();
      return;
    }

    // 커널 수신 버퍼 크게 (여유 있게 4MB)
    boost::asio::socket_base::receive_buffer_size opt(4 * 1024 * 1024);
    sock_.set_option(opt, ec);

    // 3) 퍼블리셔
    auto qos = rclcpp::SensorDataQoS().keep_last(10); // BestEffort
    pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>(topic_name_, qos);

    // 4) 수신 스레드
    recv_thread_ = std::thread([this]{ recv_loop(); });

    RCLCPP_INFO(get_logger(), "udp_rx listening on %d → topic '%s'",
                listen_port_, topic_name_.c_str());
  }

  ~UdpRawNode() override {
    try { sock_.close(); } catch (...) {}
    if (recv_thread_.joinable()) recv_thread_.join();
  }

private:
  void recv_loop() {
    // 사용자 버퍼: 최소 65536B (UDP 최대 페이로드 대응). 여유로 128KB 권장.
    static constexpr size_t kMaxDatagram = 128 * 1024; // 128KB
    std::vector<uint8_t> buf(kMaxDatagram);

    while (rclcpp::ok()) {
      udp::endpoint sender;
      boost::system::error_code ec;
      size_t n = sock_.receive_from(boost::asio::buffer(buf.data(), buf.size()), sender, 0, ec);
      // 버퍼가 작아 잘린 경우: ec == message_size 로 올 수 있음.
      if (ec) {
        if (ec == boost::asio::error::message_size) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Datagram truncated (buffer too small). Increase kMaxDatagram.");
        } else if (ec == boost::asio::error::operation_aborted) {
          // 소켓 close() 등으로 종료될 때
          break;
        } else {
          RCLCPP_WARN(this->get_logger(), "recv_from error: %s", ec.message().c_str());
        }
        continue;
      }
      if (n > 0) {
        std_msgs::msg::ByteMultiArray msg;
        msg.data.resize(n);
        std::memcpy(msg.data.data(), buf.data(), n);
        pub_->publish(msg);
      }
    }
  }

  // params (기본값 없음)
  int         listen_port_;
  std::string topic_name_;

  // UDP
  boost::asio::io_context io_;
  udp::socket             sock_;
  std::thread             recv_thread_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr pub_;

  // param change blocker
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpRawNode>());
  rclcpp::shutdown();
  return 0;
}
