// src/udp_raw_bridge/src/udp_raw_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <array>

using boost::asio::ip::udp;

class UdpRawNode : public rclcpp::Node {
public:
  UdpRawNode() : Node("udp_raw_node"), io_(), sock_(io_) {
    int listen_port = this->declare_parameter<int>("listen_port", 7802);
    std::string topic = this->declare_parameter<std::string>("topic_name", "env_info_raw");

    udp::endpoint ep(udp::v4(), listen_port);
    sock_.open(udp::v4());
    sock_.set_option(boost::asio::socket_base::reuse_address(true));
    sock_.bind(ep);

    // 커널 수신 버퍼 크게 (여유 있게 4MB)
    boost::asio::socket_base::receive_buffer_size opt(4 * 1024 * 1024);
    sock_.set_option(opt);

    // 센서류 QoS (BestEffort, depth 10)
    auto qos = rclcpp::SensorDataQoS().keep_last(10);
    pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>(topic, qos);

    recv_thread_ = std::thread([this]{ recv_loop(); });

    RCLCPP_INFO_STREAM(this->get_logger(),
    "Listening UDP :" << listen_port << " -> topic '" << topic << "'");

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

  boost::asio::io_context io_;
  udp::socket sock_;
  std::thread recv_thread_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpRawNode>());
  rclcpp::shutdown();
  return 0;
}
