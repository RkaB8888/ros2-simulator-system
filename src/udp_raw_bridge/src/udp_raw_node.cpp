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

    // 센서류 QoS (BestEffort, depth 10)
    auto qos = rclcpp::SensorDataQoS().keep_last(10);
    pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>(topic, qos);

    recv_thread_ = std::thread([this]{ recv_loop(); });

    RCLCPP_INFO_STREAM(this->get_logger(),
    "Listening UDP :" << this->get_parameter("listen_port").as_int()
    << " -> topic '" << this->get_parameter("topic_name").as_string() << "'");

  }

  ~UdpRawNode() override {
    try { sock_.close(); } catch (...) {}
    if (recv_thread_.joinable()) recv_thread_.join();
  }

private:
  void recv_loop() {
    while (rclcpp::ok()) {
      std::array<uint8_t, 4096> buf{};
      udp::endpoint sender;
      boost::system::error_code ec;
      size_t n = sock_.receive_from(boost::asio::buffer(buf), sender, 0, ec);
      if (!ec && n > 0) {
        std_msgs::msg::ByteMultiArray msg;
        msg.data.assign(buf.begin(), buf.begin() + n);
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
