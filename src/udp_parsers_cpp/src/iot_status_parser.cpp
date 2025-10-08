// src/udp_parsers_cpp/src/iot_status_parser.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <bridge_msgs/msg/iot_status.hpp>

using std::placeholders::_1;

class AppStatusParser : public rclcpp::Node {
public:
  AppStatusParser() : Node("iot_status_parser") {
    sub_ = create_subscription<std_msgs::msg::ByteMultiArray>(
      "/iot_status_raw", rclcpp::SensorDataQoS(),
      std::bind(&AppStatusParser::onRaw, this, _1));

    pub_ = create_publisher<bridge_msgs::msg::IotStatus>("/iot_status", 10);
  }

private:
  void onRaw(const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
    const auto &buf = msg->data;
    
    // header(12) + len(4) + pad(12) + payload(17)
    if (buf.size() < 28 + 17) return;
    const std::string header(reinterpret_cast<const char*>(&buf[0]), 12);
    if (header != "#Appliances$") return;
    int32_t data_len = 0;
    std::memcpy(&data_len, &buf[12], 4);
    if (data_len != 17) return;
    bridge_msgs::msg::IotStatus out;
    out.data.resize(17);
    for (size_t i = 0; i < 17; ++i)
      out.data[i] = static_cast<int8_t>(buf[28 + i]);

    pub_->publish(out);
  }

  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr sub_;
  rclcpp::Publisher<bridge_msgs::msg::IotStatus>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AppStatusParser>());
  rclcpp::shutdown();
  return 0;
}
