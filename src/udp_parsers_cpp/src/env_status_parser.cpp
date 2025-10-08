// src/udp_parsers_cpp/src/env_status_parser.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <bridge_msgs/msg/enviroment_status.hpp>  // 기존 msg 정의 사용

using std::placeholders::_1;

class EnvStatusParser : public rclcpp::Node {
public:
  EnvStatusParser() : Node("env_status_parser") {
    sub_ = create_subscription<std_msgs::msg::ByteMultiArray>(
      "/env_info_raw", rclcpp::SensorDataQoS(),
      std::bind(&EnvStatusParser::onRaw, this, _1));

    pub_ = create_publisher<bridge_msgs::msg::EnviromentStatus>("/env_status", 10);
  }

private:
  void onRaw(const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
    const auto &buf = msg->data;
    
    // 패킷 형식: header(12) + len(4) + pad(12) + payload(6)
    if (buf.size() < 28 + 6) return;

    // 1) 헤더 확인
    const std::string header(reinterpret_cast<const char*>(&buf[0]), 12);
    if (header != "#Enviroment$") return;

    // 2) 길이 확인 (LE 가정)
    int32_t data_len = 0;
    std::memcpy(&data_len, &buf[12], 4);
    if (data_len != 6) return;

    // 3) payload 파싱 (offset=28)
    const size_t p = 28;
    bridge_msgs::msg::EnviromentStatus env;
    env.weather     = static_cast<int8_t>(buf[p+0]);
    env.temperature = static_cast<int8_t>(buf[p+1]);
    env.month       = static_cast<uint8_t>(buf[p+2]);
    env.day         = static_cast<uint8_t>(buf[p+3]);
    env.hour        = static_cast<uint8_t>(buf[p+4]);
    env.minute      = static_cast<uint8_t>(buf[p+5]);

    pub_->publish(env);
  }

  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr sub_;
  rclcpp::Publisher<bridge_msgs::msg::EnviromentStatus>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnvStatusParser>());
  rclcpp::shutdown();
  return 0;
}
