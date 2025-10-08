// src/udp_parsers_cpp/src/custom_object_parser.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <bridge_msgs/msg/custom_object_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cstring>

using std::placeholders::_1;

static inline float rd_f32_le(const uint8_t *p) {
  float v; std::memcpy(&v, p, sizeof(float)); return v;
}

class CustomObjectParser : public rclcpp::Node {
public:
  CustomObjectParser() : Node("custom_object_parser") {
    sub_ = create_subscription<std_msgs::msg::ByteMultiArray>(
      "/object_info_raw", rclcpp::SensorDataQoS(),
      std::bind(&CustomObjectParser::onRaw, this, _1));

    pub_ = create_publisher<bridge_msgs::msg::CustomObjectInfo>("/custom_object_info", 10);
  }

private:
  void onRaw(const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
    const auto &buf = msg->data;
    // header(18) + len(4) + pad(12) + payload
    if (buf.size() < 34 + 12) return;

    // 1) 헤더 확인
    const std::string header(reinterpret_cast<const char*>(&buf[0]), 18);
    if (header != "#hand_control_pub$") return;

    // 2) 길이 확인 (LE 가정)
    int32_t data_len = 0;
    std::memcpy(&data_len, &buf[18], 4);

    // 기대 페이로드 길이 = 20 * 12 = 240
    const size_t data_len_sz = static_cast<size_t>(data_len);
    if (data_len != 12*20 || buf.size() < 34 + data_len_sz) return;
    const size_t payload = 34;

    bridge_msgs::msg::CustomObjectInfo info;

    const size_t object_size = 12;
    const size_t max_objects = data_len_sz / object_size;
    info.positions.reserve(max_objects);

    for (size_t i = 0; i < max_objects; ++i) {
      const uint8_t* base = &buf[payload + i * object_size];
      float x = rd_f32_le(base + 0);
      float y = rd_f32_le(base + 4);
      float z = rd_f32_le(base + 8);
      if (x == 0.0f && y == 0.0f && z == 0.0f)
        continue;  // 빈 데이터는 무시

      geometry_msgs::msg::Point pt;
      pt.x = x; pt.y = y; pt.z = z;
      info.positions.push_back(pt);
    }

    pub_->publish(info);
  }

  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr sub_;
  rclcpp::Publisher<bridge_msgs::msg::CustomObjectInfo>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomObjectParser>());
  rclcpp::shutdown();
  return 0;
}
