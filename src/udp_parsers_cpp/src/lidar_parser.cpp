// src/udp_parsers_cpp/src/lidar_parser.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cstring>
#include <cmath>

using std::placeholders::_1;

class LidarParser : public rclcpp::Node {
public:
  LidarParser() : Node("lidar_parser") {
    sub_ = create_subscription<std_msgs::msg::ByteMultiArray>(
      "/lidar_raw", rclcpp::SensorDataQoS(),
      std::bind(&LidarParser::onRaw, this, _1));

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    RCLCPP_INFO(get_logger(), "lidar_parser started");
  }

private:
  void onRaw(const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
    const auto &buf = msg->data;
    if (buf.size() < 25 + 2) return;

    // 1) 길이 읽기 (int, offset=9)
    int32_t data_len = 0;
    std::memcpy(&data_len, &buf[9], 4);

    // 2) 거리 데이터 확인
    if (buf.size() < 25 + static_cast<size_t>(data_len)) return;

    // 3) 거리 배열 파싱 (short 단위)
    const uint8_t *payload = &buf[25];
    const int num_points = data_len / 2;

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = now();
    scan.header.frame_id = "velodyne";
    scan.angle_min = -M_PI / 2;
    scan.angle_max =  M_PI / 2;
    scan.range_min = 0.0;
    scan.range_max = 100.0;
    scan.angle_increment = (scan.angle_max - scan.angle_min) / num_points;

    scan.ranges.resize(num_points);
    for (int i = 0; i < num_points; ++i) {
      int16_t raw_val = 0;
      std::memcpy(&raw_val, payload + i * 2, sizeof(int16_t));
      scan.ranges[i] = static_cast<float>(raw_val) / 100.0f; // cm → m
    }

    pub_->publish(scan);
  }

  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarParser>());
  rclcpp::shutdown();
  return 0;
}
