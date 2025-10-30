// src/udp_parsers_cpp/src/lidar_parser.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cstring>
#include <cmath>
#include <vector>

using std::placeholders::_1;

class LidarParser : public rclcpp::Node {
public:
  LidarParser() : rclcpp::Node("lidar_parser") {
    sub_ = create_subscription<std_msgs::msg::ByteMultiArray>(
      "/lidar_raw", rclcpp::SensorDataQoS(),
      std::bind(&LidarParser::onRaw, this, _1));

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan_raw", 10);
    
    last_stamp_ = this->now();

    RCLCPP_INFO(get_logger(), "lidar_parser started");
  }

private:
  void onRaw(const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
    const auto &buf = msg->data;

    // 최소 길이 체크 (헤더 25바이트 + 데이터 2바이트 이상)
    if (buf.size() < 27) return;

    // payload 길이(int32, 오프셋 9)
    int32_t data_len = 0;
    std::memcpy(&data_len, &buf[9], 4);
    if (data_len <= 0) return;
    if (buf.size() < 25 + static_cast<size_t>(data_len)) return;

    const uint8_t* payload = &buf[25];

    // 레거시 3바이트/포인트(거리2 + 강도1) 가정
    const int num_points = data_len / 3;
    if (num_points <= 0) return;

    // 시간 정보
    const auto now = this->now();
    const double scan_time = (now - last_stamp_).seconds();
    last_stamp_ = now;

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = this->now();
    // scan.header.stamp = now;
    scan.header.frame_id = "velodyne";

    // 각도: 레거시 기본(0 시작, 360 포인트, inc=π/180)과 호환되되, N에 자동 적응
    // N=360 이면 angle_increment = 2π/360 = π/180 (레거시와 동일)
    scan.angle_min = 0.0;
    const double angle_increment = (2.0 * M_PI) / static_cast<double>(num_points);
    scan.angle_increment = angle_increment;
    scan.angle_max = scan.angle_min + angle_increment * static_cast<double>(num_points - 1);

    // 범위(레거시 기본 값 유지)
    scan.range_min = 0.0;
    scan.range_max = 10.0;

    // 타이밍(레거시: 이전 프레임과의 시간차로 계산)
    // scan.scan_time = scan_time > 0.0 ? static_cast<float>(scan_time) : 0.0f;
    // scan.time_increment = (scan_time > 0.0)
    //   ? static_cast<float>(scan_time / static_cast<double>(num_points))
    //   : 0.0f;
    scan.scan_time = 0.0f;
    scan.time_increment = 0.0f;

    // 거리(mm→m) 및 강도 파싱
    scan.ranges.resize(num_points);
    scan.intensities.resize(num_points);

    // payload 레이아웃: [d0_lo, d0_hi, i0, d1_lo, d1_hi, i1, ...]
    // 거리: little-endian 16-bit(mm), 강도: 1바이트
    for (int i = 0; i < num_points; ++i) {
      const int base = i * 3;
      const uint8_t lo = payload[base + 0];
      const uint8_t hi = payload[base + 1];
      const uint8_t intensity = payload[base + 2];

      const uint16_t dist_mm = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
      const float dist_m = static_cast<float>(dist_mm) / 1000.0f; // mm → m

      scan.ranges[i] = dist_m;
      scan.intensities[i] = static_cast<float>(intensity);
    }

    // 레거시 호환: 마지막 샘플 무효화(0.0) — 레거시에 있던 처리를 유지하려면 켜기
    if (!scan.ranges.empty()) {
        scan.ranges.back() = 0.0f;
    }

    pub_->publish(scan);
  }

  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Time last_stamp_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarParser>());
  rclcpp::shutdown();
  return 0;
}
