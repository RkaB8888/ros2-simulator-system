// src/udp_parsers_cpp/src/imu_parser.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cstring>

using std::placeholders::_1;

namespace {
constexpr size_t kHeaderLen        = 9;   // ex) "#IMU_DATA" + '\0' 형태가 아니어도 9바이트 헤더로 취급
constexpr size_t kLenFieldLen      = 4;   // int32 (LE)
constexpr size_t kPadLen           = 12;  // reserved
constexpr size_t kNumDoubles       = 10;  // qw,qx,qy,qz, wx,wy,wz, ax,ay,az
constexpr size_t kPayloadBytes     = kNumDoubles * sizeof(double); // 80
constexpr size_t kStart            = kHeaderLen + kLenFieldLen + kPadLen; // 25
constexpr int32_t kExpectedLen     = static_cast<int32_t>(kPayloadBytes);

// 리틀엔디언 더블 안전 읽기
inline void rd_f64_le(double &out, const uint8_t *p) { std::memcpy(&out, p, sizeof(double)); }
inline void rd_i32_le(int32_t &out, const uint8_t *p) { std::memcpy(&out, p, sizeof(int32_t)); }
} // namespace

class ImuParser : public rclcpp::Node {
public:
  ImuParser() : Node("imu_parser")
  {
    // /imu_raw는 SensorDataQoS(BEST_EFFORT)
    sub_ = create_subscription<std_msgs::msg::ByteMultiArray>(
      "/imu_raw", rclcpp::SensorDataQoS(),
      std::bind(&ImuParser::onRaw, this, _1));

    // 결과는 rqt/rviz 친화적으로 RELIABLE(depth=10)
    pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

    RCLCPP_INFO(get_logger(), "imu_parser started (fixed layout: header=9, len=4, pad=12, payload=10*double)");
  }

private:
  void onRaw(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
  {
    const auto &buf = msg->data;

    // 최소 길이 검사
    if (buf.size() < kStart + kPayloadBytes) return;

    // 길이 필드 확인 (LE)
    int32_t len = 0;
    rd_i32_le(len, &buf[kHeaderLen]);
    if (len != kExpectedLen) {
      // 길이 불일치면 무시 (안전)
      return;
    }

    const uint8_t *p = &buf[kStart];

    double qw,qx,qy,qz, wx,wy,wz, ax,ay,az;
    rd_f64_le(qw, p + 0 * sizeof(double));
    rd_f64_le(qx, p + 1 * sizeof(double));
    rd_f64_le(qy, p + 2 * sizeof(double));
    rd_f64_le(qz, p + 3 * sizeof(double));
    rd_f64_le(wx, p + 4 * sizeof(double));
    rd_f64_le(wy, p + 5 * sizeof(double));
    rd_f64_le(wz, p + 6 * sizeof(double));
    rd_f64_le(ax, p + 7 * sizeof(double));
    rd_f64_le(ay, p + 8 * sizeof(double));
    rd_f64_le(az, p + 9 * sizeof(double));

    sensor_msgs::msg::Imu out;
    out.header.stamp = now();
    out.header.frame_id = "imu";  // 레거시와 동일하게 고정

    // orientation (쿼터니언)
    out.orientation.w = qw;
    out.orientation.x = qx;
    out.orientation.y = qy;
    out.orientation.z = qz;

    // angular velocity (rad/s)
    out.angular_velocity.x = wx;
    out.angular_velocity.y = wy;
    out.angular_velocity.z = wz;

    // linear acceleration (m/s^2)
    out.linear_acceleration.x = ax;
    out.linear_acceleration.y = ay;
    out.linear_acceleration.z = az;

    // covariance는 미정(-1)로 채워서 소비자가 무시 가능
    for (double &v : out.orientation_covariance) v = -1.0;
    for (double &v : out.angular_velocity_covariance) v = -1.0;
    for (double &v : out.linear_acceleration_covariance) v = -1.0;

    pub_->publish(out);
  }

  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuParser>());
  rclcpp::shutdown();
  return 0;
}
