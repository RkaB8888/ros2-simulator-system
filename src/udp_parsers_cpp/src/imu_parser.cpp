// src/udp_parsers_cpp/src/imu_parser.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cstring>
#include <cmath>
#include <type_traits>

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
  ImuParser() 
  : Node("imu_parser"),
    last_publish_time_(0, 0, RCL_ROS_TIME)
  {
    // /imu_raw는 SensorDataQoS(BEST_EFFORT)
    sub_ = create_subscription<std_msgs::msg::ByteMultiArray>(
      "imu_raw", rclcpp::SensorDataQoS(),
      std::bind(&ImuParser::onRaw, this, _1));

    // 결과는 rqt/rviz 친화적으로 RELIABLE(depth=10)
    pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "imu_link");

    RCLCPP_INFO(get_logger(), "imu_parser started (fixed layout: header=9, len=4, pad=12, payload=10*double)");
  }

private:
  void onRaw(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
  {
    const auto current_time = this->now();

    // ==========================================================
    // UDP 데이터 폭주(Burst) 방지 로직
    // ==========================================================
    // 이전 메시지와의 시간 차이 계산
    double diff = (current_time - last_publish_time_).seconds();

    // 만약 0.01초(10ms)보다 짧은 간격으로 데이터가 들어왔다면?
    // -> 이것은 네트워크 렉으로 밀렸던 데이터가 한꺼번에 쏟아지는 것임.
    // -> EKF 점프를 막기 위해 이 데이터는 무시(Drop)한다.
    if (diff < 0.002) {
        // [추가] 버스트 감지 경고 로그 (도배 방지를 위해 1초(1000ms)에 한 번만 출력)
        RCLCPP_WARN_THROTTLE(
            get_logger(), 
            *this->get_clock(), 
            1000, // 1000ms = 1초 제한
            "IMU Burst detected! Dropping packet to prevent jump. (dt: %.5f s)", diff
        );
        return; 
    }

    // 정상 데이터라면 마지막 시간 업데이트
    last_publish_time_ = current_time;
    // ==========================================================

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
    out.header.stamp = current_time;
    out.header.frame_id = frame_id_;

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

    // 초기화
    for (int i = 0; i < 9; ++i) {
      out.angular_velocity_covariance[i] = 0.0;
      out.linear_acceleration_covariance[i] = 0.0;
    }

    // ==========================================================
    // 공분산(Covariance) 조정
    // ==========================================================
    // 0.01 -> 0.05 로 변경하여 EKF가 IMU를 "덜 맹신"하게 만듦.
    // 데이터 튐 현상에 대한 저항력을 높임.
    out.angular_velocity_covariance[0] = 0.05; // X (Roll rate)
    out.angular_velocity_covariance[4] = 0.05; // Y (Pitch rate)
    out.angular_velocity_covariance[8] = 0.05; // Z (Yaw rate) - 중요!

    // 선가속도는 안 쓰지만, 혹시 모르니 얘도 양수로
    out.linear_acceleration_covariance[0] = 0.05; 
    out.linear_acceleration_covariance[4] = 0.05;
    out.linear_acceleration_covariance[8] = 0.05;

    pub_->publish(out);
  }

  std::string frame_id_;
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

  // 마지막으로 메시지를 발행한 시간 저장용 변수
  rclcpp::Time last_publish_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuParser>());
  rclcpp::shutdown();
  return 0;
}
