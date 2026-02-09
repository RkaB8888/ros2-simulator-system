// src/state_estimator/src/odom_publisher.cpp
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "bridge_msgs/msg/turtlebot_status.hpp"
using EgoMsg = bridge_msgs::msg::TurtlebotStatus;

// -----------------------------------------------
// 입력 메시지 선택
// - 우리가 사용할 메시지: bridge_msgs/msg/TurtlebotStatus
//   필드: geometry_msgs/Twist twist, ... (x/y/yaw 없음)
//   → 선속/각속도만 있으므로 내부에서 적분해 pose(x,y,yaw)를 만든다.
// -----------------------------------------------

using namespace std::chrono_literals;

class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher()
  : Node("odom_publisher")
  {
    // ---- Parameters
    frame_id_        = declare_parameter<std::string>("frame_id", "odom"); // odom 프레임
    child_frame_id_  = declare_parameter<std::string>("child_frame_id", "base_link"); // 로봇 프레임
    publish_tf_      = declare_parameter<bool>("publish_tf", true); // TF 발행 여부

    publish_rate_    = declare_parameter<double>("publish_rate", 0.0);   // 0이면 콜백마다 발행
    initial_yaw_deg_  = declare_parameter<double>("initial_yaw_deg", 0.0); // Twist-only 시작 각도
    // TurtlebotStatus는 SI 단위(m/s, rad/s)라고 가정
    // (다를 경우 scale 파라미터로 조정)
    lin_scale_        = declare_parameter<double>("linear_scale", 1.0);    // m/s * scale
    ang_scale_        = declare_parameter<double>("angular_scale", 1.0);   // rad/s * scale
    // 초기 공분산 (단순값)
    pose_cov_x_  = declare_parameter<double>("covariance.x", 0.02);
    pose_cov_y_  = declare_parameter<double>("covariance.y", 0.02);
    pose_cov_yaw_ = declare_parameter<double>("covariance.yaw", 0.04);


    // ---- Publisher / TF
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ---- Subscriber / 밀린 데이터를 과감히 버리고 최신 데이터를 받습니다.
    ego_sub_ = create_subscription<EgoMsg>(
      "ego_status", rclcpp::SensorDataQoS(),
      std::bind(&OdomPublisher::egoCallbackBridge, this, std::placeholders::_1));

    // 초기 상태
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = deg2rad(initial_yaw_deg_);

    if (publish_rate_ > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&OdomPublisher::onTimer, this));
    }

    RCLCPP_INFO(get_logger(),
      "odom_publisher started. frame_id='%s', child='%s', publish_tf=%s, rate=%.2f Hz (Twist-only integration mode)",
      frame_id_.c_str(), child_frame_id_.c_str(), publish_tf_ ? "true" : "false", publish_rate_);
  }

private:
  // ---------- 샘플(속도 위주) ----------
  struct Sample {
    double vx_mps{0.0};
    double wz_rps{0.0};
    rclcpp::Time stamp;
    bool valid{false};
  };

  static double deg2rad(double d) { return d * M_PI / 180.0; }
  static double normalize_yaw(double r){
    // -pi ~ pi로 정규화
    return std::atan2(std::sin(r), std::cos(r));
  }

  // TurtlebotStatus → Sample
  bool parseBridge(const EgoMsg &msg, Sample &out) const
  {
    // msg.twist.linear.x: m/s, msg.twist.angular.z: rad/s (가정)
    out.vx_mps = msg.twist.linear.x * lin_scale_;
    out.wz_rps = msg.twist.angular.z * ang_scale_;
    
    rclcpp::Time stamp_msg{msg.header.stamp};
    const auto now_node = this->now();
    if (stamp_msg.nanoseconds() == 0 || stamp_msg > now_node + rclcpp::Duration::from_seconds(0.1)) {
      stamp_msg = now_node;
    }
    out.stamp = stamp_msg;
    out.valid  = true;
    return true;
  }

  void egoCallbackBridge(const EgoMsg::ConstSharedPtr msg)
  {
    Sample s;
    if (!parseBridge(*msg, s)) return;
    handleSample(s);
  }

  void integrate(const Sample &s)
  {
    // dt 계산 및 보호
    double dt = (s.stamp - last_state_stamp_).seconds();
    if (dt < 0.0) {
      // 시간이 역행하면 무시하고 now로 보정
      dt = 0.0;
      last_state_stamp_ = now();
      return;
    }

    // 지연시간 동안의 '추측 주행'은 너무 위험하므로 포기(Skip)합니다.
    if (dt > 0.1) {
       RCLCPP_WARN(get_logger(), "Lag (dt=%.3f s). Skipping integration.", dt);
       last_state_stamp_ = s.stamp; 
       return; 
    }

    last_state_stamp_ = s.stamp;

    // 2D 비고: 로봇 헤딩(yaw_) 기준 전진
    // 간단한 전진 오일러 적분
    // 구현이 단순하고 CPU 부담이 적음. 고속 스트림에서 충분히 안정적. (필요하면 midpoint/Runge-Kutta로 교체 가능)
    yaw_ = normalize_yaw(yaw_ + s.wz_rps * dt);
    x_  += s.vx_mps * std::cos(yaw_) * dt;
    y_  += s.vx_mps * std::sin(yaw_) * dt;
  }

  void handleSample(const Sample &s)
  {
    if (!s.valid) return;

    // 첫 샘플이면 스냅만 맞추고 적분 건너뜀
    if (last_state_stamp_.nanoseconds() == 0) {
      last_state_stamp_ = s.stamp;
    } else {
      integrate(s);
    }

    // 최신 속도 저장
    latest_ = s;

    // 즉시 발행 모드(타이머 비사용)라면 바로 퍼블리시
    if (publish_rate_ <= 0.0) {
      publishOdomAndTF(s.stamp, latest_.vx_mps, latest_.wz_rps);
    }
  }

  void onTimer()
  {
    if (!latest_.valid) return;

    auto now_node = this->now();
    if (last_state_stamp_.nanoseconds() == 0) {
      last_state_stamp_ = now_node;
    } else {
      Sample neutral = latest_;
      neutral.stamp = now_node;
      integrate(neutral);
    }

    publishOdomAndTF(now_node, latest_.vx_mps, latest_.wz_rps);
  }

  void publishOdomAndTF(const rclcpp::Time &stamp, double vx_mps, double wz_rps) const
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // Pose Covariance (EKF 설정에서 x, y, yaw를 false로 하면 무시되지만 형식상 채움)
    for (double &c : odom.pose.covariance) c = 0.0;
    odom.pose.covariance[0]  = pose_cov_x_;
    odom.pose.covariance[7]  = pose_cov_y_;
    odom.pose.covariance[35] = pose_cov_yaw_;

    // Twist (속도) 채우기
    odom.twist.twist.linear.x  = vx_mps;
    odom.twist.twist.linear.y  = 0.0;    // 명시적으로 0.0 대입 (구조적 제약)
    odom.twist.twist.angular.z = wz_rps;

    // Twist Covariance 추가
    // 초기화
    for (double &c : odom.twist.covariance) c = 0.0;
    // EKF에서 사용하는 값들에 대해 공분산 설정
    // 값이 작을수록 "이 센서 값을 강하게 믿는다"는 뜻
    // 1) Linear X (전진 속도): 적당히 신뢰 (0.02)
    odom.twist.covariance[0] = 0.02;
    // 2) Linear Y (횡 속도): "0"이라는 사실을 매우 강하게 신뢰 (0.001 ~ 0.02)
    odom.twist.covariance[7] = 0.02;
    // 3) Angular Z (회전 속도): EKF에선 안 쓰지만(false), 형식상 채움
    odom.twist.covariance[35] = 0.04;

    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = frame_id_;
      tf.child_frame_id  = child_frame_id_;
      tf.transform.translation.x = x_;
      tf.transform.translation.y = y_;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf);
    }
  }

private:
  // Params
  std::string frame_id_;
  std::string child_frame_id_;
  bool publish_tf_{true};

  double publish_rate_{0.0};  // 0 → 콜백마다
  double initial_yaw_deg_{0.0};   // 시작 헤딩(도)
  double lin_scale_{1.0};         // 선속 스케일
  double ang_scale_{1.0};         // 각속도 스케일
  double pose_cov_x_;      // 위치 공분산 (x)
  double pose_cov_y_;      // 위치 공분산 (y)
  double pose_cov_yaw_;    // 위치 공분산 (yaw)

  // State (적분 기반)
  double x_{0.0}, y_{0.0}, yaw_{0.0};

  rclcpp::Time last_state_stamp_;
  Sample latest_;

  // ROS I/F
  rclcpp::Subscription<EgoMsg>::SharedPtr ego_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}