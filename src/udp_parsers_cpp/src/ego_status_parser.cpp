//ROS2 rclcpp의 핵심 API. 노드 정의/생성, 퍼블리셔/서브스크립션, 로그 등
#include <rclcpp/rclcpp.hpp>

// std/utility
#include <cstdint>   // uint8_t
#include <cstring>   // memcpy

//RAW UDP 바이트를 싣고 다니는 컨테이너 메시지 타입.
#include <std_msgs/msg/byte_multi_array.hpp>

//TurtlebotStatus 메시지 (bridge_msgs 패키지에 정의됨)
#include <bridge_msgs/msg/turtlebot_status.hpp>

//std::bind에서 콜백 첫 번째 인자 바인딩을 위해 사용.
using std::placeholders::_1;

constexpr double kPi = 3.141592653589793;

class EgoStatusParser : public rclcpp::Node { 
public:
  EgoStatusParser() : Node("ego_status_parser") { // ROS2 노드 상속. 노드 이름은 ego_status_parser

    // sub
    sub_ = create_subscription<std_msgs::msg::ByteMultiArray>( // /ego_status_raw 구독. SensorDataQoS()는 센서 스트림에 적합(주로 BestEffort)
      "/ego_status_raw", rclcpp::SensorDataQoS(),
      std::bind(&EgoStatusParser::onRaw, this, _1));
    
    // pub
    tb_pub_   = create_publisher<bridge_msgs::msg::TurtlebotStatus>("/robot/ego_status", 10); // 상태 퍼블리셔.
  }

  // 콜백: RAW → 파싱
private:
  void onRaw(const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
    const auto &buf = msg->data; // 편의상 바이트 벡터 참조.

    // 최소 길이: header(11)+len(4)+aux(12)+payload(32)
    if (buf.size() < 27 + 32) return;
    
    // 1) 헤더/길이 확인
    const std::string header(reinterpret_cast<const char*>(&buf[0]), 11); 
    if (header != "#Turtlebot$") return;

    int32_t data_len = 0;
    std::memcpy(&data_len, &buf[11], 4);
    // (필요시 엔디안 처리: 송신이 리틀엔디안 가정. 아니면 le32toh 사용.)
    if (data_len != 32) return;

    // 2) payload 파싱
    size_t p = 27; // 11+4+12
    auto need = [&](size_t n){ return (p + n) <= buf.size(); };
    auto rd_f32 = [&](float &out){
      if (!need(4)) return false;
      std::memcpy(&out, &buf[p], 4);
      p += 4;
      return true;
    }; // 페이로드 시작 오프셋과 float 안전 읽기 람다. memcpy로 aliasing/정렬 문제 회피.

    // 경계 체크 + 읽기
    float v_lin=0.f, v_ang=0.f;
    if (!rd_f32(v_lin) || !rd_f32(v_ang)) return;
    
    if (!need(1)) return;
    uint8_t batt_charge = buf[p]; p += 1;
    
    float batt_percent=0.f;
    if (!rd_f32(batt_percent)) return;
    
    float x=0.f, y=0.f, z=0.f, heading_deg=0.f;
    if (!rd_f32(x) || !rd_f32(y) || !rd_f32(z) || !rd_f32(heading_deg)) return;
    
    if (!need(3)) return;
    bool can_use_hand = buf[p++] != 0;
    bool can_put      = buf[p++] != 0;
    bool can_lift     = buf[p++] != 0;
    
    // heading은 시뮬레이터가 deg로 보낸다고 가정 → 라디안으로 변환(필요 시)
    const float heading_rad = static_cast<float>(static_cast<double>(heading_deg) * kPi / 180.0);

    // 3) 상태 메시지 발행 (파싱 결과만 내보냄)
    bridge_msgs::msg::TurtlebotStatus ts;
    ts.twist.linear.x  = v_lin; // 선속도
    ts.twist.angular.z = v_ang; // 각속도
    ts.power_supply_status = batt_charge;
    ts.battery_percentage  = batt_percent;

    // 임시 매핑(추후 EgoStatus.msg로 리팩터 권장)
    ts.twist.angular.x = x;
    ts.twist.angular.y = y;
    ts.twist.linear.z  = heading_rad; // heading을 z축 회전으로 매핑

    ts.can_use_hand = can_use_hand;
    ts.can_put      = can_put;
    ts.can_lift     = can_lift;

    tb_pub_->publish(ts); // 필드 매핑 상태 발행
  }

  // 서브스크립션/퍼블리셔 핸들.
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr sub_;
  rclcpp::Publisher<bridge_msgs::msg::TurtlebotStatus>::SharedPtr tb_pub_;
};

int main(int argc, char **argv){ // 표준 ROS2 엔트리포인트. 노드 실행/스핀/종료.
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoStatusParser>());
  rclcpp::shutdown();
  return 0;
}
