# Ego 상태 (우선 Ego 파서가 쓰는 필드만 포함)
std_msgs/Header header
geometry_msgs/Twist twist
uint8 power_supply_status
float32 battery_percentage
bool can_use_hand
bool can_put
bool can_lift
