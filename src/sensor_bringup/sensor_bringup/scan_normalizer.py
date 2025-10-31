# src/sensor_bringup/sensor_bringup/scan_normalizer.py
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanNormalizer(Node):
    def __init__(self):
        super().__init__('scan_normalizer')
        self.frame_id = self.declare_parameter('frame_id', 'laser_link').get_parameter_value().string_value
        self.topic_in = self.declare_parameter('topic_in', 'scan_in').get_parameter_value().string_value
        self.topic_out = self.declare_parameter('topic_out', 'scan_out').get_parameter_value().string_value
        self.enable_norm = self.declare_parameter('enable_normalize', False).get_parameter_value().bool_value
        self.invert = self.declare_parameter('invert_cw_to_ccw', False).get_parameter_value().bool_value
        self.rotate_180 = self.declare_parameter('rotate_180_degrees', False).get_parameter_value().bool_value

        self.sub = self.create_subscription(LaserScan, self.topic_in, self.cb, 10)
        self.pub = self.create_publisher(LaserScan, self.topic_out, 10)
        self.get_logger().info(
            f"ScanNormalizer: {self.topic_in} -> {self.topic_out} "
            f"(normalize={self.enable_norm}, invert={self.invert}, rotate_180={self.rotate_180})"
        )

    def wrap_angle(self, a):
        # [-pi, pi)로 맞추고 싶으면:
        while a >= math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a
    
    def cb(self, msg: LaserScan):
        out = LaserScan()
        # header
        out.header = msg.header
        out.header.frame_id = self.frame_id

        # --- ⬇️ (1) 180도 회전 (Unity 보상) ---
        # (YAML에서 rotate_180_degrees: true로 설정 시 작동)
        if self.rotate_180 and msg.ranges:
            mid = len(msg.ranges) // 2
            ranges_buf = msg.ranges[mid:] + msg.ranges[:mid]
            intensities_buf = list(msg.intensities) if msg.intensities else []
            out.angle_min = self.wrap_angle(msg.angle_min + math.pi)
            out.angle_max = self.wrap_angle(msg.angle_max + math.pi)
        else:
            # 180도 회전을 하지 않는 경우 원본 사용
            ranges_buf = list(msg.ranges)
            intensities_buf = list(msg.intensities)
        
        # --- ⬇️ (2) CW/CCW 좌우 반전 (YAML 설정에 따름) ---
        if self.enable_norm and self.invert:
            # CW → CCW: 각도 부호 반전 + 샘플 역순
            out.angle_min = -msg.angle_max
            out.angle_max = -msg.angle_min
            out.angle_increment = abs(msg.angle_increment)
            out.ranges = list(reversed(ranges_buf))
            out.intensities = list(reversed(intensities_buf)) if intensities_buf else []
        else:
            # (Unity는 이 경로를 타야 함)
            # 패스스루
            out.angle_min = msg.angle_min
            out.angle_max = msg.angle_max
            out.angle_increment = msg.angle_increment
            out.ranges = ranges_buf
            out.intensities = intensities_buf

        # (공통 메타데이터)
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(ScanNormalizer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
