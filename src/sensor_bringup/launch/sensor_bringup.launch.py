# src/sensor_bringup/launch/sensor_bringup.launch.py
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml, math

def _deg2rad(rpy_deg):
    r, p, y = rpy_deg
    return [math.radians(r), math.radians(p), math.radians(y)]

def _gen_nodes(context, *args, **kwargs):
    ns = LaunchConfiguration('ns').perform(context) # ''가 기본
    cfg = LaunchConfiguration('cfg').perform(context) # 'sensors.yaml'이 기본

    # ---- cfg 경로 결정 (절대/상대 모두 지원)
    if os.path.isabs(cfg):
        cfg_path = cfg
    else:
        pkg_share = get_package_share_directory('sensor_bringup')
        cfg_path = os.path.join(pkg_share, 'config', cfg)

    with open(cfg_path, 'r') as f:
        config = yaml.safe_load(f) or {}

    nodes = []

    # --- TF 프레임 ID에 네임스페이스를 적용하는 헬퍼 함수 ---
    def add_ns(frame_id):
        """네임스페이스(ns)가 있으면 'ns/frame_id'를 반환"""
        if ns and not frame_id.startswith('/'):
            return f"{ns}/{frame_id}"
        return frame_id
    
    # 모든 TF의 부모가 될 base_link 프레임 ID
    base_link_frame = add_ns('base_link') # 'base_link' -> 'robot1/base_link'

    # LiDAR
    for s in config.get('lidar', []):
        if not s.get('enabled', True):
            continue
        xyz = s['mount']['xyz_m']
        rpy = _deg2rad(s['mount']['rpy_deg'])
        child_frame = add_ns(s['frame_id'])
        nodes.append(Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            arguments=[
                '--x',  str(xyz[0]),
                '--y',  str(xyz[1]),
                '--z',  str(xyz[2]),
                '--roll',  str(rpy[0]),
                '--pitch', str(rpy[1]),
                '--yaw',   str(rpy[2]),
                '--frame-id',        base_link_frame,
                '--child-frame-id',  child_frame,
            ],
            namespace=ns, 
            name=f"tf_{s['name']}"
        ))
        # 항상 normalizer를 쓰되, normalize.enabled=false면 패스스루 동작
        norm = s.get('normalize', {})
        topics = s['topics']
        nodes.append(Node(
            package='sensor_bringup', 
            executable='scan_normalizer',
            name=f"scan_norm_{s['name']}", 
            namespace=ns, 
            output='screen',
            parameters=[{
                'frame_id': child_frame,
                'topic_in': topics['in'],
                'topic_out': topics['out'],
                'enable_normalize': bool(norm.get('enabled', False)),
                'invert_cw_to_ccw': bool(norm.get('invert_cw_to_ccw', False)),
                'rotate_180_degrees': bool(norm.get('rotate_180_degrees', False)),
            }],
        ))

    # IMU (정적 TF만)
    for s in config.get('imu', []):
        if not s.get('enabled', True):
            continue
        xyz = s['mount']['xyz_m']
        rpy = _deg2rad(s['mount']['rpy_deg'])
        child_frame = add_ns(s['frame_id'])
        nodes.append(Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            arguments=[
                '--x',  str(xyz[0]),
                '--y',  str(xyz[1]),
                '--z',  str(xyz[2]),
                '--roll',  str(rpy[0]),
                '--pitch', str(rpy[1]),
                '--yaw',   str(rpy[2]),
                '--frame-id',        base_link_frame,
                '--child-frame-id',  child_frame,
            ],
            namespace=ns, 
            name=f"tf_{s['name']}"
        ))

    # Camera (정적 TF + optical frame 있으면 보조 TF)
    for s in config.get('camera', []):
        if not s.get('enabled', True):
            continue
        xyz = s['mount']['xyz_m']
        rpy = _deg2rad(s['mount']['rpy_deg'])
        child_frame = add_ns(s['frame_id'])
        nodes.append(Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            arguments=[
                '--x',  str(xyz[0]),
                '--y',  str(xyz[1]),
                '--z',  str(xyz[2]),
                '--roll',  str(rpy[0]),
                '--pitch', str(rpy[1]),
                '--yaw',   str(rpy[2]),
                '--frame-id',        base_link_frame,
                '--child-frame-id',  child_frame,
            ],
            namespace=ns, 
            name=f"tf_{s['name']}"
        ))
        opt = s.get('optical_frame')
        if opt:
            # optical 규칙(roll=-90°, yaw=-90°) 고정 TF
            optical_frame = add_ns(opt)
            nodes.append(Node(
                package='tf2_ros', 
                executable='static_transform_publisher',
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll',  str(-math.pi/2),
                    '--pitch', '0',
                    '--yaw',   str(-math.pi/2),
                    '--frame-id',       child_frame,     # camera_link
                    '--child-frame-id', optical_frame,   # camera_optical_frame
                ],
                namespace=ns, 
                name=f"tf_{s['name']}_optical"
            ))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ns',  default_value='',               description='namespace (optional)'),
        DeclareLaunchArgument('cfg', default_value='sensors.yaml',   description='sensor config relative to sensor_bringup/config or absolute path'),
        OpaqueFunction(function=_gen_nodes),
    ])
