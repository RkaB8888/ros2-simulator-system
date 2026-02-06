# bridge_bringup/launch/bridge.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def _pkg_share(pkg, *rel):
    return os.path.join(get_package_share_directory(pkg), *rel)

def generate_launch_description():
    # 통합 YAML 기본값 (환경별 파일로 교체 가능)
    default_cfg = _pkg_share('bridge_bringup', 'config', 'system.wsl.yaml')

    # 1. RAW UDP (수신)
    raw_launch     = _pkg_share('udp_raw_bridge',    'launch', 'raw_6ports.launch.py')

    # 2. Parsers (해석)
    parsers_launch = _pkg_share('udp_parsers_cpp',   'launch', 'parsers_all.launch.py')

    # 3. Sensors (정규화) - Scan Normalizer
    sensor_launch  = _pkg_share('sensor_bringup',    'launch', 'sensor_bringup.launch.py')

    # 4. State Estimator (위치 추정) - Odom
    odom_launch    = _pkg_share('state_estimator',   'launch', 'odom.launch.py')

    # 5. Safety Chain (안전 장치 + Lifecycle Manager)
    safety_launch  = _pkg_share('safety_bringup',    'launch', 'safety_chain.launch.py')

    # 6. UDP TX (송신)
    tx_launch      = _pkg_share('udp_tx_bridge',     'launch', 'tx_all.launch.py')
    

    return LaunchDescription([
        # 공통 네임스페이스(멀티 로봇 대비). 기본은 루트("")
        DeclareLaunchArgument('ns', default_value=''),
        DeclareLaunchArgument('config_file', default_value=default_cfg),
        # 레이어별 기본 respawn 정책: RX=false, Parsers=true, TX=true
        DeclareLaunchArgument('respawn_raw',     default_value='false'),
        DeclareLaunchArgument('respawn_parsers', default_value='true'),
        DeclareLaunchArgument('respawn_tx',      default_value='true'),
        # 세이프티 체인 on/off 토글(필요 시 비활성)
        DeclareLaunchArgument('enable_safety',   default_value='true'),
        # RAW 레이어 노드들의 로그 레벨을 중앙에서 선언
        DeclareLaunchArgument('log_level_raw', default_value='info'),

        LogInfo(msg=['[bridge_all] using config: ', LaunchConfiguration('config_file')]),

        # RAW 레이어 (수신)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(raw_launch),
            launch_arguments={
                'ns':          LaunchConfiguration('ns'),
                'config_file': LaunchConfiguration('config_file'),
                'respawn':     LaunchConfiguration('respawn_raw'),
                'log_level':   LaunchConfiguration('log_level_raw'),
            }.items(),
        ),

        # PARSERS 레이어 (파싱)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(parsers_launch),
            launch_arguments={
                'ns':      LaunchConfiguration('ns'), 
                'respawn': LaunchConfiguration('respawn_parsers'),
                'log_level':   LaunchConfiguration('log_level_raw'),
            }.items(),
        ),

        # SENSORS 레이어
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensor_launch),
            launch_arguments={
                'ns': LaunchConfiguration('ns'),
            }.items(),
        ),

        # ODOM 레이어
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odom_launch),
            launch_arguments={
                'ns': LaunchConfiguration('ns'),
                'log_level': LaunchConfiguration('log_level_raw'),
            }.items(),
        ),

        # SAFETY 체인 (twist_mux → velocity_smoother → collision_monitor)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(safety_launch),
            condition=IfCondition(LaunchConfiguration('enable_safety')),
            launch_arguments={
                'ns':            LaunchConfiguration('ns'),
                'log_level':     LaunchConfiguration('log_level_raw'),
            }.items(),
        ),

        # TX 레이어 (송신)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tx_launch),
            launch_arguments={
                'ns':          LaunchConfiguration('ns'),  
                'config_file': LaunchConfiguration('config_file'),
                'respawn':     LaunchConfiguration('respawn_tx'),
                'log_level':   LaunchConfiguration('log_level_raw'),
            }.items(),
        ),
    ])
