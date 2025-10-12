# bridge_bringup/launch/bridge.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def _pkg_share(pkg, *rel):
    return os.path.join(get_package_share_directory(pkg), *rel)

def generate_launch_description():
    # 통합 YAML 기본값 (환경별 파일로 교체 가능)
    default_cfg = _pkg_share('bridge_bringup', 'config', 'system.wsl.yaml')

    raw_launch     = _pkg_share('udp_raw_bridge',    'launch', 'raw_6ports.launch.py')
    parsers_launch = _pkg_share('udp_parsers_cpp',   'launch', 'parsers_all.launch.py')
    tx_launch      = _pkg_share('udp_tx_bridge',     'launch', 'tx_all.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_cfg),
        # 레이어별 기본 respawn 정책: RX=false, Parsers=true, TX=true
        DeclareLaunchArgument('respawn_raw',     default_value='false'),
        DeclareLaunchArgument('respawn_parsers', default_value='true'),
        DeclareLaunchArgument('respawn_tx',      default_value='true'),

        LogInfo(msg=['[bridge_all] using config: ', LaunchConfiguration('config_file')]),

        # RAW 레이어 (수신)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(raw_launch),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file'),
                'respawn':     LaunchConfiguration('respawn_raw'),
            }.items(),
        ),

        # PARSERS 레이어 (파싱)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(parsers_launch),
            launch_arguments={
                'respawn': LaunchConfiguration('respawn_parsers'),
            }.items(),
        ),

        # TX 레이어 (송신)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tx_launch),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file'),
                'respawn':     LaunchConfiguration('respawn_tx'),
            }.items(),
        ),
    ])
