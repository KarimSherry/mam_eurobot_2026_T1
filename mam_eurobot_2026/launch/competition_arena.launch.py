# test launch file for Mac (cleaned)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def _spawn_model_cmd(file_uri: str, name: str, x: float, y: float, z: float, Y: float = None):
    """ros_gz_sim create のコマンド生成ヘルパ"""
    cmd = [
        "ros2", "run", "ros_gz_sim", "create",
        "-file", file_uri,
        "-name", name,
        "-x", str(x), "-y", str(y), "-z", str(z),
    ]
    if Y is not None:
        cmd += ["-Y", str(Y)]
    return cmd


def generate_launch_description():
    # パッケージ共有パス（models や worlds を見せる）
    pkg_share = FindPackageShare('mam_eurobot_2026')

    # ============ 環境変数 ============
    env_actions = [
        SetEnvironmentVariable('DISPLAY', ':1'),  # VNC / Xvfb
        SetEnvironmentVariable('HOME', '/home/rosdev'),
        SetEnvironmentVariable('ROS_LOG_DIR', '/home/rosdev/.ros/log'),
        SetEnvironmentVariable('XDG_RUNTIME_DIR', '/tmp/runtime-rosdev'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('IGN_RENDER_ENGINE', 'ogre2'),
        # Gazebo のリソース探索（models/worlds）にパッケージを通す
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg_share),
        # 作業用ディレクトリなどを事前作成
        ExecuteProcess(
            cmd=[
                '/bin/bash', '-lc',
                'mkdir -p /home/rosdev/.ros/log '
                '&& mkdir -p /tmp/runtime-rosdev '
                '&& chmod 700 /tmp/runtime-rosdev'
            ],
            output='screen'
        ),
    ]

    # ============ 引数 ============
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg_share, 'worlds', 'competition_world.sdf']),
        description='SDF world file'
    )
    world_cfg = LaunchConfiguration('world')

    # ============ Ignition (Gazebo Sim) ============
    ign = ExecuteProcess(
        cmd=["ign", "gazebo", world_cfg],
        output="screen",
    )

    # ============ Ignition 起動後に spawn ============
    # models/ 以下を GZ_SIM_RESOURCE_PATH で通しているので "model://models/<name>" でOK
    spawn_after_ign = RegisterEventHandler(
        OnProcessStart(
            target_action=ign,
            on_start=[
                ExecuteProcess(
                    cmd=_spawn_model_cmd(
                        file_uri="model://models/simple_robot",
                        name="simple_robot",
                        x=0.80, y=-1.20, z=0.05, Y=3.1415
                    ),
                    output="screen",
                ),
            ],
        )
    )

    # ============ ブリッジ ============
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
    )
    img_bridge =  Node( 
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge_front_camera',
        # namespace='front_camera',     
        arguments=['/front_camera'], 
        remappings=[
            ('image', '/front_camera/image'),
            ('camera_info', '/front_camera/camera_info'),
        ],
        parameters=[{'qos': 'sensor_data'}],  # BESTEFFORT VOLATILE shallow depth
        output='screen',
    )
    
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_bridge',
        output='screen',
        arguments=[
            # '/front_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            # '/front_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/world/eurobot_2026_arena/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
        ],
    )

    # ============ RViz ============
    rviz_env = {
        'DISPLAY': ':1',
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'LD_LIBRARY_PATH': '/opt/ros/humble/opt/rviz_ogre_vendor/lib:' + os.environ.get('LD_LIBRARY_PATH', ''),
        'HOME': '/home/rosdev',
        'ROS_LOG_DIR': '/home/rosdev/.ros/log',
        'XDG_RUNTIME_DIR': '/tmp/runtime-rosdev',
        'RCUTILS_LOGGING_USE_STDOUT': '1',
        'AMENT_PREFIX_PATH': os.environ.get('AMENT_PREFIX_PATH', ''),
    }

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        env=rviz_env,
        # parameters=[{'use_sim_time': True}],  # 必要ならコメント解除
    )

    return LaunchDescription([
        *env_actions,
        world_arg,
        ign,
        spawn_after_ign,
        cmd_vel_bridge,
        img_bridge, 
        camera_bridge,
        rviz,
    ])
