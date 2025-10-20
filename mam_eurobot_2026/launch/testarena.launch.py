# test launch file for Mac
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # pkg_path = FindPackageShare('mam_eurobot_2026')
    
    # return LaunchDescription([
    #     SetEnvironmentVariable(
    #         'GZ_SIM_RESOURCE_PATH',
    #         pkg_path
    #     ),
    #     SetEnvironmentVariable('HOME', '/home/rosdev'),
    #     SetEnvironmentVariable('ROS_LOG_DIR', '/home/rosdev/.ros/log'),
    #     SetEnvironmentVariable('XDG_RUNTIME_DIR', '/tmp/runtime-rosdev'),
    #     SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg_path),
    #     SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
    #     SetEnvironmentVariable('IGN_RENDER_ENGINE', 'ogre2'),
    #     ExecuteProcess(
    #         cmd=[
    #             '/bin/bash', '-lc',
    #             'mkdir -p /home/rosdev/.ros/log '
    #             '&& mkdir -p /tmp/runtime-rosdev '
    #             '&& chmod 700 /tmp/runtime-rosdev'
    #         ],
    #         output='screen'
    #     ),
    #     DeclareLaunchArgument("world", default_value=PathJoinSubstitution([pkg_path, 'worlds', 'arena_world.sdf'])),
    #     ExecuteProcess(
    #         # cmd=["ign", "gazebo", "-r", LaunchConfiguration("world")],
    #         # cmd=["ign", "gazebo", "-s", LaunchConfiguration("world")], # NO GUI
    #         cmd=["ign", "gazebo", LaunchConfiguration("world")], 
    #         output="screen"
    #     ),
    #     ExecuteProcess(
    #         cmd=[
    #             "ros2", "run", "ros_gz_sim", "create",
    #             "-file", "file://models/crate",
    #             "-name", "crate",
    #             "-x", "-0.20", "-y", "0", "-z", "0.05"
    #         ],
    #         output="screen"
    #     ),
    #     ExecuteProcess(
    #         cmd=[
    #             "ros2", "run", "ros_gz_sim", "create",
    #             "-file", "file://models/simple_robot",
    #             "-name", "simple_robot",
    #             "-x", "0.05", "-y", "0", "-z", "0.05", "-Y", "3.1415"
    #         ],
    #         output="screen"
    #     ),
    #     Node(
    #         package='ros_gz_bridge',
    #         executable='parameter_bridge',
    #         name='cmd_vel_bridge',
    #         output='screen',
    #         arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist']
    #     ),
    #     Node(
    #         package='ros_gz_bridge',
    #         executable='parameter_bridge',
    #         name='gz_camera_bridge',
    #         output='screen',
    #         arguments=[
    #             # 画像とカメラ内参照行列
    #             '/front_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
    #             '/front_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
    #             # ついでにシミュレーション時刻も（RVizや他ノードで便利）
    #             '/world/eurobot_2026_arena/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
    #             ]
    #     ),
    #     Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         name='rviz2',
    #         output='screen',
    #         env={'DISPLAY': ':1', 
    #              'LIBGL_ALWAYS_SOFTWARE': '1',
    #              'LD_LIBRARY_PATH': '/opt/ros/humble/opt/rviz_ogre_vendor/lib:' + os.environ.get('LD_LIBRARY_PATH', ''),
    #              'HOME': '/home/rosdev',
    #              'ROS_LOG_DIR': '/home/rosdev/.ros/log',
    #              'XDG_RUNTIME_DIR': '/tmp/runtime-rosdev',
    #     # ディスクに書かず標準出力へ（根本回避）
    #              'RCUTILS_LOGGING_USE_STDOUT': '1',
    #              'AMENT_PREFIX_PATH': os.environ.get('AMENT_PREFIX_PATH', ''),  # 最終的にこれで解決

    #             },
            
    #     )
    # ])

    pkg_path = FindPackageShare('mam_eurobot_2026')

    ign_proc = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", LaunchConfiguration("world")],  # 実行開始
        output="screen",
        additional_env={                    # ← Ignition にも VNC を渡す
            'DISPLAY': ':1',
        },
    )
    create_crate = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=["ros2","run","ros_gz_sim","create",
                #  "-file","file://models/crate",
                 "-file","model://models/crate",
                #  "-file","/home/rosdev/eurobot_2026_ws/src/mam_eurobot_2026/models/crate/model.sdf",
                 "-name","crate",
                 "-x","-0.20","-y","0","-z","0.05"],
            output="screen"
        )]
    )
    create_robot = TimerAction(
        period=1.2,
        actions=[ExecuteProcess(
            cmd=["ros2","run","ros_gz_sim","create",
                #  "-file","file://models/simple_robot",
                 "-file","model://models/simple_robot",
                 "-name","simple_robot",
                 "-x","0.05","-y","0","-z","0.05","-Y","3.1415"],
            output="screen"
        )]
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg_path),
        SetEnvironmentVariable('HOME', '/home/rosdev'),
        SetEnvironmentVariable('ROS_LOG_DIR', '/home/rosdev/.ros/log'),
        SetEnvironmentVariable('XDG_RUNTIME_DIR', '/tmp/runtime-rosdev'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('IGN_RENDER_ENGINE', 'ogre2'),
        ExecuteProcess(
            cmd=['/bin/bash','-lc',
                 'mkdir -p /home/rosdev/.ros/log '
                 '&& mkdir -p /tmp/runtime-rosdev '
                 '&& chmod 700 /tmp/runtime-rosdev'],
            output='screen'
        ),

        DeclareLaunchArgument("world", default_value=PathJoinSubstitution([pkg_path,'worlds','arena_world.sdf'])),

        ign_proc,              
        create_crate, 
        create_robot,

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            output='screen',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist']
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_camera_bridge',
            output='screen',
            arguments=[
                '/front_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                # '/front_camera/image@ignition.msgs.Image@sensor_msgs/msg/Image', # gemini answered
                '/front_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                # '/front_camera/camera_info@ignition.msgs.CameraInfo@sensor_msgs/msg/CameraInfo', # gemini answer
                '/world/eurobot_2026_arena/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',  # ← 先頭スラッシュ追加
                # '/world/eurobot_2026_arena/clock@ignition.msgs.Clock@rosgraph_msgs/msg/Clock',  # ← 先頭スラッシュ追加 gemini answer
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            env={
                'DISPLAY': ':1',
                'LIBGL_ALWAYS_SOFTWARE': '1',
                'LD_LIBRARY_PATH': '/opt/ros/humble/opt/rviz_ogre_vendor/lib:' + os.environ.get('LD_LIBRARY_PATH',''),
                'HOME': '/home/rosdev',
                'ROS_LOG_DIR': '/home/rosdev/.ros/log',
                'XDG_RUNTIME_DIR': '/tmp/runtime-rosdev',
                'RCUTILS_LOGGING_USE_STDOUT': '1',
                'AMENT_PREFIX_PATH': os.environ.get('AMENT_PREFIX_PATH',''),
            },
        ),
    ])