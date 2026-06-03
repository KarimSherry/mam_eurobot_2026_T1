import os
from glob import glob
from setuptools import setup, find_packages

# package_name = 'mam_eurobot_2026*'
package_name = 'mam_eurobot_2026'

setup(
    name=package_name,
    version='0.0.1',
    # packages=find_packages(where='.', include=[package_name]),
    packages=find_packages(where='.', include=[f'{package_name}', f'{package_name}.*']),
    include_package_data=True,  
    package_data={             
        package_name: [
            'vision/*.yaml',   
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/urdf', glob('urdf/*')),
        (f'share/{package_name}/path_planning', (
            glob('mam_eurobot_2026/path_planning/*.yaml')
            + glob('mam_eurobot_2026/path_planning/*.rviz')
            + glob('mam_eurobot_2026/path_planning/*.pgm')
        )),
        (f'share/{package_name}/task_manager', glob('mam_eurobot_2026/task_manager/*.yaml')),
        (f'share/{package_name}/vision', glob('mam_eurobot_2026/vision/*.yaml')),
        (f'share/{package_name}/launch', glob('launch/*')),
        (f'share/{package_name}/worlds', glob('worlds/*')),
        (f'share/{package_name}/models/arena3D', [
            path for path in glob('models/arena3D/*') if os.path.isfile(path)
        ]),
        (f'share/{package_name}/models/arena3D/meshes', glob('models/arena3D/meshes/*')),
        (f'share/{package_name}/models/aruco', glob('models/aruco/*')),
        (f'share/{package_name}/models/crate_blue', glob('models/crate_blue/*')),
        (f'share/{package_name}/models/crate_yellow', glob('models/crate_yellow/*')),
        (f'share/{package_name}/models/cursor', [
            path for path in glob('models/cursor/*') if os.path.isfile(path)
        ]),
        (f'share/{package_name}/models/cursor/meshes', glob('models/cursor/meshes/*')),
        (f'share/{package_name}/models/gripper', glob('models/gripper/*')),
        (f'share/{package_name}/models/mat', glob('models/mat/*')),
        (f'share/{package_name}/models/simple_robot', glob('models/simple_robot/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Freddy Liendo',
    maintainer_email='liendomf@univ-smb.fr',
    description='This is a template package for the Master Advanced Mechatronics teams preparing for the Eurobot 2026',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'py_test = mam_eurobot_2026.py_test:main',
            'aruco_detector = mam_eurobot_2026.vision.aruco_detector:main',
            'color_detector = mam_eurobot_2026.vision.color_detector:main',
            'detected_crates_tf = mam_eurobot_2026.vision.detected_crates_tf:main',
            'world_to_topcamera = mam_eurobot_2026.vision.world_to_topcamera:main' ,
            'estimate_cursor_position = mam_eurobot_2026.vision.estimate_cursor_position:main',
            'robot_pose_from_beacon = mam_eurobot_2026.vision.robot_pose_from_beacon:main',
            'staging_path_planner_node = mam_eurobot_2026.path_planning.staging_path_planner_node:main',
            'task_goal_path_planner_node = mam_eurobot_2026.path_planning.task_goal_path_planner_node:main',
            'pose_info_filter_gz = mam_eurobot_2026.localization.pose_info_filter_gz:main',
            'dammy_TF = mam_eurobot_2026.path_planning.dammy_TF:main',
            'true_tf_from_odom = mam_eurobot_2026.localization.true_tf_from_odom:main',
            'gt_pose_to_map_tf = mam_eurobot_2026.localization.gt_pose_to_map_tf:main',
            'strategy_tree = mam_eurobot_2026.behavior_tree.strategy_tree:main',
            'task_manager_fsm = mam_eurobot_2026.task_manager.task_manager_fsm:main',
            'gripper_keyboard = mam_eurobot_2026.gripper.keyboard_gripper:main',
            'path_follow_client = mam_eurobot_2026.movement.path_follow_client:main',
            'fixed_goal_path_planner_node = mam_eurobot_2026.path_planning.fixed_goal_path_planner_node:main',
        ],
    },
)
