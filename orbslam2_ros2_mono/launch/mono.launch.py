# mono.launch.py — launches the ORB-SLAM2 monocular tracking node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('orbslam2_ros2_mono')
    default_settings = os.path.join(pkg_share, 'config', 'cam_mono.yaml')

    # ORBSLAM_VOCAB env var overrides the default path
    default_vocab = os.environ.get(
        'ORBSLAM_VOCAB',
        os.path.join(os.getenv('PWD', ''), 'Portable_ORB_SLAM2', 'Vocabulary', 'ORBvoc.txt')
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'vocab',
            default_value=default_vocab,
            description='Absolute path to ORBvoc.txt. '
                        'Override via ORBSLAM_VOCAB env var or this argument.'),
        DeclareLaunchArgument(
            'settings',
            default_value=default_settings,
            description='Absolute path to camera settings YAML (cam_mono.yaml).'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/front_stereo_camera/left/image_rect_color',
            description='Input camera image topic (sensor_msgs/Image).'),
        DeclareLaunchArgument(
            'world_frame',
            default_value='map',
            description='TF frame ID for the SLAM world/map origin.'),
        DeclareLaunchArgument(
            'child_frame',
            default_value='front_stereo_camera_left',
            description='TF child frame ID published for the camera pose.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock from Isaac Sim (true) or wall clock (false).'),
        DeclareLaunchArgument(
            'max_path_length',
            default_value='10000',
            description='Maximum number of poses kept in /orb_slam2/trajectory history.'),

        Node(
            package='orbslam2_ros2_mono',
            executable='ros2_mono',
            name='orbslam2_mono',
            output='screen',
            arguments=[
                LaunchConfiguration('vocab'),
                LaunchConfiguration('settings'),
            ],
            parameters=[{
                'image_topic':     LaunchConfiguration('image_topic'),
                'world_frame':     LaunchConfiguration('world_frame'),
                'child_frame':     LaunchConfiguration('child_frame'),
                'use_sim_time':    LaunchConfiguration('use_sim_time'),
                'max_path_length': LaunchConfiguration('max_path_length'),
            }],
        ),
    ])
