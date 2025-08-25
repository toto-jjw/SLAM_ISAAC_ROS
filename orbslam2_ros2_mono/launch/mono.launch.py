# mono.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 내부 cam_mono.yaml 기본값
    pkg_share = get_package_share_directory('orbslam2_ros2_mono')
    default_settings = os.path.join(pkg_share, 'config', 'cam_mono.yaml')

    # ORBvoc 기본값: 환경변수 ORBSLAM_VOCAB 우선, 없으면 $PWD 기준
    default_vocab = os.environ.get(
        'ORBSLAM_VOCAB',
        os.path.join(os.getenv('PWD', ''), 'Portable_ORB_SLAM2', 'Vocabulary', 'ORBvoc.txt')
    )

    vocab_arg    = DeclareLaunchArgument('vocab',    default_value=default_vocab,
                                         description='Path to ORBvoc.txt')
    settings_arg = DeclareLaunchArgument('settings', default_value=default_settings,
                                         description='Path to camera settings .yaml')
    image_topic  = DeclareLaunchArgument('image_topic',
                                         default_value='/front_stereo_camera/left/image_rect_color')
    world_frame  = DeclareLaunchArgument('world_frame', default_value='map')
    child_frame  = DeclareLaunchArgument('child_frame', default_value='front_stereo_camera_left')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    max_path     = DeclareLaunchArgument('max_path_length', default_value='10000')

    node = Node(
        package='orbslam2_ros2_mono',
        executable='ros2_mono',
        name='orbslam2_mono',
        output='screen',
        # 실행 인자(필수): <vocab> <settings>
        arguments=[LaunchConfiguration('vocab'), LaunchConfiguration('settings')],
        parameters=[{
            'image_topic':     LaunchConfiguration('image_topic'),
            'world_frame':     LaunchConfiguration('world_frame'),
            'child_frame':     LaunchConfiguration('child_frame'),
            'use_sim_time':    LaunchConfiguration('use_sim_time'),
            'max_path_length': LaunchConfiguration('max_path_length'),
        }]
    )

    return LaunchDescription([
        vocab_arg, settings_arg, image_topic, world_frame, child_frame,
        use_sim_time, max_path,
        node
    ])
