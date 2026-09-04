from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Gazebo camera topic for the x500_mono_cam_down model in world "default".
CAM_TOPIC = ('/world/default/model/x500_mono_cam_down_0/'
             'link/camera_link/sensor/camera/image')

# CRITICAL: PX4 launches the Gazebo server with GZ_IP=127.0.0.1 (see PX4's
# gz_bridge CMake rule). gz-transport discovery is multicast, so an external
# process WITHOUT GZ_IP set still SEES the topic (`gz topic -l` lists it) but
# never establishes the data connection -> zero messages, silently.
# Every process that subscribes to a gz topic must use the same GZ_IP.
GZ_ENV = {'GZ_IP': '127.0.0.1'}


def generate_launch_description():
    args = [
        DeclareLaunchArgument('cam_gz_topic', default_value=CAM_TOPIC),
        DeclareLaunchArgument('weights', default_value='yolov8n.pt'),
        DeclareLaunchArgument('conf', default_value='0.25'),
        # Comma-separated class NAMES to keep, e.g. 'person'. Empty = keep all.
        # COCO weights hallucinate airplane/kite/bird on featureless nadir
        # ground, so restrict this whenever you know what you placed.
        DeclareLaunchArgument('classes', default_value=''),
        DeclareLaunchArgument('gz_ip', default_value='127.0.0.1'),
        # --- geolocation quality ---
        DeclareLaunchArgument(
            'pose_lag_s', default_value='0.25',
            description='camera+bridge latency compensated when looking up the pose'),
        DeclareLaunchArgument('min_alt_m', default_value='1.0'),
        DeclareLaunchArgument('max_alt_m', default_value='40.0'),
        # --- gating ---
        DeclareLaunchArgument('gate_topic', default_value='/survey/detecting'),
        DeclareLaunchArgument(
            'require_gate', default_value='false',
            description='true = geotag ONLY while survey_node says it is flying lanes'),
    ]

    cam = LaunchConfiguration('cam_gz_topic')

    # Gazebo -> ROS 2 image bridge. No remap: the detector subscribes to the
    # same name the bridge publishes, which removes the earlier mismatch.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        arguments=[[cam, '@sensor_msgs/msg/Image[gz.msgs.Image']],
        additional_env=GZ_ENV,
    )

    detector = Node(
        package='perception',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[{
            'image_topic': cam,
            'weights': ParameterValue(LaunchConfiguration('weights'), value_type=str),
            'conf': ParameterValue(LaunchConfiguration('conf'), value_type=float),
            'classes': ParameterValue(LaunchConfiguration('classes'), value_type=str),
            'pose_lag_s': ParameterValue(LaunchConfiguration('pose_lag_s'), value_type=float),
            'min_alt_m': ParameterValue(LaunchConfiguration('min_alt_m'), value_type=float),
            'max_alt_m': ParameterValue(LaunchConfiguration('max_alt_m'), value_type=float),
            'gate_topic': ParameterValue(LaunchConfiguration('gate_topic'), value_type=str),
            'require_gate': ParameterValue(LaunchConfiguration('require_gate'), value_type=bool),
        }],
        additional_env=GZ_ENV,
    )

    return LaunchDescription(args + [bridge, detector])
