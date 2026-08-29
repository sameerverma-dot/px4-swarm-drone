from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Default gz camera topic for the x500_mono_cam_down model, world "default".
    # Confirm the exact name on the running sim with:  gz topic -l | grep -i image
    default_cam = ('/world/default/model/x500_mono_cam_down_0/'
                   'link/camera_link/sensor/camera/image')

    args = [
        DeclareLaunchArgument('cam_gz_topic', default_value=default_cam),
        DeclareLaunchArgument('image_topic', default_value='/drone/camera'),
        DeclareLaunchArgument('weights', default_value='yolov8n.pt'),
        DeclareLaunchArgument('conf', default_value='0.25'),
        DeclareLaunchArgument('classes', default_value=''),
    ]

    cam = LaunchConfiguration('cam_gz_topic')
    image_topic = LaunchConfiguration('image_topic')

    # Gazebo -> ROS 2 image bridge (gz.msgs.Image -> sensor_msgs/Image, read-only).
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        arguments=[[cam, '@sensor_msgs/msg/Image[gz.msgs.Image']],
        remappings=[(cam, image_topic)],
    )

    detector = Node(
        package='perception',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'weights': ParameterValue(LaunchConfiguration('weights'), value_type=str),
            'conf': ParameterValue(LaunchConfiguration('conf'), value_type=float),
            'classes': ParameterValue(LaunchConfiguration('classes'), value_type=str),
        }],
    )

    return LaunchDescription(args + [bridge, detector])
