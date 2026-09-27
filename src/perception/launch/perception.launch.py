from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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


def _launch_setup(context, *args, **kwargs):
    """Deferred to launch time (via OpaqueFunction) because the number of
    camera bridges to start depends on the RESOLVED value of image_topics,
    which is only known once launch arguments are substituted - not while
    this file is merely being parsed.

    Single-drone (image_topics unset, the default): behaves exactly as
    before - one bridge on cam_gz_topic, one detector fed by cam_gz_topic.

    Swarm mode (image_topics set, ';'-separated): one bridge PER drone
    camera, and ONE detector fed the full plural param set (image_topics,
    pose_namespaces, gate_topics, annotated_topics, home_offsets) - see
    detector_node.py's module docstring for why one shared detector, not one
    per drone.
    """
    cam = LaunchConfiguration('cam_gz_topic').perform(context)
    image_topics_raw = LaunchConfiguration('image_topics').perform(context)
    cam_topics = [t.strip() for t in image_topics_raw.split(';') if t.strip()] or [cam]

    bridges = []
    for i, topic in enumerate(cam_topics):
        bridges.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge' if len(cam_topics) == 1 else f'camera_bridge_{i}',
            output='screen',
            arguments=[[topic, '@sensor_msgs/msg/Image[gz.msgs.Image']],
            additional_env=GZ_ENV,
        ))

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
            'imgsz': ParameterValue(LaunchConfiguration('imgsz'), value_type=int),
            'pose_lag_s': ParameterValue(LaunchConfiguration('pose_lag_s'), value_type=float),
            'min_alt_m': ParameterValue(LaunchConfiguration('min_alt_m'), value_type=float),
            'max_alt_m': ParameterValue(LaunchConfiguration('max_alt_m'), value_type=float),
            'gate_topic': ParameterValue(LaunchConfiguration('gate_topic'), value_type=str),
            'require_gate': ParameterValue(LaunchConfiguration('require_gate'), value_type=bool),
            'image_topics': ParameterValue(LaunchConfiguration('image_topics'), value_type=str),
            'pose_namespaces': ParameterValue(LaunchConfiguration('pose_namespaces'), value_type=str),
            'gate_topics': ParameterValue(LaunchConfiguration('gate_topics'), value_type=str),
            'annotated_topics': ParameterValue(LaunchConfiguration('annotated_topics'), value_type=str),
            'home_offsets': ParameterValue(LaunchConfiguration('home_offsets'), value_type=str),
        }],
        additional_env=GZ_ENV,
    )

    return bridges + [detector]


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
        # Inference size. Must match the camera's actual capture resolution
        # (1280x960, mono_cam SDF) or ultralytics silently downscales the
        # detail back out - see SWARM_PLAN.md and detector_node.py.
        DeclareLaunchArgument('imgsz', default_value='1280'),
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
        # --- multi-drone (swarm): ';'-separated, one entry per drone, same
        # order in all five (including cam_gz_topic's sibling image_topics
        # below). Leave image_topics empty for single-drone (the default) -
        # this file then behaves exactly as before: one bridge + one detector
        # driven by cam_gz_topic/gate_topic. See detector_node.py docstring.
        DeclareLaunchArgument(
            'image_topics', default_value='',
            description="';'-separated Gazebo camera topics, one per drone - "
                        "also determines how many bridges this file starts "
                        "(single-drone/default uses cam_gz_topic alone)"),
        DeclareLaunchArgument(
            'pose_namespaces', default_value='',
            description="';'-separated PX4 DDS namespaces, e.g. ';/px4_1' "
                        "(instance 0 = '', instance 1 = '/px4_1', ...)"),
        DeclareLaunchArgument('gate_topics', default_value=''),
        DeclareLaunchArgument('annotated_topics', default_value=''),
        DeclareLaunchArgument(
            'home_offsets', default_value='',
            description="';'-separated 'north,east' metres to fold each "
                        "drone's local NED frame into drone 0's home frame"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
