"""
swarm_mission.launch.py — the Phase I loop, scaled to N drones.

Splits ONE survey area into N contiguous bands along EAST (PX4 local NED y),
one band per drone, and flies them concurrently. One shared detector node
watches all N cameras and owns ONE hazard list, so deduplication stays
correct instead of something to bolt on afterwards (SWARM_PLAN.md section 2,
NEXT_SESSION.md's ordered plan).

--------------------------------------------------------------------------
PRE-REQ: the sim must already be running with N PX4 instances 0..N-1,
spawned with THE SAME num_drones/y_min/y_max/model this launch file gets -
they must agree on the band math, because the sim (spawn pose) and this
launch file (survey area + detector home_offsets) compute it independently:

    bash ~/px4_ros_ws/tools/start_px4_swarm.sh --num-drones 2 \\
        --y-min 0.0 --y-max 60.0 gz_x500_mono_cam_down

Then verify the namespace is actually live BEFORE flying anything - this
project's single most expensive bug was assuming a topic name without
checking (see NEXT_SESSION.md):

    ros2 topic hz /px4_1/fmu/out/vehicle_local_position_v1 --qos-reliability best_effort

Then:

    ros2 launch survey swarm_mission.launch.py num_drones:=2 \\
        x_max:=30.0 y_min:=0.0 y_max:=60.0 altitude:=10.0
--------------------------------------------------------------------------

Band math (must match tools/start_px4_swarm.sh exactly):
    band_h    = (y_max - y_min) / num_drones
    drone i local frame origin = global (north=0, east = y_min + i*band_h)
      -> spawn pose offset (gz world x,y) = (y_min + i*band_h, 0)
         [gz world x -> PX4 East, gz world y -> PX4 North; see
         NEXT_SESSION.md section 1.3's calibration]
      -> each drone's OWN survey_node then flies the SAME local box every
         time: x=[x_min,x_max], y=[0, band_h] - only the sim spawn differs.
      -> detector home_offsets add (0, y_min + i*band_h) back on, so every
         drone's hazards land in ONE shared frame (drone 0's home).

Outputs land in ~/maps/ :
    survey_track_d<i>_<ts>.csv   each drone's flown path
    hazard_points.csv            ONE combined, deduplicated hazard list
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *args, **kwargs):
    survey_share = get_package_share_directory('survey')
    perception_share = get_package_share_directory('perception')

    g = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    num_drones = max(1, int(g('num_drones')))
    x_min, x_max = g('x_min'), g('x_max')
    y_min_g, y_max_g = float(g('y_min')), float(g('y_max'))
    world = g('world')
    model = g('model')
    band_h = (y_max_g - y_min_g) / num_drones

    cam_topics, pose_ns, gate_topics, offsets, annot_topics = [], [], [], [], []
    survey_includes = []
    for i in range(num_drones):
        east_offset = y_min_g + i * band_h
        ns = '' if i == 0 else f'/px4_{i}'
        gate_topic = f'/survey/detecting_{i}'
        cam_topic = f'/world/{world}/model/{model}_{i}/link/camera_link/sensor/camera/image'

        pose_ns.append(ns)
        gate_topics.append(gate_topic)
        cam_topics.append(cam_topic)
        offsets.append(f'0,{east_offset}')
        # One annotated stream PER DRONE. Without this the detector's
        # annotated_topics falls back to the singular default, which _split()
        # broadcasts to every drone - so all N publish onto
        # /detection/image_annotated and rqt_image_view shows their frames
        # interleaved, flickering between viewpoints with no way to tell which
        # drone saw what. Harmless to the hazard CSV, ruinous in a demo.
        annot_topics.append(f'/detection/image_annotated_{i}')

        survey = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(survey_share, 'launch', 'survey.launch.py')),
            launch_arguments={
                'node_name': f'survey_node_{i}',
                'namespace': ns,
                'csv_prefix': f'survey_track_d{i}',
                'detect_topic': gate_topic,
                'x_min': x_min, 'x_max': x_max,
                'y_min': '0.0', 'y_max': f'{band_h}',
                'altitude': g('altitude'),
                'lane_spacing': g('lane_spacing'),
                'sidelap': g('sidelap'),
                'yaw_mode': g('yaw_mode'),
                'rtl_on_complete': g('rtl_on_complete'),
                'lookahead_m': g('lookahead_m'),
            }.items(),
        )
        survey_includes.append(survey)

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'perception.launch.py')),
        launch_arguments={
            'weights': g('weights'),
            'conf': g('conf'),
            'classes': g('classes'),
            'imgsz': g('imgsz'),
            'pose_lag_s': g('pose_lag_s'),
            'max_alt_m': g('max_alt_m'),
            'require_gate': g('require_gate'),
            'image_topics': ';'.join(cam_topics),
            'pose_namespaces': ';'.join(pose_ns),
            'gate_topics': ';'.join(gate_topics),
            'annotated_topics': ';'.join(annot_topics),
            'home_offsets': ';'.join(offsets),
        }.items(),
    )

    # Perception first (bridges + shared detector must be subscribed before
    # any drone starts flying, same reasoning as mission.launch.py), all N
    # survey nodes delayed together so their lanes are watched from lane 1.
    delayed_survey = TimerAction(period=g('survey_delay'), actions=survey_includes)

    return [perception, delayed_survey]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'num_drones', default_value='2',
            description='PX4 instances 0..num_drones-1 must already be running '
                        '(tools/start_px4_swarm.sh) with the SAME num_drones/'
                        'y_min/y_max/model as this launch file'),
        DeclareLaunchArgument('world', default_value='default'),
        DeclareLaunchArgument(
            'model', default_value='x500_mono_cam_down',
            description="gz model name WITHOUT the 'gz_' airframe prefix"),
        # --- total survey area, split along EAST (y) into num_drones bands ---
        DeclareLaunchArgument('x_min', default_value='0.0'),
        DeclareLaunchArgument('x_max', default_value='30.0'),
        DeclareLaunchArgument('y_min', default_value='0.0'),
        DeclareLaunchArgument(
            'y_max', default_value='60.0',
            description='enlarge this (or shrink num_drones) if drones finish '
                        'together with nothing to divide - SWARM_PLAN.md section 4'),
        DeclareLaunchArgument('altitude', default_value='10.0'),
        DeclareLaunchArgument('lane_spacing', default_value='0.0'),
        DeclareLaunchArgument('sidelap', default_value='0.3'),
        DeclareLaunchArgument('yaw_mode', default_value='course'),
        DeclareLaunchArgument('rtl_on_complete', default_value='true'),
        DeclareLaunchArgument('lookahead_m', default_value='4.0'),
        # --- detection (one shared detector for all drones) ---
        DeclareLaunchArgument('weights', default_value='yolov8n.pt'),
        DeclareLaunchArgument('conf', default_value='0.65'),
        DeclareLaunchArgument('classes', default_value='person'),
        DeclareLaunchArgument('imgsz', default_value='1280'),
        DeclareLaunchArgument('pose_lag_s', default_value='0.25'),
        DeclareLaunchArgument('max_alt_m', default_value='40.0'),
        DeclareLaunchArgument('require_gate', default_value='true'),
        # --- timing ---
        DeclareLaunchArgument('survey_delay', default_value='8.0'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
