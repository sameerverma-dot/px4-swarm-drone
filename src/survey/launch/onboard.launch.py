"""
onboard.launch.py - everything ONE drone's onboard computer runs.

    camera_bridge_<i>   the camera driver (in sim: Gazebo -> ROS 2)
    detector_node_<i>   YOLO on this drone's camera only, geotags into the
                        shared frame, logs to the run dir, shares hazards with
                        peers over /swarm/hazards
    survey_node_<i>     flies this drone's band, heartbeats to peers, takes over
                        unfinished bands, yields on separation, returns home

Nothing here subscribes to a ground station. swarm_mission.launch.py includes
this once per drone; on real hardware it is what each Pi starts at boot.

    ros2 launch survey onboard.launch.py drone_id:=1 num_drones:=3 y_max:=90.0
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GZ_ENV = {'GZ_IP': '127.0.0.1'}   # see perception.launch.py: data needs the same GZ_IP

ARGS = {
    # identity + band math (must match tools/start_px4_swarm.sh)
    'drone_id': '0', 'num_drones': '3',
    'x_min': '0.0', 'x_max': '30.0', 'y_min': '0.0', 'y_max': '90.0',
    'world': 'default', 'model': 'x500_mono_cam_down',
    'run_dir': '~/maps',
    # flight
    'altitude': '10.0', 'lane_spacing': '0.0', 'sidelap': '0.3',
    'yaw_mode': 'course', 'fixed_yaw_deg': '0.0', 'yaw_deadzone_m': '1.0',
    'lookahead_m': '4.0', 'rtl_on_complete': 'true', 'survey_delay': '8.0',
    # detection
    'weights': 'yolov8n.pt', 'conf': '0.65', 'classes': 'person', 'imgsz': '1280',
    'pose_lag_s': '0.25', 'max_alt_m': '40.0', 'min_sep_m': '4.5',
    # swarm
    'heartbeat_topic': '/swarm/heartbeat', 'hazard_topic': '/swarm/hazards',
    'takeover': 'true', 'separation': 'true',
    'sep_horizontal_m': '8.0', 'sep_vertical_m': '5.0', 'sep_climb_m': '5.0',
    'peer_timeout_s': '3.0', 'startup_grace_s': '45.0', 'deadline_margin_s': '15.0',
    'claim_wait_s': '20.0', 'max_hold_s': '120.0', 'transit_alt_offset_m': '3.0',
    'min_battery_pct': '20.0', 'min_takeover_battery': '30.0',
    # fault injection (testing only)
    'start_delay_s': '0.0', 'abort_after_lanes': '-1',
}


def _setup(context, *args, **kwargs):
    g = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    i, n = int(g['drone_id']), int(g['num_drones'])
    y_min, y_max = float(g['y_min']), float(g['y_max'])
    if not 0 <= i < n:
        raise RuntimeError(f"drone_id {i} outside 0..{n - 1}")
    east0 = y_min + i * (y_max - y_min) / n
    ns = '' if i == 0 else f'/px4_{i}'
    cam = f"/world/{g['world']}/model/{g['model']}_{i}/link/camera_link/sensor/camera/image"
    run_dir = os.path.expanduser(g['run_dir'])
    gate = f'/survey/detecting_{i}'
    f, b = float, (lambda s: s.lower() in ('1', 'true', 'yes'))

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  name=f'camera_bridge_{i}', output='screen',
                  arguments=[f'{cam}@sensor_msgs/msg/Image[gz.msgs.Image'],
                  additional_env=GZ_ENV)

    detector = Node(package='perception', executable='detector_node',
                    name=f'detector_node_{i}', output='screen', additional_env=GZ_ENV,
                    parameters=[{
                        'drone_id': i,
                        'swarm_hazard_topic': g['hazard_topic'],
                        'image_topics': cam, 'pose_namespaces': ns, 'gate_topics': gate,
                        'annotated_topics': f'/detection/image_annotated_{i}',
                        'home_offsets': f'0,{east0}',
                        'hazard_csv': os.path.join(run_dir, f'hazards_d{i}.csv'),
                        'weights': g['weights'], 'conf': f(g['conf']),
                        'classes': g['classes'], 'imgsz': int(g['imgsz']),
                        'pose_lag_s': f(g['pose_lag_s']), 'max_alt_m': f(g['max_alt_m']),
                        'min_sep_m': f(g['min_sep_m']), 'require_gate': True,
                    }])

    survey = Node(package='survey', executable='survey_node',
                  name=f'survey_node_{i}', output='screen',
                  parameters=[{
                      'drone_id': i, 'num_drones': n,
                      'swarm_y_min': y_min, 'swarm_y_max': y_max,
                      'x_min': f(g['x_min']), 'x_max': f(g['x_max']),
                      'namespace': ns, 'detect_topic': gate,
                      'csv_dir': run_dir, 'csv_prefix': f'survey_track_d{i}',
                      'altitude': f(g['altitude']), 'lane_spacing': f(g['lane_spacing']),
                      'sidelap': f(g['sidelap']), 'yaw_mode': g['yaw_mode'],
                      'fixed_yaw_deg': f(g['fixed_yaw_deg']),
                      'yaw_deadzone_m': f(g['yaw_deadzone_m']),
                      'lookahead_m': f(g['lookahead_m']),
                      'rtl_on_complete': b(g['rtl_on_complete']),
                      'heartbeat_topic': g['heartbeat_topic'],
                      'takeover': b(g['takeover']), 'separation': b(g['separation']),
                      'sep_horizontal_m': f(g['sep_horizontal_m']),
                      'sep_vertical_m': f(g['sep_vertical_m']),
                      'sep_climb_m': f(g['sep_climb_m']),
                      'peer_timeout_s': f(g['peer_timeout_s']),
                      'startup_grace_s': f(g['startup_grace_s']),
                      'deadline_margin_s': f(g['deadline_margin_s']),
                      'claim_wait_s': f(g['claim_wait_s']), 'max_hold_s': f(g['max_hold_s']),
                      'transit_alt_offset_m': f(g['transit_alt_offset_m']),
                      'min_battery_pct': f(g['min_battery_pct']),
                      'min_takeover_battery': f(g['min_takeover_battery']),
                      'start_delay_s': f(g['start_delay_s']),
                      'abort_after_lanes': int(g['abort_after_lanes']),
                  }])

    # Detector subscribed before the drone moves, so lane 1 is watched.
    return [bridge, detector, TimerAction(period=f(g['survey_delay']), actions=[survey])]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()]
        + [OpaqueFunction(function=_setup)])
