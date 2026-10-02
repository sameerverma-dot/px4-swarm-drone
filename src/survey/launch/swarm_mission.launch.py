"""
swarm_mission.launch.py - the decentralised N-drone survey (default: 3 drones).

Starts onboard.launch.py once per drone - each drone runs its OWN camera
bridge, detector and survey node, the way each onboard computer would - plus
an optional, passive ground station. Drones coordinate only drone-to-drone:

    /swarm/heartbeat   position, state, lanes, ETA, battery     (survey_node)
    /swarm/hazards     geotagged detections, shared list        (detector_node)

Capabilities (see survey_node.py / swarm_logic.py / hazard_registry.py):
  1. onboard autonomy  - no node on a drone listens to the ground; kill the
                         ground station mid-flight and nothing changes
  2. band takeover     - a drone that returns early, or goes silent past its
                         own projected finish, has its lanes flown by the
                         nearest available peer
  3. shared hazards    - each drone de-duplicates against its peers' finds,
                         e.g. an object on a band boundary is logged once
  4. separation        - a drone holds and moves vertically away if a peer gets
                         within sep_horizontal_m / sep_vertical_m

--------------------------------------------------------------------------
    bash ~/px4_ros_ws/tools/start_px4_swarm.sh --num-drones 3 --y-min 0 --y-max 90
    bash ~/px4_ros_ws/tools/check_system.sh
    NUM_DRONES=3 Y_MIN=0 Y_MAX=90 bash ~/px4_ros_ws/tools/add_swarm_targets.sh
    ros2 launch survey swarm_mission.launch.py num_drones:=3 y_min:=0.0 y_max:=90.0
    ros2 run perception hazard_map            # renders the newest run directory
--------------------------------------------------------------------------

Everything a run writes goes in ONE directory, ~/maps/swarm_<YYYYmmdd_HHMMSS>/:
    run.json                  the band math, so hazard_map needs no arguments
    survey_track_d<i>_<t>.csv each drone's flown path (its own local frame)
    hazards_d<i>.csv          each drone's onboard log: own + peer hazards
    ground_hazards.csv        the ground station's merged copy (if it ran)

Band math (must match tools/start_px4_swarm.sh): band_h = (y_max-y_min)/N,
drone i spawns at shared east y_min + i*band_h and flies band i.

Fault injection for testing, ';'-separated per drone:
    start_delay:="0;23;0"           drone 1 starts 23 s late
    abort_after_lanes:="-1;-1;1"    drone 2 returns early after 1 lane
"""

import importlib.util
import json
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _onboard_args():
    path = os.path.join(get_package_share_directory('survey'), 'launch', 'onboard.launch.py')
    spec = importlib.util.spec_from_file_location('onboard_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return path, mod.ARGS


def _per_drone(value, n, default):
    items = [s.strip() for s in value.split(';')] if value.strip() else []
    if items and len(items) != n:
        raise RuntimeError(f"expected {n} ';'-separated entries, got {value!r}")
    return items or [default] * n


def _setup(context, *args, **kwargs):
    onboard_path, onboard_args = _onboard_args()
    g = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    n = max(1, int(g('num_drones')))
    y_min, y_max = float(g('y_min')), float(g('y_max'))
    if y_max <= y_min:
        raise RuntimeError(f"y_max ({y_max}) must be greater than y_min ({y_min})")

    run_dir = g('run_dir').strip() or os.path.join(
        '~/maps', time.strftime('swarm_%Y%m%d_%H%M%S'))
    run_dir_abs = os.path.expanduser(run_dir)
    os.makedirs(run_dir_abs, exist_ok=True)
    with open(os.path.join(run_dir_abs, 'run.json'), 'w') as f:
        json.dump({'num_drones': n, 'x_min': float(g('x_min')), 'x_max': float(g('x_max')),
                   'y_min': y_min, 'y_max': y_max, 'altitude': float(g('altitude')),
                   'started': time.strftime('%Y-%m-%d %H:%M:%S')}, f, indent=2)
    print(f"[swarm_mission] run directory: {run_dir_abs}")

    delays = _per_drone(g('start_delay'), n, '0.0')
    aborts = _per_drone(g('abort_after_lanes'), n, '-1')

    actions = []
    for i in range(n):
        la = {k: g(k) for k in onboard_args if k not in ('drone_id', 'run_dir',
                                                          'start_delay_s', 'abort_after_lanes')}
        la.update({'drone_id': str(i), 'run_dir': run_dir_abs,
                   'start_delay_s': delays[i], 'abort_after_lanes': aborts[i]})
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(onboard_path), launch_arguments=la.items()))

    if g('ground_station').lower() in ('1', 'true', 'yes'):
        actions.append(Node(package='perception', executable='ground_station',
                            name='ground_station', output='screen',
                            parameters=[{'run_dir': run_dir_abs,
                                         'heartbeat_topic': g('heartbeat_topic'),
                                         'hazard_topic': g('hazard_topic'),
                                         'min_sep_m': float(g('min_sep_m'))}]))
    return actions


def generate_launch_description():
    _, onboard_args = _onboard_args()
    decls = [DeclareLaunchArgument(k, default_value=v) for k, v in onboard_args.items()
             if k not in ('drone_id', 'run_dir', 'start_delay_s', 'abort_after_lanes')]
    decls += [
        DeclareLaunchArgument('run_dir', default_value='',
                              description="'' = new ~/maps/swarm_<stamp>/ per run"),
        DeclareLaunchArgument('ground_station', default_value='true',
                              description='passive monitor; the mission does not need it'),
        DeclareLaunchArgument('start_delay', default_value='',
                              description="TEST: ';'-separated seconds per drone"),
        DeclareLaunchArgument('abort_after_lanes', default_value='',
                              description="TEST: ';'-separated, -1 = fly the whole band"),
    ]
    return LaunchDescription(decls + [OpaqueFunction(function=_setup)])
