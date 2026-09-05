"""
survey.launch.py - run survey_node with every parameter exposed as a launch arg.

Every parameter the node declares is listed here. If you add a parameter to
survey_node.py, add it to the matching dict below or it cannot be set from a
launch file - and, worse, mission.launch.py silently cannot forward it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    float_defaults = {
        # --- survey area (PX4 local NED metres, home = 0,0) ---
        'x_min': '0.0',
        'x_max': '40.0',
        'y_min': '0.0',
        'y_max': '30.0',
        'altitude': '10.0',
        # 0.0 = derive from the camera footprint at the chosen altitude
        # (2*h*tan(HFOV/2) * (1 - sidelap)). A positive value overrides it.
        'lane_spacing': '0.0',
        'sidelap': '0.3',
        'hfov_rad': '1.74',
        # --- tolerances & timeouts ---
        'reach_tol': '1.5',
        'return_tol': '3.0',
        'arm_timeout_s': '30.0',
        'return_timeout_s': '180.0',
        # --- speed cap: ground speed ends up ~0.95 * this (m/s).
        # 0.0 flies flat out at MPC_XY_VEL_MAX, which is what smeared the
        # geotags on the first camera run (9.2 m/s).
        'lookahead_m': '4.0',
        # --- heading ---
        'fixed_yaw_deg': '0.0',      # only used when yaw_mode:=fixed
        'yaw_deadzone_m': '1.0',     # hold last yaw inside this radius
    }
    bool_defaults = {
        'rtl_on_complete': 'true',
        'verify': 'true',
    }
    str_defaults = {
        'csv_dir': '~/maps',
        # Bool topic telling the detector when frames are worth geotagging.
        'detect_topic': '/survey/detecting',
        # 'course' = face the direction of travel (default)
        # 'fixed'  = hold fixed_yaw_deg (0 = North - the old locked behaviour)
        # 'hold'   = don't command yaw at all
        'yaw_mode': 'course',
    }

    decls = [DeclareLaunchArgument(k, default_value=v)
             for k, v in {**float_defaults, **bool_defaults, **str_defaults}.items()]

    params = {}
    for k in float_defaults:
        params[k] = ParameterValue(LaunchConfiguration(k), value_type=float)
    for k in bool_defaults:
        params[k] = ParameterValue(LaunchConfiguration(k), value_type=bool)
    for k in str_defaults:
        params[k] = ParameterValue(LaunchConfiguration(k), value_type=str)

    node = Node(
        package='survey',
        executable='survey_node',
        name='survey_node',
        output='screen',
        parameters=[params],
    )
    return LaunchDescription(decls + [node])
