from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    float_defaults = {
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
        'reach_tol': '1.5',
        'return_tol': '3.0',
        'arm_timeout_s': '30.0',
        'return_timeout_s': '180.0',
    }
    bool_defaults = {
        'rtl_on_complete': 'true',
        'verify': 'true',
    }
    str_defaults = {
        'csv_dir': '~/maps',
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
