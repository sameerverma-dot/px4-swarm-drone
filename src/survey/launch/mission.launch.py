"""
mission.launch.py — the full Phase I loop in one command.

Runs the coverage survey AND the detection pipeline together:

    survey_node      -> flies a boustrophedon pattern over the area, RTLs,
                        and publishes /survey/detecting so the detector knows
                        when the drone is actually over the survey area
    camera_bridge    -> Gazebo camera  ->  ROS 2
    detector_node    -> YOLO -> annotated image + geotagged hazard CSV

Pre-req: the sim must already be running with the CAMERA drone:
    bash ~/px4_ros_ws/start_px4_sim.sh gz_x500_mono_cam_down

Then:
    ros2 launch survey mission.launch.py x_max:=30.0 y_max:=20.0 altitude:=5.0

Outputs land in ~/maps/ :
    survey_track_<ts>.csv   flown path (from the survey node's verification)
    hazard_points.csv       geotagged detections (from the detector)

DEFAULTS THAT CHANGED AFTER THE FIRST FULL CAMERA RUN
-----------------------------------------------------
  classes:='person'   COCO weights invent airplane/kite/bird on empty nadir
                      ground with high confidence. Restrict to what you
                      actually placed. Use classes:='' to see everything.
  conf:=0.40          was 0.25; the junk sat between 0.26 and 0.85.
  require_gate:=true  no geotagging during climb, RTL or landing. The old run
                      logged "hazards" at 29.7 m on the way home.
  lookahead_m:=4.0    caps ground speed ~3.8 m/s. The first run flew lanes at
                      9.2 m/s, which is what smeared the North coordinate.
                      Replaying that flight's data: 3.98 m RMS along-track
                      error -> 0.96 m with the cap plus pose_lag 0.25.
  lane_spacing:=0.0   derived from the camera footprint instead of guessed.
  yaw_mode:=course    the drone faces where it is going. Setpoints previously
                      carried the default yaw=0.0, which actively commands
                      "face North", so it crabbed sideways down every
                      south-bound lane. yaw_mode:=fixed restores that.

Useful overrides:
    weights:=/home/sam/runs/detect/train/weights/best.pt   # your trained model
    lane_spacing:=8.0  altitude:=15.0  conf:=0.35  classes:=''
    sidelap:=0.5                                            # denser coverage
    rtl_on_complete:=false                                  # stay airborne at the end
    lookahead_m:=0.0                                        # fly flat-out again
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    survey_share = get_package_share_directory('survey')
    perception_share = get_package_share_directory('perception')

    args = [
        # --- survey area (PX4 local NED metres, home = 0,0) ---
        DeclareLaunchArgument('x_min', default_value='0.0'),
        DeclareLaunchArgument('x_max', default_value='40.0'),
        DeclareLaunchArgument('y_min', default_value='0.0'),
        DeclareLaunchArgument('y_max', default_value='30.0'),
        DeclareLaunchArgument('altitude', default_value='15.0'),
        DeclareLaunchArgument(
            'lane_spacing', default_value='0.0',
            description='0.0 derives it from the camera footprint at this altitude'),
        DeclareLaunchArgument('sidelap', default_value='0.3'),
        DeclareLaunchArgument(
            'yaw_mode', default_value='course',
            description="course = face direction of travel; fixed = locked to "
                        "fixed_yaw_deg (0 = North, the old behaviour); hold = "
                        "don't command yaw"),
        DeclareLaunchArgument('rtl_on_complete', default_value='true'),
        DeclareLaunchArgument(
            'lookahead_m', default_value='4.0',
            description='ground-speed cap via setpoint lookahead; 0.0 = flat out'),
        # --- detection ---
        DeclareLaunchArgument('weights', default_value='yolov8n.pt'),
        DeclareLaunchArgument('conf', default_value='0.40'),
        DeclareLaunchArgument('classes', default_value='person'),
        DeclareLaunchArgument('pose_lag_s', default_value='0.25'),
        DeclareLaunchArgument('max_alt_m', default_value='40.0'),
        DeclareLaunchArgument('require_gate', default_value='true'),
        # --- timing ---
        DeclareLaunchArgument(
            'survey_delay', default_value='8.0',
            description='seconds to let the camera pipeline settle before flying'),
    ]

    # Perception first: bridge + YOLO need to be subscribed before the drone
    # starts moving, or the first lanes are flown with nothing watching.
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'perception.launch.py')),
        launch_arguments={
            'weights': LaunchConfiguration('weights'),
            'conf': LaunchConfiguration('conf'),
            'classes': LaunchConfiguration('classes'),
            'pose_lag_s': LaunchConfiguration('pose_lag_s'),
            'max_alt_m': LaunchConfiguration('max_alt_m'),
            'require_gate': LaunchConfiguration('require_gate'),
        }.items(),
    )

    survey = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(survey_share, 'launch', 'survey.launch.py')),
        launch_arguments={
            'x_min': LaunchConfiguration('x_min'),
            'x_max': LaunchConfiguration('x_max'),
            'y_min': LaunchConfiguration('y_min'),
            'y_max': LaunchConfiguration('y_max'),
            'altitude': LaunchConfiguration('altitude'),
            'lane_spacing': LaunchConfiguration('lane_spacing'),
            'sidelap': LaunchConfiguration('sidelap'),
            'yaw_mode': LaunchConfiguration('yaw_mode'),
            'rtl_on_complete': LaunchConfiguration('rtl_on_complete'),
            'lookahead_m': LaunchConfiguration('lookahead_m'),
        }.items(),
    )

    # Delay the flight so detection is already streaming when lane 1 begins.
    delayed_survey = TimerAction(
        period=LaunchConfiguration('survey_delay'),
        actions=[survey],
    )

    return LaunchDescription(args + [perception, delayed_survey])
