r"""
Run the arm pose estimation pipeline, live or on a recording.

Live (camera and robot drivers running):
    ros2 launch arm_pose_estimation arm_pose_estimation.launch.py subject_id:=1

Recording with precomputed keypoints (no camera data needed):
    ros2 launch arm_pose_estimation arm_pose_estimation.launch.py subject_id:=1 trial:=A \
        bag:=<recording>

Live, it starts MediaPipe (keypoint_extraction) and the 3D solver
(swivel_solver) on the camera. With bag:=, it plays the recording instead,
which already holds their output, and stops when the recording ends.
"""

from arm_pose_estimation.launch_config import (
    arm_description, camera_extrinsic_arguments, general_parameters, PACKAGE_NAME,
    recorded_message, share_path, subject_parameters)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction,
    RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Wall seconds between launch and the start of a recording, for the nodes to come up.
PLAYBACK_DELAY_SEC = 3.0


def _launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context).strip()

    bag = arg('bag')
    use_sim_time = bool(bag)
    subject = subject_parameters(arg('subject_id'), arg('trial'))
    prefix = general_parameters()['frames']['arm_prefix']
    general = share_path('config', 'general.yaml')
    description = arm_description(prefix, 'blue', subject)
    # On a recording, the EKF gets the robot URDF before playback starts, so it misses no data.
    robot_description = recorded_message(
        bag, general_parameters()['topics']['robot_description']).data if bag else ''

    actions = [LogInfo(msg=f'Subject {arg("subject_id")}: grip offset '
                           f'{subject["grip_offset_key"]} {subject["grip_offset_deg"]} deg, '
                           f'sensors={arg("sensors")}.')]
    if bag:
        playback = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag, '--clock', '--delay', str(PLAYBACK_DELAY_SEC)],
            name='recording', output='screen')
        actions += [playback, RegisterEventHandler(OnProcessExit(
            target_action=playback,
            on_exit=[EmitEvent(event=Shutdown(reason='recording finished'))]))]
    else:
        actions += [
            Node(package=PACKAGE_NAME, executable='keypoint_extraction',
                 name='keypoint_extraction_node', output='screen', parameters=[general]),
            Node(package=PACKAGE_NAME, executable='swivel_solver', name='swivel_solver_node',
                 output='screen',
                 parameters=[general, {
                     'subject.upper_arm_length': subject['upper_arm_length'],
                     'subject.forearm_length': subject['forearm_length']}]),
        ]

    return actions + [
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='camera_extrinsic_publisher', output='screen',
             parameters=[{'use_sim_time': use_sim_time}],
             arguments=camera_extrinsic_arguments()),
        Node(package=PACKAGE_NAME, executable='arm_pose_ekf_node', name='arm_pose_ekf',
             output='screen',
             parameters=[general, share_path('config', 'ekf.yaml'), {
                 'use_sim_time': use_sim_time,
                 'sensors': arg('sensors'),
                 'arm_description': description,
                 'robot_description': robot_description,
                 'grip_offset_deg': subject['grip_offset_deg']}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=PACKAGE_NAME, name='arm_state_publisher', output='screen',
             parameters=[{'robot_description': description, 'use_sim_time': use_sim_time}],
             remappings=[('joint_states', general_parameters()['topics']['ekf_joint_states'])]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'use_sim_time': use_sim_time}],
             arguments=['-d', share_path('config', 'ekf.rviz')]),
    ]


def generate_launch_description():
    """Declare the arguments; the pipeline is built in _launch_setup."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'subject_id', default_value='1',
            description='Subject in config/subject_arm_lengths.yaml (1 -> subject_01).'),
        DeclareLaunchArgument(
            'trial', default_value='',
            description='Trial of the recording (A, B): selects grip_offset_<trial>.'),
        DeclareLaunchArgument(
            'bag', default_value='',
            description='Recording to play. Empty: live, from the camera and robot drivers.'),
        DeclareLaunchArgument(
            'sensors', default_value='fused', choices=['fused', 'robot', 'camera'],
            description='What the EKF fuses: robot and camera, robot only, or camera only.'),
        DeclareLaunchArgument(
            'rviz', default_value='true', choices=['true', 'false'], description='Start RViz.'),
        OpaqueFunction(function=_launch_setup),
    ])
