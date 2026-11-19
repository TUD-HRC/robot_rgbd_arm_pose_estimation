"""The configuration the launch file reads before any node starts."""

import os
from typing import Dict, List

import yaml


PACKAGE_NAME = 'arm_pose_estimation'


def share_path(*parts) -> str:
    """Return a path inside this package's installed share directory."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(PACKAGE_NAME), *parts)


def _load(*parts) -> dict:
    with open(share_path(*parts), 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def general_parameters() -> dict:
    """Return the parameters in config/general.yaml."""
    return _load('config', 'general.yaml')['/**']['ros__parameters']


def subject_parameters(subject_id, trial: str) -> Dict:
    """
    Return one subject's arm lengths and grip offset (config/subject_arm_lengths.yaml).

    The grip offset is grip_offset_<trial> if the subject has one, else
    grip_offset, else zero (the universal grip in ekf.yaml).
    """
    key = f'subject_{int(subject_id):02d}'
    subject = (_load('config', 'subject_arm_lengths.yaml').get('subjects') or {}).get(key)
    if subject is None:
        raise RuntimeError(f'{key} is not in config/subject_arm_lengths.yaml.')
    candidates = ([f'grip_offset_{trial}'] if trial else []) + ['grip_offset']
    grip_key = next((name for name in candidates if name in subject), None)
    return {
        'upper_arm_length': float(subject['upper_arm_length']),
        'forearm_length': float(subject['forearm_length']),
        'hand_length': float(subject['hand_length']),
        'grip_offset_deg': [float(v) for v in subject[grip_key]] if grip_key else [0.0, 0.0, 0.0],
        'grip_offset_key': grip_key or 'none (universal grip)',
    }


def recorded_message(bag: str, topic: str):
    """Return the last message on `topic` in a recording (e.g. a latched robot description)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        raise RuntimeError(f'{bag} has no {topic}.')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    data = None
    while reader.has_next():
        _, data, _ = reader.read_next()
    return deserialize_message(data, get_message(types[topic]))


def camera_extrinsic_arguments() -> List[str]:
    """Return config/camera_extrinsic.yaml as static_transform_publisher arguments."""
    extrinsic = _load('config', 'camera_extrinsic.yaml')
    x, y, z = (str(v) for v in extrinsic['translation'])
    qx, qy, qz, qw = (str(v) for v in extrinsic['rotation'])
    return ['--x', x, '--y', y, '--z', z, '--qx', qx, '--qy', qy, '--qz', qz, '--qw', qw,
            '--frame-id', extrinsic['parent_frame'],
            '--child-frame-id', extrinsic['child_frame']]


def arm_description(prefix: str, color: str, subject: Dict):
    """Return the subject's arm URDF (right_arm.urdf.xacro) as a node parameter value."""
    from launch.substitutions import Command
    from launch_ros.parameter_descriptions import ParameterValue

    return ParameterValue(Command([
        'xacro ', share_path('urdf', 'right_arm.urdf.xacro'),
        ' robot_prefix:=', prefix, ' robot_color:=', color,
        ' upper_arm_length:=', str(subject['upper_arm_length']),
        ' forearm_length:=', str(subject['forearm_length']),
        ' hand_length:=', str(subject['hand_length'])]), value_type=str)
