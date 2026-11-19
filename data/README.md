# Data

The recordings of the study are not in the git repository. Download them from
**[TODO: organization folder link]** and put them in this folder, so that it reads:

```
data/
  recordings/subject_XX_trial_Y.mcap       ROS 2 bag (MCAP, ROS 2 Jazzy)
  motion_capture/subject_XX_trial_Y.mot    reference joint angles: OpenSim inverse kinematics
  motion_capture/subject_XX_trial_Y.csv    Motive export: rigid bodies and markers, 240 Hz
  time_offsets.yaml                        aligns the motion capture with the recording
  SHA256SUMS                               checksums: `sha256sum -c SHA256SUMS` in this folder
```

There are 20 recordings: participants 1–10, trials A and B. Each holds the robot's data and the
output of the vision front end; camera images and point clouds are not included.

To replay one through the estimator (see the main README), from this folder:

```bash
ros2 launch arm_pose_estimation arm_pose_estimation.launch.py subject_id:=1 trial:=A \
    bag:=$PWD/recordings/subject_01_trial_A.mcap
```

## Topics

| Topic | Type | |
|---|---|---|
| `/lbr/joint_states` | `sensor_msgs/JointState` | KUKA LBR iiwa 7 joint positions and velocities, 1 kHz |
| `/lbr/robot_description` | `std_msgs/String` | the robot's URDF |
| `/tf_static`, `/tf` | `tf2_msgs/TFMessage` | the robot's links; the shoulder frame `upt_RightShoulder` (parent `camera_depth_optical_frame`) |
| `/arm_keypoints` | `geometry_msgs/PoseArray` | shoulder, elbow and wrist joint centres in `camera_depth_optical_frame` (positions only) |
| `/arm_keypoints_valid` | `std_msgs/Bool` | one per processed camera frame; `false`: the frame was rejected and has no keypoints |
| `/arm_keypoint_markers` | `visualization_msgs/MarkerArray` | the keypoints for RViz, with the elbow before its correction |

- The keypoints come from the package's vision front end (MediaPipe 0.10.21 and the 3D solver) run
  on the recorded camera. The camera pose in the robot base frame is in the package's
  `config/camera_extrinsic.yaml`.
- The robot's meshes come from `lbr_description`
  ([lbr_fri_ros2_stack](https://github.com/lbr-stack/lbr_fri_ros2_stack)); install it to see the
  robot in RViz.
- The session dates are removed: every recording starts at 1000 s. Time differences are unchanged.

## Motion capture

`time_offsets.yaml` gives, for each recording, the recording time of motion-capture time 0: a
`.mot` or `.csv` row at time `t` is at recording time `t + time_offset`.

The `.mot` columns that are the reference for the 7 estimated joints:

| `.mot` column | joint of the arm model | sign |
|---|---|---|
| `elv_angle` | `jRightShoulder_elv_angle` | + |
| `shoulder_elv` | `jRightShoulder_shoulder_elv` | + |
| `shoulder_rot` | `jRightShoulder_shoulder_rot` | + |
| `elbow_flexion` | `jRightElbow_rotz` | + |
| `pro_sup` | `jRightElbow_roty` | + |
| `flexion` | `jRightWrist_rotx` | + |
| `deviation` | `jRightWrist_rotz` | − |

- Units: degrees (`inDegrees=yes`), **except `flexion` and `deviation`, which are in radians.**
- Match columns by name: `deviation` comes before `flexion`.
- Not every row is a valid reference. In the paper, rows were not scored when they were one-frame
  IK flicks (more than 3° off a 37 ms running median) or steps over 5° per row; segments shorter
  than 0.25 s and 50 ms at each segment end were dropped as well.
