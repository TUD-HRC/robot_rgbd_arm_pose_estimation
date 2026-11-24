# Data

This folder holds the study's dataset:

```
data/
  recordings/subject_XX_trial_Y.mcap       ROS 2 bag (MCAP, ROS 2 Jazzy)
  motion_capture/subject_XX_trial_Y.mot    reference joint angles: OpenSim inverse kinematics
  motion_capture/subject_XX_trial_Y.csv    Motive export: rigid bodies and markers, 240 Hz
  module_outputs/subject_XX_trial_Y_<module>.csv
                                           the paper's estimates, paired with the reference
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

## Module outputs

`module_outputs/` holds the estimates the paper evaluated: one file per recording and module, each
row paired with its motion-capture reference.

| `<module>` | |
|---|---|
| `fused` | the EKF with robot and camera measurements: the proposed method |
| `robot` | the EKF with robot measurements only |
| `camera` | the EKF with camera measurements only. It does not observe pronation–supination or the wrist (`jRightElbow_roty`, `jRightWrist_rotx`, `jRightWrist_rotz`); the paper does not score those for it |
| `ik` | the inverse-kinematics baseline: MoveIt 2's KDL plugin, solving the robot-measured hand pose of each `fused` row |

Columns: `time`, then for each of the 7 joints `gt_position.<joint>`, `est_position.<joint>`,
and after them `gt_velocity.<joint>`, `est_velocity.<joint>`.

- `time`: recording time in s, on the clock of the bags (from 1000 s). One row per estimate: every
  EKF cycle for `fused`, `robot` and `ik` (about 59 Hz), every camera frame for `camera` (15 Hz).
- Positions in degrees, velocities in degrees per second, joints as in the table above.
- `gt_*`: the nearest valid `.mot` row (no interpolation), with the sign and units of the table
  above. Velocities are the finite difference within each valid segment, then a zero-phase
  second-order Butterworth low-pass at 6 Hz. `nan`: no valid reference row.
- `est_*`: the module's output. The `ik` velocities are derived from its positions after the
  run, by the same difference and filter as the reference (a non-causal filter, which an online
  solver could not use).
- The paper takes the error as `est − gt`, positions wrapped to [−180°, 180°). Its statistics use
  every row that has a reference, from the start of each recording to its end. The dropout table
  (Table II) also needs the EKF's record of which cycles had a camera update, which is not
  included.

They come from replaying each original recording in real time, with the vision front end run
live on the camera images, and the package's EKF and configuration (each segment direction
corrects all seven joints through its full Jacobian; participant grips and segment lengths in
`config/subject_arm_lengths.yaml`). MediaPipe does not give identical results run to run, and
the keypoints in the bags come from a separate run of the front end, so replaying a bag gives
estimates close to these but not identical. The IK solver restarts from random seeds when it
fails, so a new solve can also differ.
