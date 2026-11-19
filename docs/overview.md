# How the code works

## Nodes

```
camera ─► keypoint_extraction ─► swivel_solver ─► /arm_keypoints ──┐
          (MediaPipe, 2D)        (3D, point cloud)  + shoulder TF   ├─► arm_pose_ekf_node ─► /arm_pose_ekf/joint_states
robot  ─────────────────────────────────────────► /lbr/joint_states┘
```

- **`keypoint_extraction`** (`mediapipe_kp.py`) runs MediaPipe Pose on each RGB image and
  publishes the pixels of the right shoulder, elbow and wrist.
- **`swivel_solver`** (`arm_solver_swivel.py`) reads those pixels out of the organized point
  cloud, moves each point from the skin to the joint centre, rejects frames whose bone lengths
  do not fit the subject, rebuilds the elbow from the corrected shoulder angles, and publishes
  shoulder, elbow and wrist. It also broadcasts the shoulder frame the arm model hangs off:
  it follows the measured shoulder for the first 5 s, then stays fixed.
- **`arm_pose_ekf_node`** (`arm_pose_ekf_node.cpp`) estimates the arm's 7 joint angles and
  their velocities.

On a recording, the first two nodes are not run: the recording already holds their output.

## The filter

- **State:** `x = [q, dq]`, with `q` the seven joints of `urdf/right_arm.urdf.xacro`:
  elevation plane, elevation, shoulder rotation, elbow flexion, pro/supination, wrist flexion,
  wrist deviation. All quantities are in the shoulder frame.
- **Cycle:** each robot joint state (at most 60 Hz) runs predict → robot updates → camera
  updates → publish. In camera-only mode, each keypoint set runs the cycle instead.
- **Predict:** constant velocity with white-noise acceleration.
- **Robot updates:** the robot holds the hand, so its forward kinematics, the handle
  transform `ee_to_hand` and the subject's grip offset give the hand pose and twist. They
  update the hand position, rotation and twist of the arm model.
- **Camera updates:** the unit upper-arm and forearm directions. The latest keypoint set is
  applied on every cycle until a newer one arrives or it is older than 0.2 s.
- **Without vision:** the robot sees only 6 of the 7 DOF (the hand stays still while the elbow
  swivels). A pseudo-measurement then holds the swivel rate at zero (self-motion damping).
- **Joint limits:** the URDF limits are enforced after every step by truncating the Gaussian.

Each measurement is a pure function in `measurements.hpp` that returns the residual, its
Jacobian and its noise; every update is applied in sequence with a Joseph-form covariance update.

## Files

| Path | Content |
|---|---|
| `include/.../ekf/joint_state_ekf.hpp` | state, predict, correct, joint limits (no ROS) |
| `include/.../ekf/arm_kinematics.hpp` | the arm model: forward kinematics, Jacobians, null space (no ROS) |
| `include/.../ekf/robot_kinematics.hpp` | robot end-effector pose and twist (no ROS) |
| `include/.../ekf/measurements.hpp` | measurement models (no ROS) |
| `include/.../ekf/ekf_config.hpp` | the node's parameters |
| `src/arm_pose_ekf_node.cpp` | subscriptions, filter cycle, publishing |
| `arm_pose_estimation/*.py` | vision front end, launch helpers |
| `launch/arm_pose_estimation.launch.py` | the one launch file |
| `config/` | topics and frames, filter settings, subjects, camera extrinsic, RViz |
| `urdf/right_arm.urdf.xacro` | the human arm (OpenSim MoBL-ARMS kinematics), scaled per subject |
| `additional_resources/` | CAD of the robot handle |

## Timing records

For the real-time benchmark, the EKF publishes one record per cycle on
`/arm_pose_ekf/cycle_timing` and the solver one per keypoint set on `/arm_keypoints_timing`.
The fields are listed in `kCycleTimingFields` (`arm_pose_ekf_node.hpp`) and `TIMING_FIELDS`
(`arm_solver_swivel.py`). Times are real time only with an optimized build (the default).
