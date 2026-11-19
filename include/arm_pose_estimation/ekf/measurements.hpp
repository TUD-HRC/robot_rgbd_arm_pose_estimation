#pragma once

// Measurement models: each function returns one LinearizedMeasurement
// (residual r = z - h(x), Jacobian H, noise R) at the arm's current
// kinematics. Pure functions, no ROS. Everything is in the shoulder frame,
// the root of the arm model.

#include "arm_pose_estimation/ekf/arm_kinematics.hpp"
#include "arm_pose_estimation/ekf/joint_state_ekf.hpp"

#include <pinocchio/spatial/se3.hpp>

#include <optional>
#include <string>

namespace arm_pose_estimation::ekf
{

using Vector6 = Eigen::Matrix<double, 6, 1>;

// The robot-measured hand: pose and twist [v; w] in the shoulder frame.
struct HandObservation
{
    pinocchio::SE3 pose;
    Vector6 twist;
};

// The hand held by the robot end effector, in the shoulder frame:
// T_shoulder_hand = T_world_shoulder^-1 T_world_ee T_ee_hand, with the EE
// twist carried to the hand point and rotated into the shoulder frame.
HandObservation handFromEndEffector(
    const pinocchio::SE3& T_world_ee, const Vector6& ee_twist_world,
    const pinocchio::SE3& T_world_shoulder, const pinocchio::SE3& T_ee_hand);

// r = p_measured - p_hand(q),  H = [J_v, 0],  R = sigma^2 I.
LinearizedMeasurement handPosition(
    ArmKinematics& arm, const Eigen::Vector3d& measured, double sigma);

// r = log3(R_measured R_hand(q)^T),  H = [J_w, 0],  R = sigma^2 I.
LinearizedMeasurement handRotation(
    ArmKinematics& arm, const Eigen::Matrix3d& measured, double sigma);

// r = v_measured - J_hand(q) dq,  H = [0, J_hand],  R = diag(sigma_v^2 I, sigma_w^2 I).
LinearizedMeasurement handTwist(
    ArmKinematics& arm, const Vector6& measured, const JointVector& dq,
    double sigma_linear, double sigma_angular);

// Unit bone direction u = s/|s|, s = p_end - p_start:
//   r = u_measured - u(q),  H = [(I - u u^T)/|s| (J_end - J_start), 0],  R = sigma^2 I.
// nullopt (with `skip_reason` set) if the measured or the model bone is degenerate.
std::optional<LinearizedMeasurement> boneDirection(
    ArmKinematics& arm, const Bone& bone, const Eigen::Vector3d& measured_start,
    const Eigen::Vector3d& measured_end, double sigma, std::string& skip_reason);

// Self-motion damping: the hand-unobservable part of dq is zero.
//   r = -N^T dq,  H = [0, N^T],  R = sigma^2 I.
LinearizedMeasurement selfMotionDamping(
    const Eigen::MatrixXd& null_space_basis, const JointVector& dq, double sigma);

}  // namespace arm_pose_estimation::ekf
