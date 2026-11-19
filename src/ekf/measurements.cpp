#include "arm_pose_estimation/ekf/measurements.hpp"

#include <pinocchio/spatial/explog.hpp>

#include <algorithm>

namespace arm_pose_estimation::ekf
{

HandObservation handFromEndEffector(
    const pinocchio::SE3& T_world_ee, const Vector6& ee_twist_world,
    const pinocchio::SE3& T_world_shoulder, const pinocchio::SE3& T_ee_hand)
{
    // Composed from live robot FK, not looked up as a shoulder->EE TF chain:
    // tf2 resolves a chain at the latest common time, so a stalled shoulder
    // frame would freeze the robot side too.
    HandObservation hand;
    hand.pose = T_world_shoulder.inverse() * T_world_ee * T_ee_hand;

    const Eigen::Vector3d ee_to_hand_world = T_world_ee.rotation() * T_ee_hand.translation();
    Vector6 twist_world;
    twist_world.head<3>() =
        ee_twist_world.head<3>() + ee_twist_world.tail<3>().cross(ee_to_hand_world);
    twist_world.tail<3>() = ee_twist_world.tail<3>();

    const Eigen::Matrix3d R_shoulder_world = T_world_shoulder.rotation().transpose();
    hand.twist.head<3>() = R_shoulder_world * twist_world.head<3>();
    hand.twist.tail<3>() = R_shoulder_world * twist_world.tail<3>();
    return hand;
}

LinearizedMeasurement handPosition(
    ArmKinematics& arm, const Eigen::Vector3d& measured, double sigma)
{
    LinearizedMeasurement m;
    m.name = "hand position";
    m.residual = measured - arm.handPose().translation();
    m.H = Eigen::MatrixXd::Zero(3, kStateSize);
    m.H.block(0, 0, 3, kDof) = arm.handJacobian().topRows<3>();
    m.R = Eigen::Matrix3d::Identity() * sigma * sigma;
    return m;
}

LinearizedMeasurement handRotation(
    ArmKinematics& arm, const Eigen::Matrix3d& measured, double sigma)
{
    // Residual is left/world-aligned to match the LOCAL_WORLD_ALIGNED Jacobian.
    LinearizedMeasurement m;
    m.name = "hand rotation";
    m.residual = pinocchio::log3(measured * arm.handPose().rotation().transpose());
    m.H = Eigen::MatrixXd::Zero(3, kStateSize);
    m.H.block(0, 0, 3, kDof) = arm.handJacobian().bottomRows<3>();
    m.R = Eigen::Matrix3d::Identity() * sigma * sigma;
    return m;
}

LinearizedMeasurement handTwist(
    ArmKinematics& arm, const Vector6& measured, const JointVector& dq,
    double sigma_linear, double sigma_angular)
{
    const Eigen::Matrix<double, 6, kDof> J = arm.handJacobian();
    LinearizedMeasurement m;
    m.name = "hand twist";
    m.residual = measured - J * dq;
    m.H = Eigen::MatrixXd::Zero(6, kStateSize);
    m.H.block(0, kDof, 6, kDof) = J;
    m.R = Eigen::MatrixXd::Zero(6, 6);
    m.R.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * (sigma_linear * sigma_linear);
    m.R.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() * (sigma_angular * sigma_angular);
    return m;
}

std::optional<LinearizedMeasurement> boneDirection(
    ArmKinematics& arm, const Bone& bone, const Eigen::Vector3d& measured_start,
    const Eigen::Vector3d& measured_end, double sigma, std::string& skip_reason)
{
    const Eigen::Vector3d measured_bone = measured_end - measured_start;
    const double measured_length = measured_bone.norm();
    if (measured_length < 1.0e-9) {
        skip_reason = "the measured " + bone.name + " is degenerate";
        return std::nullopt;
    }
    const Eigen::Vector3d model_bone =
        arm.framePose(bone.end_frame).translation() - arm.framePose(bone.start_frame).translation();
    const double model_length = model_bone.norm();
    if (model_length < 1.0e-9) {
        skip_reason = "the model " + bone.name + " is too short";
        return std::nullopt;
    }
    const Eigen::Vector3d u = model_bone / model_length;

    // d(s/|s|)/dq = (I - u u^T)/|s| * ds/dq
    const Eigen::Matrix3d normalization =
        (Eigen::Matrix3d::Identity() - u * u.transpose()) / model_length;

    LinearizedMeasurement m;
    m.name = bone.name + " direction";
    m.residual = measured_bone / measured_length - u;
    m.H = Eigen::MatrixXd::Zero(3, kStateSize);
    m.H.leftCols(kDof) = normalization * (arm.frameLinearJacobian(bone.end_frame) -
                                          arm.frameLinearJacobian(bone.start_frame));
    const double s = std::max(1.0e-6, sigma);
    m.R = Eigen::Matrix3d::Identity() * (s * s);
    return m;
}

LinearizedMeasurement selfMotionDamping(
    const Eigen::MatrixXd& null_space_basis, const JointVector& dq, double sigma)
{
    // The d(N^T dq)/dq term is dropped: second order in the joint rates, and
    // differentiating an SVD basis is fragile exactly near singularities.
    const auto rows = null_space_basis.cols();
    LinearizedMeasurement m;
    m.name = "self-motion damping";
    m.residual = -(null_space_basis.transpose() * dq);
    m.H = Eigen::MatrixXd::Zero(rows, kStateSize);
    m.H.block(0, kDof, rows, kDof) = null_space_basis.transpose();
    const double s = std::max(1.0e-9, sigma);
    m.R = Eigen::MatrixXd::Identity(rows, rows) * (s * s);
    return m;
}

}  // namespace arm_pose_estimation::ekf
