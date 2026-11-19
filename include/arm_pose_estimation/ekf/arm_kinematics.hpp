#pragma once

// The human arm model the EKF state lives on: forward kinematics and
// Jacobians of the subject-scaled URDF, expressed in the 7 EKF coordinates.

#include "arm_pose_estimation/ekf/joint_state_ekf.hpp"

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <Eigen/Dense>

#include <array>
#include <string>
#include <vector>

namespace arm_pose_estimation::ekf
{

// Orthonormal basis of the joint-rate directions the hand cannot see, with the
// numbers behind the rank decision (for diagnostics).
struct SelfMotionBasis
{
    Eigen::MatrixXd basis;  // kDof x m; m == 1 away from singularities
    double smallest_singular_value = 0.0;
    double tolerance = 0.0;
};

// A bone the camera measures as a direction: start and end frame of the model.
struct Bone
{
    std::string name;
    pinocchio::FrameIndex start_frame;
    pinocchio::FrameIndex end_frame;
};

class ArmKinematics
{
public:
    // `urdf_xml` is the rendered right_arm.urdf.xacro; `prefix` its joint/frame
    // prefix (e.g. "upt_"); `hand_frame` the frame the robot holds.
    ArmKinematics(const std::string& urdf_xml, const std::string& prefix,
                  const std::string& hand_frame);

    // Forward kinematics and joint Jacobians at q. Every query below refers to
    // the q of the most recent update().
    void update(const JointVector& q);

    const pinocchio::SE3& framePose(pinocchio::FrameIndex frame) const;
    const pinocchio::SE3& handPose() const { return framePose(hand_frame_); }
    // Jacobians in the root (shoulder) frame orientation, LOCAL_WORLD_ALIGNED,
    // reduced to the 7 EKF coordinates.
    Eigen::Matrix<double, 3, kDof> frameLinearJacobian(pinocchio::FrameIndex frame);
    Eigen::Matrix<double, 6, kDof> handJacobian();
    // Null space of the hand Jacobian: the elbow swivel, plus any direction
    // whose singular value falls below ratio * largest.
    SelfMotionBasis handSelfMotionBasis(double singular_value_ratio);

    // Upper arm (shoulder -> elbow) and forearm (elbow -> wrist).
    const Bone& upperArm() const { return upper_arm_; }
    const Bone& forearm() const { return forearm_; }

    const JointLimits& limits() const { return limits_; }
    const std::vector<std::string>& jointNames() const { return joint_names_; }

private:
    Eigen::VectorXd modelConfiguration(const JointVector& q) const;
    Eigen::Matrix<double, 6, kDof> reduceJacobian(const Eigen::Matrix<double, 6, Eigen::Dynamic>& J) const;
    pinocchio::FrameIndex requireFrame(const std::string& name) const;
    pinocchio::JointIndex requireJoint(const std::string& name) const;

    pinocchio::Model model_;
    pinocchio::Data data_;
    std::vector<std::string> joint_names_;
    std::array<pinocchio::JointIndex, kDof> joint_ids_{};
    pinocchio::FrameIndex hand_frame_ = 0;
    Bone upper_arm_;
    Bone forearm_;
    JointLimits limits_;
};

}  // namespace arm_pose_estimation::ekf
