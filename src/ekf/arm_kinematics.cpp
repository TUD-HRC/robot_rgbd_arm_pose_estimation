#include "arm_pose_estimation/ekf/arm_kinematics.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <stdexcept>

namespace arm_pose_estimation::ekf
{
namespace
{

// The URDF joints behind the 7 EKF coordinates, without prefix.
constexpr std::array<const char*, kDof> kJointNames = {
    "jRightShoulder_elv_angle",
    "jRightShoulder_shoulder_elv",
    "jRightShoulder_shoulder_rot",
    "jRightElbow_rotz",
    "jRightElbow_roty",
    "jRightWrist_rotx",
    "jRightWrist_rotz",
};

}  // namespace

ArmKinematics::ArmKinematics(
    const std::string& urdf_xml, const std::string& prefix, const std::string& hand_frame)
{
    // mimic = true: pinocchio folds every <mimic> joint into its source joint, so the model's
    // configuration is just the 7 coordinates (see right_arm.urdf.xacro).
    pinocchio::urdf::buildModelFromXML(urdf_xml, model_, false, true);
    data_ = pinocchio::Data(model_);

    for (int i = 0; i < kDof; ++i) {
        joint_names_.push_back(prefix + kJointNames[i]);
        joint_ids_[i] = requireJoint(joint_names_[i]);
        const auto q_index = model_.idx_qs[joint_ids_[i]];
        limits_.q_lower[i] = model_.lowerPositionLimit[q_index];
        limits_.q_upper[i] = model_.upperPositionLimit[q_index];
        limits_.dq_max[i] = model_.velocityLimit[model_.idx_vs[joint_ids_[i]]];
    }

    const auto shoulder = requireFrame(prefix + "RightShoulder");
    const auto elbow = requireFrame(prefix + "RightForeArm");
    const auto wrist = requireFrame(prefix + "RightForeArm_f1");
    hand_frame_ = requireFrame(hand_frame);
    upper_arm_ = {"upper_arm", shoulder, elbow};
    forearm_ = {"forearm", elbow, wrist};
}

pinocchio::FrameIndex ArmKinematics::requireFrame(const std::string& name) const
{
    if (!model_.existFrame(name)) {
        throw std::runtime_error("Arm model has no frame '" + name + "'.");
    }
    return model_.getFrameId(name);
}

pinocchio::JointIndex ArmKinematics::requireJoint(const std::string& name) const
{
    if (!model_.existJointName(name)) {
        throw std::runtime_error("Arm model has no joint '" + name + "'.");
    }
    return model_.getJointId(name);
}

Eigen::VectorXd ArmKinematics::modelConfiguration(const JointVector& q) const
{
    Eigen::VectorXd q_model = Eigen::VectorXd::Zero(model_.nq);
    for (int i = 0; i < kDof; ++i) {
        q_model[model_.idx_qs[joint_ids_[i]]] = q[i];
    }
    return q_model;
}

Eigen::Matrix<double, 6, kDof> ArmKinematics::reduceJacobian(
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& J) const
{
    Eigen::Matrix<double, 6, kDof> reduced;
    for (int i = 0; i < kDof; ++i) {
        reduced.col(i) = J.col(model_.idx_vs[joint_ids_[i]]);
    }
    return reduced;
}

void ArmKinematics::update(const JointVector& q)
{
    const Eigen::VectorXd q_model = modelConfiguration(q);
    pinocchio::computeJointJacobians(model_, data_, q_model);
    pinocchio::framesForwardKinematics(model_, data_, q_model);
    pinocchio::updateFramePlacements(model_, data_);
}

const pinocchio::SE3& ArmKinematics::framePose(pinocchio::FrameIndex frame) const
{
    return data_.oMf[frame];
}

Eigen::Matrix<double, 3, kDof> ArmKinematics::frameLinearJacobian(pinocchio::FrameIndex frame)
{
    const Eigen::Matrix<double, 6, Eigen::Dynamic> J =
        pinocchio::getFrameJacobian(model_, data_, frame, pinocchio::LOCAL_WORLD_ALIGNED);
    return reduceJacobian(J).topRows<3>();
}

Eigen::Matrix<double, 6, kDof> ArmKinematics::handJacobian()
{
    return reduceJacobian(
        pinocchio::getFrameJacobian(model_, data_, hand_frame_, pinocchio::LOCAL_WORLD_ALIGNED));
}

SelfMotionBasis ArmKinematics::handSelfMotionBasis(double singular_value_ratio)
{
    // SVD J = U S V^T. J is 6x7, so V's 7th column has no singular value at all
    // and is always in the null space; any other column whose singular value
    // collapses joins it (near singularities the null space grows, and that is
    // exactly where holding it matters most). J mixes m/s and rad/s rows, so
    // the relative threshold is a heuristic.
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(handJacobian(), Eigen::ComputeFullV);
    const Eigen::VectorXd values = svd.singularValues();

    SelfMotionBasis result;
    result.smallest_singular_value = values(values.size() - 1);
    result.tolerance = std::max(1.0e-9, values(0) * std::max(0.0, singular_value_ratio));

    std::vector<int> columns;
    for (int i = 0; i < kDof; ++i) {
        const double value = i < values.size() ? values(i) : 0.0;
        if (value <= result.tolerance) {
            columns.push_back(i);
        }
    }
    result.basis.resize(kDof, static_cast<int>(columns.size()));
    for (std::size_t i = 0; i < columns.size(); ++i) {
        result.basis.col(static_cast<int>(i)) = svd.matrixV().col(columns[i]);
    }
    return result;
}

}  // namespace arm_pose_estimation::ekf
