#pragma once

// The robot holding the hand: end-effector pose and twist from its joint
// states. Only the URDF text is needed, so any arm works.

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <Eigen/Dense>

#include <string>

namespace arm_pose_estimation::ekf
{

struct EndEffectorState
{
    pinocchio::SE3 pose;                  // in the robot base frame
    Eigen::Matrix<double, 6, 1> twist;    // [v; w], LOCAL_WORLD_ALIGNED, at the EE origin
};

class RobotKinematics
{
public:
    // Throws if `ee_frame` is not a frame of the URDF.
    RobotKinematics(const std::string& urdf_xml, const std::string& ee_frame);

    EndEffectorState endEffector(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

    int nq() const { return model_.nq; }
    int nv() const { return model_.nv; }
    int njoints() const { return model_.njoints; }
    const std::string& name() const { return model_.name; }

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_frame_ = 0;
};

}  // namespace arm_pose_estimation::ekf
