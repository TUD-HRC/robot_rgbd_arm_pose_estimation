#include "arm_pose_estimation/ekf/robot_kinematics.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <stdexcept>

namespace arm_pose_estimation::ekf
{

RobotKinematics::RobotKinematics(const std::string& urdf_xml, const std::string& ee_frame)
{
    pinocchio::urdf::buildModelFromXML(urdf_xml, model_);
    data_ = pinocchio::Data(model_);
    // Pinocchio returns nframes for an unknown name, which would silently track
    // the wrong link.
    if (!model_.existFrame(ee_frame)) {
        throw std::runtime_error(
            "frames.robot_ee is '" + ee_frame + "', which the robot URDF does not define. "
            "Set it to the link the human hand grips.");
    }
    ee_frame_ = model_.getFrameId(ee_frame);
}

EndEffectorState RobotKinematics::endEffector(const Eigen::VectorXd& q, const Eigen::VectorXd& dq)
{
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::framesForwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    const Eigen::Matrix<double, 6, Eigen::Dynamic> J =
        pinocchio::getFrameJacobian(model_, data_, ee_frame_, pinocchio::LOCAL_WORLD_ALIGNED);
    return {data_.oMf[ee_frame_], J * dq};
}

}  // namespace arm_pose_estimation::ekf
