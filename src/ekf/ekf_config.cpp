#include "arm_pose_estimation/ekf/ekf_config.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace arm_pose_estimation::ekf
{
namespace
{

SensorSet parseSensorSet(const std::string& text)
{
    if (text == "fused") return SensorSet::kFused;
    if (text == "robot") return SensorSet::kRobot;
    if (text == "camera") return SensorSet::kCamera;
    throw std::runtime_error("sensors is '" + text + "'; expected 'fused', 'robot' or 'camera'.");
}

// Grip offset: extrinsic X -> Y -> Z angles about the hand's own axes, so the
// hand rotation is R_ee_hand * Rz * Ry * Rx.
Eigen::Matrix3d gripRotation(const Eigen::Vector3d& angles)
{
    return (Eigen::AngleAxisd(angles.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(angles.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(angles.x(), Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

}  // namespace

const char* toString(SensorSet sensors)
{
    switch (sensors) {
        case SensorSet::kFused: return "fused";
        case SensorSet::kRobot: return "robot";
        case SensorSet::kCamera: return "camera";
    }
    return "?";
}

EkfConfig loadEkfConfig(rclcpp::Node& node)
{
    const auto text = [&node](const std::string& name) {
        return node.declare_parameter<std::string>(name);
    };
    const auto number = [&node](const std::string& name) {
        return node.declare_parameter<double>(name);
    };
    const auto vector = [&node](const std::string& name, std::size_t size) {
        const auto values = node.declare_parameter<std::vector<double>>(name);
        if (values.size() != size) {
            throw std::runtime_error(name + " needs " + std::to_string(size) + " values, got " +
                                     std::to_string(values.size()) + ".");
        }
        return Eigen::VectorXd(Eigen::VectorXd::Map(values.data(), values.size()));
    };

    EkfConfig c;
    c.sensors = parseSensorSet(text("sensors"));

    c.arm_description = text("arm_description");
    c.arm_prefix = text("frames.arm_prefix");
    c.robot_description = text("robot_description");
    c.robot_description_topic = text("topics.robot_description");
    const Eigen::VectorXd translation = vector("ee_to_hand.translation", 3);
    const Eigen::VectorXd xyzw = vector("ee_to_hand.rotation", 4);
    const Eigen::Vector3d grip = vector("grip_offset_deg", 3) * (M_PI / 180.0);
    c.ee_to_hand = pinocchio::SE3(
        Eigen::Quaterniond(xyzw[3], xyzw[0], xyzw[1], xyzw[2]).normalized().toRotationMatrix() *
            gripRotation(grip),
        translation);

    c.world_frame = text("frames.world");
    c.robot_ee_frame = text("frames.robot_ee");
    c.shoulder_frame = c.arm_prefix + "RightShoulder";
    c.hand_frame = c.arm_prefix + "RightHandCOM";
    c.robot_joint_topic = text("topics.robot_joint_states");
    c.keypoints_topic = text("topics.arm_keypoints");
    c.keypoints_valid_topic = text("topics.arm_keypoints_valid");
    c.output_topic = text("topics.ekf_joint_states");
    c.cycle_timing_topic = text("topics.ekf_cycle_timing");

    c.initial_q = vector("state.initial_q", kDof);
    c.initial_dq = vector("state.initial_dq", kDof);
    c.initial_q_variance = number("state.initial_q_variance");
    c.initial_dq_variance = number("state.initial_dq_variance");
    c.acceleration_variance = number("process_noise.acceleration_variance");
    c.max_update_rate_hz = number("max_update_rate_hz");
    c.tf_lookup_timeout_sec = number("tf_lookup_timeout_sec");

    c.hand_position_sigma = number("measurements.hand_pose.sigma_position");
    c.hand_rotation_sigma = number("measurements.hand_pose.sigma_rotation");
    c.hand_linear_velocity_sigma = number("measurements.hand_twist.sigma_linear_velocity");
    c.hand_angular_velocity_sigma = number("measurements.hand_twist.sigma_angular_velocity");
    c.keypoint_direction_sigma = number("measurements.arm_keypoints.direction_sigma");
    c.keypoint_max_age_sec = number("measurements.arm_keypoints.max_age_sec");
    c.self_motion_sigma = number("measurements.self_motion_damping.sigma");
    c.self_motion_singular_value_ratio =
        number("measurements.self_motion_damping.singular_value_ratio");
    return c;
}

}  // namespace arm_pose_estimation::ekf
