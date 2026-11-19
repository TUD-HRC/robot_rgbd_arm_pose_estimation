#pragma once

// The parameters of arm_pose_ekf_node. They have no defaults in code: the
// values come from config/general.yaml, config/ekf.yaml and the launch file.

#include "arm_pose_estimation/ekf/joint_state_ekf.hpp"

#include <rclcpp/rclcpp.hpp>

#include <pinocchio/spatial/se3.hpp>

#include <string>

namespace arm_pose_estimation::ekf
{

// Which sensors feed the filter (launch argument sensors:=).
enum class SensorSet
{
    kFused,   // robot hand pose/twist + camera bone directions
    kRobot,   // robot only
    kCamera,  // camera only; cycles are driven by the keypoints
};

const char* toString(SensorSet sensors);

struct EkfConfig
{
    SensorSet sensors;
    bool usesRobot() const { return sensors != SensorSet::kCamera; }
    bool usesCamera() const { return sensors != SensorSet::kRobot; }

    // --- models -------------------------------------------------------------
    // The subject's arm: right_arm.urdf.xacro rendered by the launch.
    std::string arm_description;
    std::string arm_prefix;
    // The robot's URDF: given (read from the recording by the launch), or
    // empty to read it from its latched topic.
    std::string robot_description;
    std::string robot_description_topic;
    // Robot end effector -> hand, with the subject's grip offset applied.
    pinocchio::SE3 ee_to_hand;

    // --- frames and topics --------------------------------------------------
    std::string world_frame;
    std::string robot_ee_frame;
    std::string shoulder_frame;  // arm_prefix + RightShoulder
    std::string hand_frame;      // arm_prefix + RightHandCOM
    std::string robot_joint_topic;
    std::string keypoints_topic;
    std::string keypoints_valid_topic;
    std::string output_topic;
    std::string cycle_timing_topic;

    // --- state, process, cycle ----------------------------------------------
    JointVector initial_q;
    JointVector initial_dq;
    double initial_q_variance;
    double initial_dq_variance;
    double acceleration_variance;
    double max_update_rate_hz;
    double tf_lookup_timeout_sec;

    // --- measurement noise --------------------------------------------------
    double hand_position_sigma;
    double hand_rotation_sigma;
    double hand_linear_velocity_sigma;
    double hand_angular_velocity_sigma;
    double keypoint_direction_sigma;
    double keypoint_max_age_sec;
    double self_motion_sigma;
    double self_motion_singular_value_ratio;
};

// Declare every parameter on `node` and read it. Throws if one is missing or
// cannot be interpreted.
EkfConfig loadEkfConfig(rclcpp::Node& node);

}  // namespace arm_pose_estimation::ekf
