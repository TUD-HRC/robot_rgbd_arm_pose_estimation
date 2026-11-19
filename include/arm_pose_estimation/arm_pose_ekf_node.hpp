#pragma once

// arm_pose_ekf: the ROS node around the EKF in ekf/.
//
// Estimates the 7-DOF right-arm joint state of a person holding the robot,
// fusing (depending on `sensors`) the robot-measured hand pose and twist with
// the upper-arm and forearm directions from the vision front end.
//
// One cycle: predict to the trigger stamp, apply the measurements, publish.
// The trigger is each robot joint state (fused, robot) or each keypoint set
// (camera). The other callbacks only cache their data. Every cycle also
// publishes a timing record (kCycleTimingFields).

#include "arm_pose_estimation/ekf/arm_kinematics.hpp"
#include "arm_pose_estimation/ekf/ekf_config.hpp"
#include "arm_pose_estimation/ekf/joint_state_ekf.hpp"
#include "arm_pose_estimation/ekf/robot_kinematics.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>

namespace arm_pose_estimation
{

// Fields of the timing record, in order. *_wall: Unix seconds on the system
// clock; *_stamp: sensor stamps; *_ms: steady-clock durations. NaN: not applicable.
inline constexpr std::array<const char*, 14> kCycleTimingFields = {
    "trigger_stamp",          // stamp of the message that triggered the cycle (= output stamp)
    "trigger_source_wall",    // when that message was published (DDS source timestamp)
    "trigger_received_wall",  // when it reached this node
    "cycle_start_wall",       // cycle start, after the rate limiter
    "output_wall",            // right after the estimate was published
    "predict_ms",
    "robot_ms",               // hand pose/twist updates (0 in camera-only mode)
    "vision_ms",              // bone-direction updates, or the dropout fallback
    "publish_ms",
    "compute_ms",             // sum of the four stages
    "vision_outcome",         // VisionOutcome
    "bone_updates",           // bone-direction updates applied (0-2)
    "keypoint_stamp",         // stamp of the keypoints applied (NaN if none)
    "keypoint_source_wall",   // when the solver published them (NaN if none)
};

// What the vision step did on one cycle.
enum class VisionOutcome : int
{
    kFresh = 0,     // applied a keypoint set for the first time
    kReused = 1,    // re-applied the cached set (the camera is slower than the cycle)
    kInvalid = 2,   // fallback: the solver rejected its latest frame
    kNone = 3,      // fallback: no keypoints yet
    kStale = 4,     // fallback: keypoints older than max_age_sec
    kRejected = 5,  // fallback: both bone updates were skipped
    kOff = 6,       // fallback: vision not in use (robot only)
};

class ArmPoseEkfNode : public rclcpp::Node
{
public:
    ArmPoseEkfNode();

private:
    using JointState = sensor_msgs::msg::JointState;
    using PoseArray = geometry_msgs::msg::PoseArray;

    // Shoulder, elbow and wrist centres from the solver, in the shoulder frame.
    struct ArmKeypoints
    {
        Eigen::Vector3d shoulder = Eigen::Vector3d::Zero();
        Eigen::Vector3d elbow = Eigen::Vector3d::Zero();
        Eigen::Vector3d wrist = Eigen::Vector3d::Zero();
        rclcpp::Time stamp;
        double source_wall = std::numeric_limits<double>::quiet_NaN();
        bool available = false;
    };

    struct VisionStep
    {
        VisionOutcome outcome = VisionOutcome::kOff;
        int bone_updates = 0;
    };

    // Clocks of one cycle; lap() closes the current stage (predict, robot, vision, publish).
    struct CycleTiming
    {
        explicit CycleTiming(const rclcpp::MessageInfo& trigger);
        void lap();

        double trigger_source_wall;
        double trigger_received_wall;
        double start_wall;
        double end_wall = std::numeric_limits<double>::quiet_NaN();
        std::array<double, 4> stage_ms{};
        std::size_t stage = 0;
        std::chrono::steady_clock::time_point last;
    };

    // --- callbacks ----------------------------------------------------------
    void onRobotJointState(const JointState::SharedPtr msg, const rclcpp::MessageInfo& info);
    void onArmKeypoints(const PoseArray::SharedPtr msg, const rclcpp::MessageInfo& info);
    void onKeypointsValid(const std_msgs::msg::Bool::SharedPtr msg);

    // --- one cycle ----------------------------------------------------------
    void predictTo(const rclcpp::Time& stamp);
    void applyRobotMeasurements(const JointState& msg);
    // Bone-direction updates, or the dropout fallback on a cycle without usable
    // vision. `trust_valid_flag` is false when the keypoints triggered the cycle.
    VisionStep applyCameraMeasurements(bool trust_valid_flag);
    void applyVisionDropoutFallback();
    // ekf.correct + limits + kinematics refresh: every measurement goes through here.
    void correct(const ekf::LinearizedMeasurement& measurement);
    void applyLimits(const std::string& source);

    // --- inputs -------------------------------------------------------------
    std::optional<pinocchio::SE3> shoulderPose();
    bool keypointsInShoulderFrame(const PoseArray& msg, ArmKeypoints& keypoints) const;

    // --- outputs ------------------------------------------------------------
    void publishState(const rclcpp::Time& stamp);
    void publishMeasuredHand(const pinocchio::SE3& T_shoulder_hand, const rclcpp::Time& stamp);
    void publishCycleTiming(const rclcpp::Time& stamp, const CycleTiming& timing,
                            const VisionStep& vision);
    void reportMissingInputs();

    ekf::EkfConfig config_;
    std::unique_ptr<ekf::RobotKinematics> robot_;  // null in camera-only mode
    std::unique_ptr<ekf::ArmKinematics> arm_;
    ekf::JointStateEkf ekf_;

    // Last world -> shoulder, held while the shoulder TF is out.
    std::optional<pinocchio::SE3> last_shoulder_;
    bool initialized_ = false;
    rclcpp::Time last_predict_stamp_;
    rclcpp::Time last_processed_robot_stamp_;
    ArmKeypoints keypoints_;
    // Stamp (ns) of the keypoint set last applied: tells fresh from re-applied.
    std::int64_t last_applied_keypoint_ns_ = -1;
    // The solver's verdict on its latest frame.
    bool keypoint_frame_valid_ = true;

    // Steady-clock arrival times for the input watchdog (negative: never received).
    double last_robot_msg_time_ = -1.0;
    double last_keypoint_msg_time_ = -1.0;

    rclcpp::Subscription<JointState>::SharedPtr robot_joint_sub_;
    rclcpp::Subscription<PoseArray>::SharedPtr keypoint_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr keypoint_valid_sub_;
    rclcpp::Publisher<JointState>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cycle_timing_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace arm_pose_estimation
