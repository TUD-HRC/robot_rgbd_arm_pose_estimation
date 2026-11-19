#include "arm_pose_estimation/arm_pose_ekf_node.hpp"

#include "arm_pose_estimation/ekf/measurements.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

namespace arm_pose_estimation
{
namespace
{

#ifdef __OPTIMIZE__
constexpr bool kOptimizedBuild = true;
#else
constexpr bool kOptimizedBuild = false;
#endif

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kRobotDescriptionTimeoutSec = 10.0;
// Input watchdog: report every period; a topic quiet for longer than the timeout is missing.
constexpr double kReportPeriodSec = 5.0;
constexpr double kSilenceTimeoutSec = 5.0;
// A topic with a publisher may take this long to deliver its first message (a recording starts
// a few seconds after launch, its first keypoints later still).
constexpr double kFirstMessageGraceSec = 10.0;

// Seconds on a monotonic clock: the watchdog must work when /clock never comes.
double steadyNowSec()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Unix seconds on the system clock, the clock DDS source timestamps use.
double systemNowSec()
{
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

// A middleware timestamp (ns, system clock) in seconds; NaN if unset.
double rmwTimeSec(rmw_time_point_value_t nanoseconds)
{
    return nanoseconds > 0 ? static_cast<double>(nanoseconds) * 1e-9 : kNaN;
}

pinocchio::SE3 toSE3(const geometry_msgs::msg::Transform& transform)
{
    const Eigen::Quaterniond rotation(
        transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
    return pinocchio::SE3(
        rotation.normalized().toRotationMatrix(),
        Eigen::Vector3d(transform.translation.x, transform.translation.y, transform.translation.z));
}

// The robot's URDF from its latched description topic. Read on a throwaway
// node, since the EKF node is still inside its constructor.
std::string fetchRobotDescription(const std::string& topic, double timeout_sec)
{
    // No global arguments: the launch's node-name remap would clash with the EKF's own name.
    auto listener = std::make_shared<rclcpp::Node>(
        "arm_pose_ekf_description_listener", rclcpp::NodeOptions().use_global_arguments(false));
    std::string description;
    const auto subscription = listener->create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(1).transient_local().reliable(),
        [&description](const std_msgs::msg::String::SharedPtr msg) { description = msg->data; });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(listener);
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeout_sec));
    while (description.empty() && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_once(std::chrono::milliseconds(100));
    }
    executor.remove_node(listener);

    if (description.empty()) {
        throw std::runtime_error(
            "No robot description on '" + topic + "' within " + std::to_string(timeout_sec) +
            " s. Start the robot driver, or play a recording.");
    }
    return description;
}

}  // namespace

// =============================================================================
// SETUP
// =============================================================================
ArmPoseEkfNode::ArmPoseEkfNode()
    : rclcpp::Node("arm_pose_ekf"),
      config_(ekf::loadEkfConfig(*this))
{
    const auto& c = config_;
    start_time_ = steadyNowSec();

    if (c.usesRobot()) {
        if (c.robot_description.empty()) {
            RCLCPP_INFO(get_logger(), "Waiting for the robot model on %s...",
                        c.robot_description_topic.c_str());
        }
        robot_ = std::make_unique<ekf::RobotKinematics>(
            c.robot_description.empty()
                ? fetchRobotDescription(c.robot_description_topic, kRobotDescriptionTimeoutSec)
                : c.robot_description,
            c.robot_ee_frame);
    }
    arm_ = std::make_unique<ekf::ArmKinematics>(c.arm_description, c.arm_prefix, c.hand_frame);

    ekf_.init(c.initial_q, c.initial_dq, c.initial_q_variance, c.initial_dq_variance);
    applyLimits("initialization");
    arm_->update(ekf_.q());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    state_pub_ = create_publisher<JointState>(c.output_topic, 10);
    cycle_timing_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        c.cycle_timing_topic, 100);

    if (c.usesRobot()) {
        robot_joint_sub_ = create_subscription<JointState>(
            c.robot_joint_topic, 1,
            [this](const JointState::SharedPtr msg, const rclcpp::MessageInfo& info) {
                onRobotJointState(msg, info);
            });
    }
    if (c.usesCamera()) {
        keypoint_sub_ = create_subscription<PoseArray>(
            c.keypoints_topic, 1,
            [this](const PoseArray::SharedPtr msg, const rclcpp::MessageInfo& info) {
                onArmKeypoints(msg, info);
            });
        keypoint_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
            c.keypoints_valid_topic, 1,
            std::bind(&ArmPoseEkfNode::onKeypointsValid, this, std::placeholders::_1));
    }
    // Wall timer: still fires under use_sim_time when /clock never arrives.
    watchdog_timer_ = create_wall_timer(
        std::chrono::duration<double>(kReportPeriodSec),
        std::bind(&ArmPoseEkfNode::reportMissingInputs, this));

    if (!kOptimizedBuild) {
        RCLCPP_WARN(get_logger(), "Built without optimization: cycle times are not real time. "
                                  "Rebuild with -DCMAKE_BUILD_TYPE=Release.");
    }
    RCLCPP_INFO(get_logger(), "arm_pose_ekf ready: sensors=%s.", ekf::toString(c.sensors));
    if (c.sensors == ekf::SensorSet::kCamera) {
        RCLCPP_WARN(get_logger(), "Camera only: pro/supination and the wrist (q4-q6) are not "
                                  "observed and stay near their initial values.");
    }
}

// =============================================================================
// CALLBACKS
// =============================================================================

// Trigger (fused, robot): predict -> robot -> camera (or fallback) -> publish.
void ArmPoseEkfNode::onRobotJointState(
    const JointState::SharedPtr msg, const rclcpp::MessageInfo& info)
{
    last_robot_msg_time_ = steadyNowSec();
    if (msg->position.size() != static_cast<std::size_t>(robot_->nq()) ||
        msg->velocity.size() != static_cast<std::size_t>(robot_->nv())) {
        RCLCPP_WARN(get_logger(), "Robot joint state size mismatch with model.");
        return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    if (initialized_ && config_.max_update_rate_hz > 0.0 &&
        (stamp - last_processed_robot_stamp_).seconds() < 1.0 / config_.max_update_rate_hz) {
        return;
    }

    CycleTiming timing(info);
    predictTo(stamp);
    last_processed_robot_stamp_ = stamp;
    timing.lap();
    applyRobotMeasurements(*msg);
    timing.lap();
    const VisionStep vision = applyCameraMeasurements(true);
    timing.lap();
    publishState(stamp);
    timing.lap();
    publishCycleTiming(stamp, timing, vision);
}

// Caches the keypoints; in camera-only mode also runs the cycle.
void ArmPoseEkfNode::onArmKeypoints(
    const PoseArray::SharedPtr msg, const rclcpp::MessageInfo& info)
{
    last_keypoint_msg_time_ = steadyNowSec();
    CycleTiming timing(info);
    ArmKeypoints keypoints;
    if (!keypointsInShoulderFrame(*msg, keypoints)) {
        return;
    }
    keypoints.source_wall = rmwTimeSec(info.get_rmw_message_info().source_timestamp);
    keypoints_ = keypoints;

    if (config_.sensors == ekf::SensorSet::kCamera) {
        predictTo(keypoints_.stamp);
        timing.lap();
        timing.lap();   // no robot stage
        const VisionStep vision = applyCameraMeasurements(false);
        timing.lap();
        publishState(keypoints_.stamp);
        timing.lap();
        publishCycleTiming(keypoints_.stamp, timing, vision);
    }
}

// One flag per processed camera frame; the solver sends no keypoints for a rejected one.
void ArmPoseEkfNode::onKeypointsValid(const std_msgs::msg::Bool::SharedPtr msg)
{
    last_keypoint_msg_time_ = steadyNowSec();
    keypoint_frame_valid_ = msg->data;
}

// =============================================================================
// CYCLE
// =============================================================================
void ArmPoseEkfNode::predictTo(const rclcpp::Time& stamp)
{
    if (!initialized_) {
        initialized_ = true;
        last_predict_stamp_ = stamp;
    }
    double dt = (stamp - last_predict_stamp_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
        dt = 1.0 / 250.0;
    }
    ekf_.predict(dt, config_.acceleration_variance);
    last_predict_stamp_ = stamp;
    applyLimits("predict");
    arm_->update(ekf_.q());
}

void ArmPoseEkfNode::correct(const ekf::LinearizedMeasurement& measurement)
{
    ekf_.correct(measurement);
    applyLimits(measurement.name);
    arm_->update(ekf_.q());
}

void ArmPoseEkfNode::applyLimits(const std::string& source)
{
    const auto& limits = arm_->limits();
    for (const auto& t : ekf_.applyLimits(limits)) {
        RCLCPP_DEBUG(get_logger(), "Truncated %s from %.6f into [%.6f, %.6f] during %s.",
                     arm_->jointNames()[t.joint].c_str(), t.before, limits.q_lower[t.joint],
                     limits.q_upper[t.joint], source.c_str());
    }
}

void ArmPoseEkfNode::applyRobotMeasurements(const JointState& msg)
{
    const auto ee = robot_->endEffector(
        Eigen::VectorXd::Map(msg.position.data(), robot_->nq()),
        Eigen::VectorXd::Map(msg.velocity.data(), robot_->nv()));
    const auto T_world_shoulder = shoulderPose();
    if (!T_world_shoulder) {
        return;
    }
    const auto& c = config_;
    const auto hand = ekf::handFromEndEffector(ee.pose, ee.twist, *T_world_shoulder, c.ee_to_hand);
    publishMeasuredHand(hand.pose, msg.header.stamp);

    correct(ekf::handPosition(*arm_, hand.pose.translation(), c.hand_position_sigma));
    correct(ekf::handRotation(*arm_, hand.pose.rotation(), c.hand_rotation_sigma));
    correct(ekf::handTwist(*arm_, hand.twist, ekf_.dq(), c.hand_linear_velocity_sigma,
                           c.hand_angular_velocity_sigma));
}

// The latest keypoint set is applied on every cycle until a newer one arrives
// or it is older than max_age_sec.
ArmPoseEkfNode::VisionStep ArmPoseEkfNode::applyCameraMeasurements(bool trust_valid_flag)
{
    const auto fallback = [this](VisionOutcome outcome) {
        applyVisionDropoutFallback();
        return VisionStep{outcome, 0};
    };
    if (!config_.usesCamera()) {
        return fallback(VisionOutcome::kOff);
    }
    if (trust_valid_flag && !keypoint_frame_valid_) {
        return fallback(VisionOutcome::kInvalid);
    }
    if (!keypoints_.available) {
        return fallback(VisionOutcome::kNone);
    }
    if ((last_predict_stamp_ - keypoints_.stamp).seconds() > config_.keypoint_max_age_sec) {
        return fallback(VisionOutcome::kStale);
    }

    int bone_updates = 0;
    const std::array<std::pair<const ekf::Bone*, std::pair<Eigen::Vector3d, Eigen::Vector3d>>, 2>
        bones = {{
            {&arm_->upperArm(), {keypoints_.shoulder, keypoints_.elbow}},
            {&arm_->forearm(), {keypoints_.elbow, keypoints_.wrist}},
        }};
    for (const auto& [bone, points] : bones) {
        std::string skip_reason;
        const auto measurement = ekf::boneDirection(
            *arm_, *bone, points.first, points.second, config_.keypoint_direction_sigma,
            skip_reason);
        if (!measurement) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Skipping a bone-direction update: %s.", skip_reason.c_str());
            continue;
        }
        correct(*measurement);
        ++bone_updates;
    }
    if (bone_updates == 0) {
        return fallback(VisionOutcome::kRejected);
    }
    const std::int64_t stamp_ns = keypoints_.stamp.nanoseconds();
    const bool fresh = stamp_ns != last_applied_keypoint_ns_;
    last_applied_keypoint_ns_ = stamp_ns;
    return {fresh ? VisionOutcome::kFresh : VisionOutcome::kReused, bone_updates};
}

// Without vision the robot observes 6 of the 7 DOF: damp the self-motion
// (elbow swivel) rate. Not used without the robot.
void ArmPoseEkfNode::applyVisionDropoutFallback()
{
    if (!config_.usesRobot()) {
        return;
    }
    const auto basis = arm_->handSelfMotionBasis(config_.self_motion_singular_value_ratio);
    if (basis.basis.cols() == 0) {
        return;
    }
    correct(ekf::selfMotionDamping(basis.basis, ekf_.dq(), config_.self_motion_sigma));
}

// =============================================================================
// INPUTS
// =============================================================================

// World -> shoulder: the latest shoulder TF, or the last known pose while it is out.
std::optional<pinocchio::SE3> ArmPoseEkfNode::shoulderPose()
{
    try {
        last_shoulder_ = toSE3(tf_buffer_->lookupTransform(
            config_.world_frame, config_.shoulder_frame, tf2::TimePointZero).transform);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s: %s",
                             last_shoulder_ ? "Shoulder TF unavailable, using the last pose"
                                            : "No shoulder pose yet, skipping the hand update",
                             ex.what());
    }
    return last_shoulder_;
}

// The solver publishes shoulder, elbow and wrist in the camera frame; the arm
// model is rooted at the shoulder frame.
bool ArmPoseEkfNode::keypointsInShoulderFrame(const PoseArray& msg, ArmKeypoints& keypoints) const
{
    if (msg.poses.size() < 3) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Arm keypoint message has %zu poses; expected shoulder, elbow, wrist.",
                             msg.poses.size());
        return false;
    }
    pinocchio::SE3 T_shoulder_camera;
    try {
        T_shoulder_camera = toSE3(tf_buffer_->lookupTransform(
            config_.shoulder_frame, msg.header.frame_id, msg.header.stamp,
            rclcpp::Duration::from_seconds(config_.tf_lookup_timeout_sec)).transform);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Failed to transform arm keypoints to the shoulder frame: %s",
                             ex.what());
        return false;
    }

    const auto to_shoulder = [&T_shoulder_camera](const geometry_msgs::msg::Pose& pose) {
        return T_shoulder_camera.act(Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));
    };
    keypoints.shoulder = to_shoulder(msg.poses[0]);
    keypoints.elbow = to_shoulder(msg.poses[1]);
    keypoints.wrist = to_shoulder(msg.poses[2]);
    keypoints.stamp = msg.header.stamp;
    keypoints.available = true;
    return keypoints.shoulder.allFinite() && keypoints.elbow.allFinite() &&
        keypoints.wrist.allFinite();
}

// =============================================================================
// OUTPUTS
// =============================================================================
void ArmPoseEkfNode::publishState(const rclcpp::Time& stamp)
{
    applyLimits("publish");

    JointState msg;
    msg.header.stamp = stamp;
    msg.name = arm_->jointNames();
    const ekf::JointVector q = ekf_.q();
    const ekf::JointVector dq = ekf_.dq();
    msg.position.assign(q.data(), q.data() + ekf::kDof);
    msg.velocity.assign(dq.data(), dq.data() + ekf::kDof);
    state_pub_->publish(msg);
}

// For display: the robot-measured hand, to compare with the estimate in RViz.
void ArmPoseEkfNode::publishMeasuredHand(
    const pinocchio::SE3& T_shoulder_hand, const rclcpp::Time& stamp)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = config_.shoulder_frame;
    transform.child_frame_id = config_.arm_prefix + "RightHand_measured";
    const Eigen::Vector3d& t = T_shoulder_hand.translation();
    transform.transform.translation.x = t.x();
    transform.transform.translation.y = t.y();
    transform.transform.translation.z = t.z();
    const Eigen::Quaterniond q(T_shoulder_hand.rotation());
    transform.transform.rotation.w = q.w();
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    tf_broadcaster_->sendTransform(transform);
}

// =============================================================================
// TIMING
// =============================================================================
ArmPoseEkfNode::CycleTiming::CycleTiming(const rclcpp::MessageInfo& trigger)
    : trigger_source_wall(rmwTimeSec(trigger.get_rmw_message_info().source_timestamp)),
      trigger_received_wall(rmwTimeSec(trigger.get_rmw_message_info().received_timestamp)),
      start_wall(systemNowSec()),
      last(std::chrono::steady_clock::now())
{
}

void ArmPoseEkfNode::CycleTiming::lap()
{
    const auto now = std::chrono::steady_clock::now();
    if (stage < stage_ms.size()) {
        stage_ms[stage++] = std::chrono::duration<double, std::milli>(now - last).count();
    }
    last = now;
    end_wall = systemNowSec();
}

// Sent after the cycle, so it is not part of the time it reports.
void ArmPoseEkfNode::publishCycleTiming(
    const rclcpp::Time& stamp, const CycleTiming& timing, const VisionStep& vision)
{
    const bool used_keypoints = vision.outcome == VisionOutcome::kFresh ||
        vision.outcome == VisionOutcome::kReused;
    const auto& ms = timing.stage_ms;
    const std::array<double, kCycleTimingFields.size()> values = {
        stamp.seconds(),
        timing.trigger_source_wall,
        timing.trigger_received_wall,
        timing.start_wall,
        timing.end_wall,
        ms[0], ms[1], ms[2], ms[3],
        ms[0] + ms[1] + ms[2] + ms[3],
        static_cast<double>(static_cast<int>(vision.outcome)),
        static_cast<double>(vision.bone_updates),
        used_keypoints ? keypoints_.stamp.seconds() : kNaN,
        used_keypoints ? keypoints_.source_wall : kNaN,
    };

    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(1);
    // The label says which build made the record: unoptimized times are no benchmark.
    msg.layout.dim[0].label = kOptimizedBuild ? "cycle_timing_v1 optimized"
                                              : "cycle_timing_v1 unoptimized";
    msg.layout.dim[0].size = static_cast<std::uint32_t>(values.size());
    msg.layout.dim[0].stride = static_cast<std::uint32_t>(values.size());
    msg.data.assign(values.begin(), values.end());
    cycle_timing_pub_->publish(msg);
}

// Logs every required input that is silent. The two keypoint topics count as
// one: the solver withholds keypoints on rejected frames, which is normal.
void ArmPoseEkfNode::reportMissingInputs()
{
    const double now = steadyNowSec();
    std::vector<std::string> problems;
    const auto check = [&](const std::string& label, double last_seen, std::size_t publishers) {
        if (last_seen < 0.0) {
            if (publishers == 0) {
                problems.push_back(label + ": no publisher");
            } else if (now - start_time_ > kFirstMessageGraceSec) {
                problems.push_back(label + ": " + std::to_string(publishers) +
                                   " publisher(s) but no message received");
            }
        } else if (now - last_seen > kSilenceTimeoutSec) {
            char seconds[32];
            std::snprintf(seconds, sizeof(seconds), "%.1f", now - last_seen);
            problems.push_back(label + ": silent for " + seconds + " s");
        }
    };

    if (config_.usesRobot()) {
        check(config_.robot_joint_topic, last_robot_msg_time_,
              count_publishers(config_.robot_joint_topic));
    }
    if (config_.usesCamera()) {
        check(config_.keypoints_topic + " / " + config_.keypoints_valid_topic,
              last_keypoint_msg_time_,
              count_publishers(config_.keypoints_topic) +
                  count_publishers(config_.keypoints_valid_topic));
    }
    if (problems.empty()) {
        return;
    }
    std::string message = "Required input topic(s) unavailable (sensors=" +
        std::string(ekf::toString(config_.sensors)) + "):";
    for (const auto& problem : problems) {
        message += "\n  " + problem;
    }
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
}

}  // namespace arm_pose_estimation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int status = 0;
    try {
        rclcpp::spin(std::make_shared<arm_pose_estimation::ArmPoseEkfNode>());
    } catch (const std::exception& ex) {
        RCLCPP_FATAL(rclcpp::get_logger("arm_pose_ekf"), "arm_pose_ekf failed: %s", ex.what());
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
