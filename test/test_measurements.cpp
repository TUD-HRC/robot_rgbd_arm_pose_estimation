// Measurement Jacobians against finite differences, on the real arm model
// (urdf/right_arm.urdf.xacro rendered by xacro).

#include "arm_pose_estimation/ekf/arm_kinematics.hpp"
#include "arm_pose_estimation/ekf/measurements.hpp"

#include <gtest/gtest.h>

#include <pinocchio/spatial/explog.hpp>

#include <array>
#include <cstdio>
#include <memory>
#include <string>

using namespace arm_pose_estimation::ekf;

namespace
{

constexpr double kStep = 1e-6;

std::string renderArm()
{
    const std::string command = std::string("xacro ") + ARM_XACRO_PATH +
        " robot_prefix:=upt_ upper_arm_length:=0.3 forearm_length:=0.25 hand_length:=0.09";
    std::unique_ptr<FILE, int (*)(FILE*)> pipe(popen(command.c_str(), "r"), pclose);
    std::string xml;
    std::array<char, 4096> buffer;
    std::size_t count;
    while ((count = std::fread(buffer.data(), 1, buffer.size(), pipe.get())) > 0) {
        xml.append(buffer.data(), count);
    }
    return xml;
}

class MeasurementTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { urdf_ = new std::string(renderArm()); }
    static void TearDownTestSuite() { delete urdf_; }

    void SetUp() override
    {
        ASSERT_FALSE(urdf_->empty()) << "xacro produced no URDF";
        arm_ = std::make_unique<ArmKinematics>(*urdf_, "upt_", "upt_RightHandCOM");
        // A generic configuration away from joint limits and singularities.
        q_ << 0.3, 0.6, 0.2, 1.2, 0.1, 0.1, -0.1;
        arm_->update(q_);
    }

    // Column i of d f / d q, by central differences; restores the kinematics.
    template <typename F>
    Eigen::VectorXd numericColumn(F f, int i)
    {
        JointVector plus = q_, minus = q_;
        plus[i] += kStep;
        minus[i] -= kStep;
        arm_->update(plus);
        const Eigen::VectorXd f_plus = f();
        arm_->update(minus);
        const Eigen::VectorXd f_minus = f();
        arm_->update(q_);
        return (f_plus - f_minus) / (2.0 * kStep);
    }

    static std::string* urdf_;
    std::unique_ptr<ArmKinematics> arm_;
    JointVector q_;
};

std::string* MeasurementTest::urdf_ = nullptr;

}  // namespace

TEST_F(MeasurementTest, KeepsTheSevenCoordinatesAndTheirLimits)
{
    // The mimic joints add no coordinate; the limits are the independent joints' own.
    EXPECT_EQ(arm_->jointNames().size(), static_cast<std::size_t>(kDof));
    EXPECT_NEAR(arm_->limits().q_lower[5], -1.22173, 1e-9);
    EXPECT_NEAR(arm_->limits().q_upper[5], 1.22173, 1e-9);
    EXPECT_NEAR(arm_->limits().q_lower[6], -0.43633231, 1e-9);
    EXPECT_NEAR(arm_->limits().q_upper[6], 0.17453293, 1e-9);
}

TEST_F(MeasurementTest, HandPositionJacobian)
{
    const auto m = handPosition(*arm_, Eigen::Vector3d::Zero(), 0.03);
    for (int i = 0; i < kDof; ++i) {
        // r = z - h(q), so dr/dq = -H.
        const Eigen::VectorXd dr = numericColumn(
            [&] { return Eigen::VectorXd(handPosition(*arm_, Eigen::Vector3d::Zero(), 0.03).residual); }, i);
        EXPECT_TRUE(dr.isApprox(-m.H.col(i), 1e-5)) << "column " << i;
    }
    EXPECT_TRUE(m.H.rightCols(kDof).isZero());
}

TEST_F(MeasurementTest, HandRotationJacobian)
{
    const Eigen::Matrix3d R0 = arm_->handPose().rotation();
    const auto m = handRotation(*arm_, R0, 0.05);
    for (int i = 0; i < kDof; ++i) {
        // Rotation of the hand relative to R0: log3(R(q) R0^T) ~= J_w dq.
        const Eigen::VectorXd dtheta = numericColumn(
            [&] { return Eigen::VectorXd(pinocchio::log3(arm_->handPose().rotation() * R0.transpose())); }, i);
        EXPECT_TRUE(dtheta.isApprox(m.H.col(i), 1e-5)) << "column " << i;
    }
    EXPECT_TRUE(m.R.isApprox(0.0025 * Eigen::Matrix3d::Identity()));
}

TEST_F(MeasurementTest, HandTwistIsLinearInVelocity)
{
    const JointVector dq = JointVector::LinSpaced(-0.3, 0.3);
    const auto m = handTwist(*arm_, Vector6::Zero(), dq, 0.01, 0.03);
    EXPECT_TRUE(m.H.leftCols(kDof).isZero());
    EXPECT_TRUE(m.residual.isApprox(-m.H.rightCols(kDof) * dq));
}

TEST_F(MeasurementTest, BoneDirectionJacobian)
{
    for (const Bone* bone : {&arm_->upperArm(), &arm_->forearm()}) {
        std::string reason;
        const Eigen::Vector3d start = arm_->framePose(bone->start_frame).translation();
        const Eigen::Vector3d end = arm_->framePose(bone->end_frame).translation();
        const auto m = boneDirection(*arm_, *bone, start, end, 0.05, reason);
        ASSERT_TRUE(m.has_value()) << reason;
        EXPECT_LT(m->residual.norm(), 1e-12);  // measured == model

        const auto direction = [&] {
            const Eigen::Vector3d s = arm_->framePose(bone->end_frame).translation() -
                arm_->framePose(bone->start_frame).translation();
            return Eigen::VectorXd(s.normalized());
        };
        for (int i = 0; i < kDof; ++i) {
            EXPECT_LT((numericColumn(direction, i) - m->H.col(i).head<3>()).norm(), 1e-6)
                << bone->name << " column " << i;
        }
        EXPECT_TRUE(m->H.rightCols(kDof).isZero());
    }
}

TEST_F(MeasurementTest, DegenerateBoneIsSkipped)
{
    std::string reason;
    const auto m = boneDirection(*arm_, arm_->upperArm(), Eigen::Vector3d::Ones(),
                                 Eigen::Vector3d::Ones(), 0.05, reason);
    EXPECT_FALSE(m.has_value());
    EXPECT_FALSE(reason.empty());
}

TEST_F(MeasurementTest, SelfMotionBasisIsTheHandNullSpace)
{
    const auto basis = arm_->handSelfMotionBasis(1e-3);
    ASSERT_EQ(basis.basis.cols(), 1);  // generic configuration: just the swivel
    EXPECT_LT((arm_->handJacobian() * basis.basis).norm(), 1e-9);
    EXPECT_NEAR(basis.basis.col(0).norm(), 1.0, 1e-12);

    const JointVector dq = basis.basis.col(0);
    const auto m = selfMotionDamping(basis.basis, dq, 1e-3);
    EXPECT_NEAR(m.residual[0], -1.0, 1e-12);
    EXPECT_TRUE(m.H.leftCols(kDof).isZero());
}

TEST_F(MeasurementTest, HandFromEndEffectorComposesFrames)
{
    const pinocchio::SE3 T_world_shoulder(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1, 0, 0));
    const pinocchio::SE3 T_world_ee(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1, 1, 0));
    const pinocchio::SE3 T_ee_hand(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0.1));
    Vector6 twist;
    twist << 0, 0, 0, 0, 0, 1;  // pure rotation about world z at the EE
    const auto hand = handFromEndEffector(T_world_ee, twist, T_world_shoulder, T_ee_hand);
    EXPECT_TRUE(hand.pose.translation().isApprox(Eigen::Vector3d(0, 1, 0.1)));
    // The hand sits on the rotation axis (offset along z), so no linear velocity.
    EXPECT_LT(hand.twist.head<3>().norm(), 1e-12);
    EXPECT_NEAR(hand.twist[5], 1.0, 1e-12);
}
