// Filter core: predict, correct and the PDF-truncation limits.

#include "arm_pose_estimation/ekf/joint_state_ekf.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace arm_pose_estimation::ekf;

namespace
{

bool isSymmetricPsd(const StateMatrix& P)
{
    if (!P.isApprox(P.transpose(), 1e-12)) return false;
    return Eigen::SelfAdjointEigenSolver<StateMatrix>(P).eigenvalues().minCoeff() > -1e-12;
}

JointStateEkf makeFilter()
{
    JointStateEkf filter;
    JointVector q0;
    q0 << 0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0;
    JointVector dq0 = JointVector::Constant(0.5);
    filter.init(q0, dq0, 0.1, 0.2);
    return filter;
}

}  // namespace

TEST(JointStateEkf, PredictIsConstantVelocity)
{
    JointStateEkf filter = makeFilter();
    const JointVector q_before = filter.q();
    filter.predict(0.01, 1e-3);
    EXPECT_TRUE(filter.q().isApprox(q_before + 0.01 * JointVector::Constant(0.5)));
    EXPECT_TRUE(filter.dq().isApprox(JointVector::Constant(0.5)));
    EXPECT_TRUE(isSymmetricPsd(filter.P()));
    // Position uncertainty grows through the velocity coupling.
    EXPECT_GT(filter.P()(0, 0), 0.1);
    EXPECT_GT(filter.P()(0, kDof), 0.0);
}

TEST(JointStateEkf, CorrectMovesTowardMeasurementAndShrinksVariance)
{
    JointStateEkf filter = makeFilter();
    LinearizedMeasurement m;
    m.name = "q0";
    m.H = Eigen::MatrixXd::Zero(1, kStateSize);
    m.H(0, 0) = 1.0;
    m.residual = Eigen::VectorXd::Constant(1, 1.0 - filter.q()[0]);  // measure q0 = 1.0
    m.R = Eigen::MatrixXd::Constant(1, 1, 0.1);                     // same as prior variance

    filter.correct(m);
    EXPECT_NEAR(filter.q()[0], 0.55, 1e-12);  // equal weights: halfway
    EXPECT_NEAR(filter.P()(0, 0), 0.05, 1e-12);
    EXPECT_NEAR(filter.q()[1], 0.2, 1e-12);   // uncorrelated: untouched
    EXPECT_TRUE(isSymmetricPsd(filter.P()));
}

TEST(JointStateEkf, EmptyMeasurementIsANoOp)
{
    JointStateEkf filter = makeFilter();
    const StateVector x = filter.x();
    filter.correct(LinearizedMeasurement{"empty", Eigen::VectorXd(0), Eigen::MatrixXd(0, kStateSize),
                                         Eigen::MatrixXd(0, 0)});
    EXPECT_TRUE(filter.x().isApprox(x));
}

TEST(JointStateEkf, LimitsPullMeanInsideAndReport)
{
    JointStateEkf filter;
    JointVector q0 = JointVector::Zero();
    q0[3] = 2.0;  // outside [0, 1.5]
    filter.init(q0, JointVector::Zero(), 0.01, 0.01);

    JointLimits limits;
    limits.q_lower[3] = 0.0;
    limits.q_upper[3] = 1.5;
    const auto truncations = filter.applyLimits(limits);

    ASSERT_EQ(truncations.size(), 1u);
    EXPECT_EQ(truncations[0].joint, 3);
    EXPECT_DOUBLE_EQ(truncations[0].before, 2.0);
    EXPECT_LE(filter.q()[3], 1.5 + 1e-9);
    EXPECT_GE(filter.q()[3], 0.0);
    EXPECT_LT(filter.P()(3, 3), 0.01);
    // At the upper bound, velocity pointing further out is truncated away.
    EXPECT_LE(filter.dq()[3], 1e-9);
    EXPECT_TRUE(isSymmetricPsd(filter.P()));
}

TEST(JointStateEkf, LimitsAreANoOpWellInside)
{
    JointStateEkf filter = makeFilter();
    const StateVector x = filter.x();
    JointLimits limits;
    limits.q_lower = JointVector::Constant(-10.0);
    limits.q_upper = JointVector::Constant(10.0);
    EXPECT_TRUE(filter.applyLimits(limits).empty());
    EXPECT_TRUE(filter.x().isApprox(x, 1e-9));
}
