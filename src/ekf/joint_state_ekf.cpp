#include "arm_pose_estimation/ekf/joint_state_ekf.hpp"

#include <algorithm>
#include <cmath>

namespace arm_pose_estimation::ekf
{
namespace
{

// Mean and variance of a STANDARD normal truncated to [a, b] (a may be -inf,
// b may be +inf). When the interval holds numerically no mass, degrade to
// projecting onto the nearest bound with zero residual variance.
struct TruncatedMoments
{
    double mean;
    double variance;
};

TruncatedMoments standardTruncatedMoments(double a, double b)
{
    constexpr double kInvSqrt2Pi = 0.3989422804014327;
    constexpr double kInvSqrt2 = 0.7071067811865476;
    constexpr double kInf = std::numeric_limits<double>::infinity();
    const auto pdf = [](double z) {
        return std::isfinite(z) ? kInvSqrt2Pi * std::exp(-0.5 * z * z) : 0.0;
    };
    const auto cdf = [](double z) {
        if (z == kInf) return 1.0;
        if (z == -kInf) return 0.0;
        return 0.5 * std::erfc(-z * kInvSqrt2);
    };

    const double mass = cdf(b) - cdf(a);
    if (!(mass > 1.0e-12)) {
        return {std::clamp(0.0, a, b), 0.0};
    }
    const double pa = pdf(a);
    const double pb = pdf(b);
    const double mean = (pa - pb) / mass;
    const double term_a = std::isfinite(a) ? a * pa : 0.0;
    const double term_b = std::isfinite(b) ? b * pb : 0.0;
    const double variance = 1.0 + (term_a - term_b) / mass - mean * mean;
    return {mean, std::clamp(variance, 0.0, 1.0)};
}

}  // namespace

void JointStateEkf::init(
    const JointVector& q0, const JointVector& dq0, double q_variance, double dq_variance)
{
    x_.head<kDof>() = q0;
    x_.tail<kDof>() = dq0;
    P_.setZero();
    P_.block<kDof, kDof>(0, 0) = q_variance * Eigen::Matrix<double, kDof, kDof>::Identity();
    P_.block<kDof, kDof>(kDof, kDof) = dq_variance * Eigen::Matrix<double, kDof, kDof>::Identity();
}

void JointStateEkf::predict(double dt, double acceleration_variance)
{
    const double step = std::max(dt, 1.0e-4);
    const Eigen::Matrix<double, kDof, kDof> identity = Eigen::Matrix<double, kDof, kDof>::Identity();

    StateMatrix F = StateMatrix::Identity();
    F.block<kDof, kDof>(0, kDof) = step * identity;
    x_.head<kDof>() += step * x_.tail<kDof>();

    // Discretized white-noise acceleration: [dt^3/3, dt^2/2; dt^2/2, dt] * variance.
    const double dt2 = step * step;
    StateMatrix Q = StateMatrix::Zero();
    Q.block<kDof, kDof>(0, 0) = (dt2 * step / 3.0) * acceleration_variance * identity;
    Q.block<kDof, kDof>(0, kDof) = (dt2 / 2.0) * acceleration_variance * identity;
    Q.block<kDof, kDof>(kDof, 0) = (dt2 / 2.0) * acceleration_variance * identity;
    Q.block<kDof, kDof>(kDof, kDof) = step * acceleration_variance * identity;

    P_ = F * P_ * F.transpose() + Q;
    P_ = 0.5 * (P_ + P_.transpose());
}

void JointStateEkf::correct(const LinearizedMeasurement& measurement)
{
    const auto& r = measurement.residual;
    const auto& H = measurement.H;
    const auto& R = measurement.R;
    if (r.size() == 0) {
        return;
    }

    const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    const Eigen::MatrixXd K =
        P_ * H.transpose() * S.ldlt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
    x_ += K * r;
    const Eigen::MatrixXd IKH = StateMatrix::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
}

void JointStateEkf::truncateComponent(int index, double lower, double upper)
{
    // For a single-component box constraint the PDF-truncation update is
    // rank one: with s^2 = P(i,i) and v = P.col(i), standardize the bounds,
    // take the truncated moments (m, w), then x += (m/s) v and
    // P += ((w-1)/s^2) v v^T. A no-op when the marginal is well inside.
    const double s2 = P_(index, index);
    if (!(s2 > 1.0e-12)) {
        return;
    }
    const double s = std::sqrt(s2);
    const double value = x_[index];
    constexpr double kInf = std::numeric_limits<double>::infinity();
    const double a = std::isfinite(lower) ? (lower - value) / s : -kInf;
    const double b = std::isfinite(upper) ? (upper - value) / s : kInf;
    if (a > b) {
        return;
    }

    const TruncatedMoments moments = standardTruncatedMoments(a, b);
    const StateVector v = P_.col(index);
    x_ += (moments.mean / s) * v;
    P_ += ((moments.variance - 1.0) / s2) * (v * v.transpose());
    P_ = 0.5 * (P_ + P_.transpose());
}

std::vector<Truncation> JointStateEkf::applyLimits(const JointLimits& limits)
{
    constexpr double kInf = std::numeric_limits<double>::infinity();
    std::vector<Truncation> truncations;

    for (int i = 0; i < kDof; ++i) {
        const double before = x_[i];
        const double lower = limits.q_lower[i];
        const double upper = limits.q_upper[i];
        truncateComponent(i, lower, upper);
        if ((std::isfinite(lower) && before < lower) || (std::isfinite(upper) && before > upper)) {
            truncations.push_back({i, before, x_[i]});
        }
    }

    for (int i = 0; i < kDof; ++i) {
        const double dq_max = limits.dq_max[i];
        if (std::isfinite(dq_max) && dq_max > 0.0) {
            truncateComponent(kDof + i, -dq_max, dq_max);
        }
    }

    for (int i = 0; i < kDof; ++i) {
        const double lower = limits.q_lower[i];
        const double upper = limits.q_upper[i];
        if (std::isfinite(lower) && x_[i] <= lower + 1.0e-6) {
            truncateComponent(kDof + i, 0.0, kInf);
        } else if (std::isfinite(upper) && x_[i] >= upper - 1.0e-6) {
            truncateComponent(kDof + i, -kInf, 0.0);
        }
    }
    return truncations;
}

}  // namespace arm_pose_estimation::ekf
