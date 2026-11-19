#pragma once

// Filter core: state, covariance, predict, correct, joint limits.
// No ROS and no kinematics: measurements arrive already linearized.

#include <Eigen/Dense>

#include <limits>
#include <string>
#include <vector>

namespace arm_pose_estimation::ekf
{

// Seven independent arm coordinates, in this order:
//   0 elv_angle, 1 shoulder_elv, 2 shoulder_rot, 3 elbow_flexion,
//   4 pro_sup (forearm), 5 wrist_x, 6 wrist_z
constexpr int kDof = 7;
constexpr int kStateSize = 2 * kDof;  // x = [q, dq]

using JointVector = Eigen::Matrix<double, kDof, 1>;
using StateVector = Eigen::Matrix<double, kStateSize, 1>;
using StateMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;

// One measurement, linearized at the current state: residual r = z - h(x),
// Jacobian H = dh/dx (rows x kStateSize) and noise covariance R.
struct LinearizedMeasurement
{
    std::string name;
    Eigen::VectorXd residual;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;
};

// Box limits on q and |dq|; infinite entries are unconstrained.
struct JointLimits
{
    JointVector q_lower = JointVector::Constant(-std::numeric_limits<double>::infinity());
    JointVector q_upper = JointVector::Constant(std::numeric_limits<double>::infinity());
    JointVector dq_max = JointVector::Constant(std::numeric_limits<double>::infinity());
};

// A joint whose mean was outside its position limits before truncation.
struct Truncation
{
    int joint;
    double before;
    double after;
};

class JointStateEkf
{
public:
    void init(const JointVector& q0, const JointVector& dq0, double q_variance, double dq_variance);

    // Constant-velocity model with white-noise acceleration:
    //   q += dt * dq,  dq unchanged,  P = F P F^T + Q(dt, acceleration_variance).
    void predict(double dt, double acceleration_variance);

    // Standard EKF update with a Joseph-form covariance update.
    void correct(const LinearizedMeasurement& measurement);

    // Enforce the limits by Gaussian PDF truncation (mean and covariance):
    // positions, then velocity magnitudes, then the velocity pointing further
    // out of a position bound the joint sits on.
    std::vector<Truncation> applyLimits(const JointLimits& limits);

    // Truncate one state component to [lower, upper] (Simon & Simon 2010).
    void truncateComponent(int index, double lower, double upper);

    JointVector q() const { return x_.head<kDof>(); }
    JointVector dq() const { return x_.tail<kDof>(); }
    const StateVector& x() const { return x_; }
    const StateMatrix& P() const { return P_; }

private:
    StateVector x_ = StateVector::Zero();
    StateMatrix P_ = StateMatrix::Identity();
};

}  // namespace arm_pose_estimation::ekf
