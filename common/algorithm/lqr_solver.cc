#include "common/algorithm/lqr_solver.h"

namespace math {
namespace algorithm {

LQRSolver::LQRSolver(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                     const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
    : A_(A), B_(B), Q_(Q), R_(R) {}

bool LQRSolver::Solve(Eigen::MatrixXd* K) {
  if (K == nullptr) {
    return false;
  }

  // Solve the discrete-time algebraic Riccati equation (DARE)
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(A_.rows(), A_.cols());
  if (!SolveDARE(A_, B_, Q_, R_, &P)) {
    return false;
  }

  // Compute the LQR gain
  *K = (R_ + B_.transpose() * P * B_).inverse() * B_.transpose() * P * A_;
  return true;
}

bool LQRSolver::SolveDARE(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                          const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                          Eigen::MatrixXd* P) {
  const int max_iterations = 1000;
  const double tolerance = 1e-9;
  Eigen::MatrixXd P_old = Q;
  Eigen::MatrixXd P_new = Q;

  for (int i = 0; i < max_iterations; ++i) {
    P_new = A.transpose() * P_old * A -
            A.transpose() * P_old * B *
                (R + B.transpose() * P_old * B).inverse() * B.transpose() *
                P_old * A +
            Q;
    if ((P_new - P_old).norm() < tolerance) {
      *P = P_new;
      return true;
    }
    P_old = P_new;
  }

  return false;  // Did not converge
}

}  // namespace algorithm
}  // namespace math
