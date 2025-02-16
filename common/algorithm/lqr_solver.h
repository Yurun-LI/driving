#include "eigen-3.4-rc1/Eigen/Core"

namespace math {
namespace algorithm {

class LQRSolver {
 public:
  LQRSolver(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
            const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
  bool Solve(Eigen::MatrixXd* K);

 private:
  bool SolveDARE(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                 const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                 Eigen::MatrixXd* P);

  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
};

}  // namespace algorithm
}  // namespace math
