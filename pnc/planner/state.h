#include <cmath>
namespace planner {
class Pose {
public:
    Pose(double x = 0.0, double y = 0.0, double theta = 0.0)
        : x_(x)
        , y_(y)
        , theta_(theta) {}

    void translated(double dx, double dy) {
        x_ += dx * cos(theta_) - dy * sin(theta_);
        y_ += dx * sin(theta_) + dy * cos(theta_);
    }

    void rotated(double dtheta) {
        theta_ += dtheta;
        normalizeAngle();
    }

    double x() const { return x_; }
    double y() const { return y_; }
    double theta() const { return theta_; }

private:
    void normalizeAngle() {
        while (theta_ > M_PI) {
            theta_ -= 2 * M_PI;
        }
        while (theta_ < -M_PI) {
            theta_ += 2 * M_PI;
        }
    }

    double x_;
    double y_;
    double theta_;
};
} // namespace planner