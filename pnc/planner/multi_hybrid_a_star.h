#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

// Pose class to represent the pose in the Hybrid A* algorithm
class Pose {
public:
     x_y;        // x and y coordinates
    double theta;   // orientation

    Pose(double x, double y, double theta)
        : x_y(x, y)
        , theta(theta) {}

    // Rotate the pose by a given angle
    void rotate(double angle) {
        double new_x = x_y.x * std::cos(angle) - x_y.y * std::sin(angle);
        double new_y = x_y.x * std::sin(angle) + x_y.y * std::cos(angle);
        x_y.x        = new_x;
        x_y.y        = new_y;
        theta += angle;
    }

    // Translate the pose by a given vector
    void translate(const vec& translation) {
        x_y.x += translation.x;
        x_y.y += translation.y;
    }

    bool operator==(const Pose& other) const { return x_y == other.x_y && theta == other.theta; }
};

// Hash function for Pose to be used in unordered_map
struct PoseHash {
    std::size_t operator()(const Pose& pose) const { return std::hash<double>()(pose.x_y.x) ^ std::hash<double>()(pose.x_y.y) ^ std::hash<double>()(pose.theta); }
};

// State class to represent the state in the Hybrid A* algorithm
class State {
public:
    Pose pose;
    double g;   // cost to reach this state
    double h;   // heuristic cost to goal

    State(double x, double y, double theta, double g, double h)
        : pose(x, y, theta)
        , g(g)
        , h(h) {}

    double f() const { return g + h; }

    bool operator==(const State& other) const { return pose == other.pose; }
};

// Hash function for State to be used in unordered_map
struct StateHash {
    std::size_t operator()(const State& state) const { return PoseHash()(state.pose); }
};

// Hybrid A* algorithm class
class HybridAStar {
public:
    HybridAStar() {}

    std::vector<State> plan(const State& start, const State& goal) {
        std::priority_queue<State, std::vector<State>, CompareState> open_set;
        std::unordered_map<State, State, StateHash> came_from;
        std::unordered_map<State, double, StateHash> cost_so_far;

        open_set.push(start);
        came_from[start]   = start;
        cost_so_far[start] = 0;

        while (!open_set.empty()) {
            State current = open_set.top();
            open_set.pop();

            if (isGoal(current, goal)) {
                return reconstructPath(came_from, current);
            }

            for (const State& next : getNeighbors(current)) {
                double new_cost = cost_so_far[current] + cost(current, next);
                if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]) {
                    cost_so_far[next] = new_cost;
                    double priority   = new_cost + heuristic(next, goal);
                    open_set.push(State(next.pose.x_y.x, next.pose.x_y.y, next.pose.theta, new_cost, priority));
                    came_from[next] = current;
                }
            }
        }

        return std::vector<State>();   // return empty path if no path found
    }

private:
    struct CompareState {
        bool operator()(const State& a, const State& b) { return a.f() > b.f(); }
    };

    bool isGoal(const State& state, const State& goal) {
        // Define goal condition
        return std::hypot(state.pose.x_y.x - goal.pose.x_y.x, state.pose.x_y.y - goal.pose.x_y.y) < 1.0 && std::abs(state.pose.theta - goal.pose.theta) < 0.1;
    }

    std::vector<State> getNeighbors(const State& state) {
        // Define how to get neighbors
        std::vector<State> neighbors;
        // Add neighbor generation logic here
        return neighbors;
    }

    double cost(const State& from, const State& to) {
        // Define cost function
        return std::hypot(from.pose.x_y.x - to.pose.x_y.x, from.pose.x_y.y - to.pose.x_y.y);
    }

    double heuristic(const State& state, const State& goal) {
        // Define heuristic function
        return std::hypot(state.pose.x_y.x - goal.pose.x_y.x, state.pose.x_y.y - goal.pose.x_y.y);
    }

    std::vector<State> reconstructPath(std::unordered_map<State, State, StateHash>& came_from, State current) {
        std::vector<State> path;
        while (came_from[current] != current) {
            path.push_back(current);
            current = came_from[current];
        }
        path.push_back(current);
        std::reverse(path.begin(), path.end());
        return path;
    }
};

#endif   // MULTI_HYBRID_A_STAR_H