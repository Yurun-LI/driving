#include "common/algorithm/solver/a_star.h"
#include <glog/logging.h>
#include <cmath>
#include "common/math/vec.h"

namespace common {
namespace algorithm {

// --- GridCoord implementation ---
GridCoord::GridCoord(int x_, int y_) : x(x_), y(y_) {
  CHECK_LE(x, kGridSize) << "GridCoord x exceeds 10000";
  CHECK_LE(y, kGridSize) << "GridCoord y exceeds 10000";
}

int GridCoord::GetId() const {
  return x * kGridSize + y;
}

bool GridCoord::operator==(const GridCoord& other) const {
  return x == other.x && y == other.y;
}

double GridCoord::ManhattanDistance(const GridCoord& other) const {
  return std::abs(x - other.x) + std::abs(y - other.y);
}

// --- AStar implementation ---
AStar::AStar(const math::Vec2i& start, const math::Vec2i& final,
             const std::vector<math::Vec2i>& obstacles, int width, int height)
    : start_(GridCoord(start.x, start.y)),
      final_(GridCoord(final.x, final.y)),
      width_(width),
      height_(height) {
  for (const auto& obs : obstacles) {
    obstacles_.insert(GridCoord(obs.x, obs.y));
  }
  path_.clear();
  all_nodes_list_.reserve(kGridSize);
  all_nodes_list_.emplace_back(start_, 0, ComputeHeuristic(start_), nullptr);
  open_list_.emplace(&all_nodes_list_.back());
  visited_node_ptr_map_.clear();
}

AStar::~AStar() = default;

void AStar::Solve() {
  RunAStar();
}

double AStar::ComputeTransitionCost(const GridCoord& from,
                                    const GridCoord& to) const {
  return from.ManhattanDistance(to);
}

double AStar::ComputeHeuristic(const GridCoord& coord) const {
  return final_.ManhattanDistance(coord);
}

void AStar::GetSolution() const {
  for (const auto& state : path_) {
    printf("(%d, %d) ", state.x, state.y);
  }
  printf("\n");
}

const std::vector<GridCoord>& AStar::GetPath() const {
  return path_;
}

AStar::Node::Node(GridCoord c, double g_, double h_, Node* p)
    : coord(c), g_cost(g_), h_cost(h_), prev_node(p) {}

bool AStar::NodeCmp::operator()(const Node* a, const Node* b) const {
  return a->g_cost + a->h_cost > b->g_cost + b->h_cost;
}

void AStar::RunAStar() {
  open_list_ = OpenList();
  visited_node_ptr_map_.clear();
  all_nodes_list_.reserve(kGridSize);
  all_nodes_list_.emplace_back(start_, 0, ComputeHeuristic(start_), nullptr);
  open_list_.emplace(&all_nodes_list_.back());
  while (!open_list_.empty()) {
    Node* curr_node_ptr = open_list_.top();
    open_list_.pop();
    curr_node_ptr->is_closed = true;
    visited_node_ptr_map_.insert({curr_node_ptr->coord.GetId(), curr_node_ptr});
    if (curr_node_ptr->coord == final_) {
      BackTrackPath(curr_node_ptr);
      return;
    }
    for (const auto& d : directions_) {
      GridCoord next_coord{curr_node_ptr->coord.x + d.x,
                           curr_node_ptr->coord.y + d.y};
      auto it = visited_node_ptr_map_.find(next_coord.GetId());
      if (ExceedSearchRange(next_coord) || obstacles_.count(next_coord)) {
        continue;
      }
      if (it != visited_node_ptr_map_.end() && it->second->is_closed) {
        continue;
      }
      double next_g_cost =
          curr_node_ptr->g_cost +
          ComputeTransitionCost(curr_node_ptr->coord, next_coord);
      double next_h_cost = ComputeHeuristic(next_coord);
      if (it == visited_node_ptr_map_.end()) {
        all_nodes_list_.emplace_back(next_coord, next_g_cost, next_h_cost,
                                     curr_node_ptr);
        open_list_.emplace(&all_nodes_list_.back());
      } else {
        Node* next_node_ptr = it->second;
        if (next_node_ptr->g_cost + next_node_ptr->h_cost >
            next_g_cost + next_h_cost) {
          next_node_ptr->g_cost = next_g_cost;
          next_node_ptr->h_cost = next_h_cost;
          next_node_ptr->prev_node = curr_node_ptr;
          open_list_.emplace(next_node_ptr);
        }
      }
    }
  }
  open_list_ = OpenList();
  visited_node_ptr_map_.clear();
  all_nodes_list_.clear();
}

void AStar::BackTrackPath(Node* node_ptr) {
  path_.clear();
  while (node_ptr) {
    path_.insert(path_.begin(), node_ptr->coord);
    node_ptr = node_ptr->prev_node;
  }
}

bool AStar::ExceedSearchRange(const GridCoord& coord) const {
  return !(coord.x >= 0 && coord.x < width_ && coord.y >= 0 &&
           coord.y < height_);
}

}  // namespace algorithm
}  // namespace common
