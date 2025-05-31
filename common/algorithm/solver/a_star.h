// Author: Yurun-LI
// Date: 2024-05-31
// Description: A* pathfinding algorithm for grid-based maps.

#pragma once

#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "common/algorithm/solver/solver.h"
#include "common/math/vec.h"

namespace common {
namespace algorithm {

constexpr double kGridSize = 1e4;

struct GridCoord {
  int x, y;
  int GetId() const;
  bool operator==(const GridCoord& other) const;
  double ManhattanDistance(const GridCoord& other) const;
  GridCoord(int x_ = 0, int y_ = 0);
};

struct GridCoordHash {
  std::size_t operator()(const GridCoord& c) const {
    return std::hash<int>()(c.x) ^ (std::hash<int>()(c.y) << 1);
  }
};

class AStar : public Solver {
 public:
  // Constructors & Destructor
  explicit AStar(const math::Vec2i& start, const math::Vec2i& final,
                 const std::vector<math::Vec2i>& obstacles, int width,
                 int height);
  ~AStar() override;

  // Core API
  void Solve() override;
  double ComputeTransitionCost(const GridCoord& from,
                               const GridCoord& to) const;
  double ComputeHeuristic(const GridCoord& coord) const;
  void GetSolution() const override;
  const std::vector<GridCoord>& GetPath() const;

 private:
  // Node structure
  struct Node {
    GridCoord coord;
    bool is_closed = false;
    double g_cost;
    double h_cost;
    Node* prev_node = nullptr;
    Node(GridCoord c, double g_, double h_, Node* p);
  };

  struct NodeCmp {
    bool operator()(const Node* a, const Node* b) const;
  };

  using OpenList = std::priority_queue<Node*, std::vector<Node*>, NodeCmp>;

  // Fields
  OpenList open_list_;
  std::unordered_map<int, Node*> visited_node_ptr_map_;
  std::vector<Node> all_nodes_list_;
  std::vector<GridCoord> path_;
  std::unordered_set<GridCoord, GridCoordHash> obstacles_{};
  GridCoord start_;
  GridCoord final_;
  int width_ = 0, height_ = 0;
  const std::vector<GridCoord> directions_ = {
      GridCoord(1, 0), GridCoord(-1, 0), GridCoord(0, 1), GridCoord(0, -1)};

  // Internal methods
  void RunAStar();
  void BackTrackPath(Node* node_ptr);
  bool ExceedSearchRange(const GridCoord& coord) const;
};

}  // namespace algorithm
}  // namespace common
