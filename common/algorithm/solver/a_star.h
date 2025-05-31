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
#include "common/flags.h"
#include "common/math/vec.h"
#include "common/proto/grid_search.pb.h"

namespace common {
namespace algorithm {

constexpr double kGridSize = 1e4;

struct GridCoord {
  int x, y;
  int GetId() const;
  bool operator==(const GridCoord& other) const;
  double ManhattanDistance(const GridCoord& other) const;
  GridCoord(int x_ = 0, int y_ = 0);

  common::algorithm::proto::GridCoord ToProto() const {
    common::algorithm::proto::GridCoord proto_coord;
    proto_coord.set_x(x);
    proto_coord.set_y(y);
    return proto_coord;
  }

  static GridCoord FromProto(
      const common::algorithm::proto::GridCoord& proto_coord) {
    return GridCoord(proto_coord.x(), proto_coord.y());
  }
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

  const common::algorithm::proto::AStarSearchOutput& a_star_search_output()
      const {
    return a_star_output_;
  }

 private:
  // Node structure
  struct Node {
    GridCoord coord;
    bool is_closed = false;
    double g_cost;
    double h_cost;
    Node* prev_node = nullptr;
    Node(GridCoord c, double g_, double h_, Node* p);

    friend std::ostream& operator<<(std::ostream& os, const Node& node) {
      os << "Node(coord: (" << node.coord.x << ", " << node.coord.y
         << "), g_cost: " << node.g_cost << ", h_cost: " << node.h_cost
         << ", prev_node: " << (node.prev_node ? "exists" : "nullptr") << ")";
      return os;
    }

    common::algorithm::proto::Node ToProto() const {
      common::algorithm::proto::Node proto_node;
      proto_node.mutable_coord()->CopyFrom(coord.ToProto());
      proto_node.set_g_cost(g_cost);
      proto_node.set_h_cost(h_cost);
      return proto_node;
    }
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

  // for debug
  common::algorithm::proto::AStarSearchOutput a_star_output_;

  // Internal methods
  void RunAStar();
  void BackTrackPath(Node* node_ptr);
  bool ExceedSearchRange(const GridCoord& coord) const;
};

}  // namespace algorithm
}  // namespace common
