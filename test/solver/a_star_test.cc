#include "common/algorithm/solver/a_star.h"
#include "common/flags.h"
#include <fstream>
#include <glog/logging.h>


int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // Example usage of AStar solver
  math::Vec2i start(0, 0);
  math::Vec2i final(5, 5);
  std::vector<math::Vec2i> obstacles = {{1, 1}, {2, 2}, {3, 3}};
  int width = 10;
  int height = 10;

  LOG(ERROR) << "FLAGS_debug_search_solver: " << FLAGS_debug_search_solver;

  common::algorithm::AStar a_star_solver(start, final, obstacles, width, height);
  a_star_solver.Solve();

  const common::algorithm::proto::AStarSearchOutput& a_star_search_output =
      a_star_solver.a_star_search_output();

  // write proto
  std::ofstream ofs("output/bin/a_star_search_output.bin", std::ios::binary);
  if (!ofs) {
    std::cerr << "Failed to open output/bin/a_star_search_output.bin for writing." << std::endl;
    return 1;
  }
  if (!a_star_search_output.SerializeToOstream(&ofs)) {
    std::cerr << "Failed to serialize proto to file." << std::endl;
    return 1;
  }
  ofs.close();

  return 0;
}