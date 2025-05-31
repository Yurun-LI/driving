// Author: Yurun-LI
// Date: 2025-05-31
// Description: Abstract base class for all solvers in the common::algorithm namespace.

namespace common {
namespace algorithm {
class Solver {
  public:
    // Constructor
    Solver() = default;
  
    // Destructor
    virtual ~Solver() = default;
  
    // Solve method to be implemented by derived classes
    virtual void Solve() = 0;
  
    // Method to get the solution, to be implemented by derived classes
    virtual void GetSolution() const = 0;
  };
  }  // namespace algorithm
  }  // namespace common