#pragma once
#include <vector>

namespace pf
{

  struct Tower
  {
    double x;
    double y;
    double reflect;
  };

  inline const std::vector<Tower> &reflectors()
  {
    static const std::vector<Tower> towers = {
        {-1.5, 1.5, 0.95},    // 1
        {18.5, 11.5, 0.90},   // 2
        {-21.5, 1.5, 0.85},   // 3
        {-1.0, 11.5, 0.80},   // 4
        {-16.5, -3.0, 0.75},  // 5
        {-1.5, -8.5, 0.70},   // 6
        {18.5, 0.0, 0.65},    // 7
        {15.0, -10.0, 0.60},  // 8
        {-21.5, -12.0, 0.55}, // 9
        {-9.0, -12.0, 0.50},  // 10
        {8.5, 3.0, 0.45}      // 11
    };
    return towers;
  }

} // namespace pf
