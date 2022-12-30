#include <cmath>
#include <vector>

#include "data.h"

using std::vector;

namespace Basic {
inline auto calc_slope(const Point &a, const Point &b) -> double {
  return (b.y - a.y) / (b.x - a.x);
}

inline auto distance(const Point &a, const Point &b) -> double {
  return hypot(a.x - b.x, a.y - b.y);
}

auto solve(const vector<Rectangle> &rectangles, const Point &start,
           const Point &end) -> double;
} // namespace Basic
