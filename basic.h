#include <cmath>

#include "data.h"

namespace Basic {
inline auto calc_slope(const Point &a, const Point &b) -> double {
  return (b.y - a.y) / (b.x - a.x);
}

inline auto distance(const Point &a, const Point &b) -> double {
  return hypot(a.x - b.x, a.y - b.y);
}

auto solve(const Plane &plane, const Point &start, const Point &end)
    -> std::pair<Path, double>;
} // namespace Basic
