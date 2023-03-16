#include <cmath>

#include <boost/geometry/algorithms/detail/distance/interface.hpp>
#include <boost/geometry/arithmetic/cross_product.hpp>
#include <boost/geometry/arithmetic/dot_product.hpp>

#include "data.h"

using bg::distance;

namespace Solve {
/**
 * @brief Compute the dot product of two 3D points
 * @param a The first point
 * @param b The second point
 * @return The dot product of the two points
 */
inline auto dot(const Point3D &a, const Point3D &b) -> double {
  return bg::dot_product(a, b);
}

/**
 * @brief Compute the cross product of two 3D points
 * @param a The first point
 * @param b The second point
 * @return The cross product of the two points
 */
inline auto cross(const Point3D &a, const Point3D &b) -> Point3D {
  return bg::cross_product(a, b);
}

/**
 * @brief Check if four 3D points are coplanar
 * @param a The first point
 * @param b The second point
 * @param c The third point
 * @param d The fourth point
 * @return True if the four points are coplanar, false otherwise
 */
inline auto isCoplanar(const Point3D &a, const Point3D &b, const Point3D &c,
                       const Point3D &d) -> bool {
  auto p1 = a - b;
  auto p2 = a - c;
  auto p3 = a - d;
  return std::abs(dot(p1, cross(p2, p3))) < EPS;
}

/**
 * @brief Project a 3D point onto a 3D line
 * @param a The point to project
 * @param line The line to project onto
 * @return The projection of the point onto the line
 */
inline auto point_projection(const Point3D &p, const Line3D &line) -> Point3D {
  const auto [x0, y0, z0] = p;
  const auto [x1, y1, z1] = line.first;
  const auto [x2, y2, z2] = line.second;
  const auto k =
      -((x1 - x0) * (x2 - x1) + (y1 - y0) * (y2 - y1) + (z1 - z0) * (z2 - z1)) /
      ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));
  return Point3D(std::lerp(x1, x2, k), std::lerp(y1, y2, k),
                 std::lerp(z1, z2, k));
}

/**
 * @brief Check if a 2D point is inside a rectangle
 * @param p The point to check
 * @param r The rectangle to check against
 * @return True if the point is inside the rectangle, false otherwise
 */
inline auto isPointInsideRectangle(const Point &p, const Rectangle &r) -> bool {
  return p.x >= r.LL.x - EPS && p.x <= r.UR.x + EPS && p.y >= r.LL.y - EPS &&
         p.y <= r.UR.y + EPS;
}

/**
 * @brief Check if a 3D point is inside a 3D rectangle
 * @param p The point to check
 * @param r The rectangle to check against
 * @return True if the point is inside the rectangle, false otherwise
 */
inline auto isPointInsideRectangle3D(const Point3D &p, const Rectangle3D &r)
    -> bool {
  if (isCoplanar(r.LL, r.LR, r.UR, p)) {
    const auto p1 = point_projection(p, {r.LR, r.LL});
    const auto p2 = point_projection(p, {r.LR, r.UR});
    return std::abs(distance(p1, r.LR) + distance(p1, r.LL) -
                    distance(r.LL, r.LR)) < EPS &&
           std::abs(distance(p2, r.LR) + distance(p2, r.UR) -
                    distance(r.UR, r.LR)) < EPS;
  }
  return false;
}

/**
 * @brief Compute the slope of a line defined by two 2D points
 * @param a The first point
 * @param b The second point
 * @return The slope of the line
 */
inline auto calc_slope(const Point &a, const Point &b) -> double {
  return (b.y - a.y) / (b.x - a.x);
}

auto solve(const vector<Rectangle> &rectangles, const Point &start,
           const Point &end) -> std::pair<Path, double>;

auto solve3D(const Plane &plane, const Point3D &start, const Point3D &end)
    -> std::pair<Path3D, double>;
} // namespace Solve
