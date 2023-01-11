#include <cmath>

#include "data.h"

namespace Basic {
inline auto distance(const Point &a, const Point &b) -> double {
  return std::hypot(a.x - b.x, a.y - b.y);
}

inline auto distance(const Point3D &a, const Point3D &b) -> double {
  return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline auto dot(const Point3D &a, const Point3D &b) -> double {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline auto cross(const Point3D &a, const Point3D &b) -> Point3D {
  return {a.y * b.z - b.y * a.z, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline auto isCoplanar(const Point3D &a, const Point3D &b, const Point3D &c,
                       const Point3D &d) -> bool {
  auto p1 = a - b;
  auto p2 = a - c;
  auto p3 = a - d;
  return std::abs(dot(p1, cross(p2, p3))) < 1e-9;
};

// 定义一个函数，用于计算三维点在line上的投影
inline auto point_projection(const Point3D &a, const Line &line)
    -> const Point3D {
  // 计算点a到line的垂足
  const auto [x0, y0, z0] = a;
  const auto [x1, y1, z1] = line.p1;
  const auto [x2, y2, z2] = line.p2;
  const auto k =
      -((x1 - x0) * (x2 - x1) + (y1 - y0) * (y2 - y1) + (z1 - z0) * (z2 - z1)) /
      ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));
  return {std::lerp(x1, x2, k), std::lerp(y1, y2, k), std::lerp(z1, z2, k)};
};

// 计算两个三维点之间的坡度
// line参数为参考线，默认为x轴
inline auto calc_slope(const Point &a, const Point &b) -> double {
  // 计算两个三维点在line上的投影之间的距离的差，再除以两个点的距离，得到坡度
  return (b.y - a.y) / (b.x - a.x);
}

auto solve(const vector<Rectangle> &rectangles, const Point &start,
           const Point &end) -> std::pair<Path, double>;

auto solve3D(const Plane &plane, const Point3D &start, const Point3D &end)
    -> std::pair<Path3D, double>;
} // namespace Basic
