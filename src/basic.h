#include <cmath>

#include "data.h"

namespace Basic {
// 计算两个三维点之间的距离
inline auto distance(const Point3D &a, const Point3D &b) -> double {
  return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

// 计算两个三维点之间的坡度
// line参数为参考线，默认为x轴
inline auto calc_slope(const Point3D &a, const Point3D &b,
                       const Line &line = {{0, 0, 0}, {1, 0, 0}}) -> double {
  // 定义一个函数，用于计算三维点在line上的投影
  auto point_projection = [&line](const Point3D &a) -> const Point3D {
    // 计算点a到line的垂足
    const auto [x0, y0, z0] = a;
    const auto [x1, y1, z1] = line.p1;
    const auto [x2, y2, z2] = line.p2;
    const auto k =
        -((x1 - x0) * (x2 - x1) + (y1 - y0) * (y2 - y1) +
          (z1 - z0) * (z2 - z1)) /
        ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));
    return {std::lerp(x1, x2, k), std::lerp(y1, y2, k), std::lerp(z1, z2, k)};
  };
  // 计算两个三维点在line上的投影之间的距离的差，再除以两个点的距离，得到坡度
  return (distance(b, point_projection(b)) - distance(a, point_projection(a))) /
         distance(point_projection(a), point_projection(b));
}

auto solve(const Plane &plane, const Point3D &start, const Point3D &end)
    -> std::pair<Path, double>;
} // namespace Basic
