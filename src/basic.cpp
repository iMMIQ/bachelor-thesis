#include <algorithm>
#include <array>
#include <cassert>
#include <limits>

#include "basic.h"

using std::max;
using std::min;

namespace Basic {
auto solve(const Plane &plane, const Point3D &start, const Point3D &end)
    -> std::pair<Path, double> {
  constexpr auto INF = std::numeric_limits<double>::max(), EPS = 1e-9;
  // TODO: 这样定义是因为斜率计算未考虑正负值，需要修改calc_slope函数
  const Line x_axis{plane[0].LL - Point3D(100, 100, 100),
                    plane[0].LR - Point3D(100, 100, 100)};
  int n = plane.size();
  vector<Point> L(n), R(n);
  vector<std::array<double, 2>> dp(n + 1, {INF, INF});
  Path path{start};
  for (int i = 0; i < n - 1; ++i) {
    if (plane[i].LL.y > plane[i + 1].LL.y) {
      L[i] = {plane[i].LL.y, plane[i].LL.z};
    } else {
      L[i] = {plane[i + 1].LL.y, plane[i + 1].LL.z};
    }
    if (plane[i].UR.y < plane[i + 1].UR.y) {
      R[i] = {plane[i].UR.y, plane[i].UR.z};
    } else {
      R[i] = {plane[i + 1].UR.y, plane[i + 1].UR.z};
    }
  }
  auto left_slope = -INF;
  auto right_slope = INF;
  auto res = INF;
  auto isCoplanar = [](const Point3D &a, const Point3D &b, const Point3D &c,
                       const Point3D &d) -> bool {
    auto p1 = a - b;
    auto p2 = a - c;
    auto p3 = a - d;
    return std::abs(dot(p1, cross(p2, p3))) < EPS;
  };
  auto isPointInsideRectangle = [&isCoplanar](const Point3D &p,
                                              const Rectangle &r) -> bool {
    if (isCoplanar(r.LL, r.LR, r.UR, p)) {
      const auto p1 = point_projection(p, {r.LR, r.LL});
      const auto p2 = point_projection(p, {r.LR, r.UR});
      return std::abs(distance(p1, r.LR) + distance(p1, r.LL) -
                      distance(r.LL, r.LR)) < EPS &&
             std::abs(distance(p2, r.LR) + distance(p2, r.UR) -
                      distance(r.UR, r.LR)) < EPS;
    }
    return false;
  };
  int start_rectangle =
      std::ranges::find_if(plane.begin(), plane.end(),
                           [&start, &isPointInsideRectangle](const auto &r) {
                             return isPointInsideRectangle(start, r);
                           }) -
      plane.begin();
  int end_rectangle =
      std::ranges::find_if(plane.begin(), plane.end(),
                           [&end, &isPointInsideRectangle](const auto &r) {
                             return isPointInsideRectangle(end, r);
                           }) -
      plane.begin();
  for (int i = start_rectangle; i < end_rectangle; ++i) {
    if (i == start_rectangle && start.x == plane[i].UR.x) {
      dp[i][0] = distance(start, {plane[i].UR.x, L[i]});
      dp[i][1] = distance(start, {plane[i].UR.x, R[i]});
      if (start.y < plane[i].LL.y || start.y > plane[i].UR.y) {
        left_slope = INF, right_slope = -INF;
        break;
      }
    }
    auto l = calc_slope(start, {plane[i].UR.x, L[i]}, x_axis);
    auto r = calc_slope(start, {plane[i].UR.x, R[i]}, x_axis);
    if (left_slope < l + EPS && right_slope + EPS > l) {
      dp[i][0] = distance(start, {plane[i].UR.x, L[i]});
    }
    if (left_slope < r + EPS && right_slope + EPS > r) {
      dp[i][1] = distance(start, {plane[i].UR.x, R[i]});
    }
    left_slope = max(left_slope, l), right_slope = min(right_slope, r);
  }
  auto k = calc_slope(start, end, x_axis);
  if (left_slope < k + EPS && right_slope + EPS > k) {
    res = distance(start, end);
  }
  auto update_res = [&res, &end, &path](const auto dp_value,
                                        decltype(end) &p) -> void {
    if (auto value = dp_value + distance(p, end); res > value) {
      res = value;
      path.emplace_back(p);
    }
  };
  for (int i = start_rectangle; i < end_rectangle; ++i) {
    auto left_k1 = -INF;
    auto right_k1 = INF;
    auto left_k2 = -INF;
    auto right_k2 = INF;
    for (int j = i + 1; j < end_rectangle; ++j) {
      auto l = calc_slope({plane[i].UR.x, L[i]}, {plane[j].UR.x, L[j]}, x_axis);
      auto r = calc_slope({plane[i].UR.x, L[i]}, {plane[j].UR.x, R[j]}, x_axis);
      if (left_k1 < l + EPS && right_k1 + EPS > l) {
        dp[j][0] = min(
            dp[i][0] + distance({plane[i].UR.x, L[i]}, {plane[j].UR.x, L[j]}),
            dp[j][0]);
      }
      if (left_k1 < r + EPS && right_k1 + EPS > r) {
        dp[j][1] = min(
            dp[i][0] + distance({plane[i].UR.x, L[i]}, {plane[j].UR.x, R[j]}),
            dp[j][1]);
      }
      left_k1 = max(left_k1, l), right_k1 = min(right_k1, r);
      l = calc_slope({plane[i].UR.x, R[i]}, {plane[j].UR.x, L[j]}, x_axis),
      r = calc_slope({plane[i].UR.x, R[i]}, {plane[j].UR.x, R[j]}, x_axis);
      if (left_k2 < l + EPS && right_k2 + EPS > l) {
        dp[j][0] = min(
            dp[i][1] + distance({plane[i].UR.x, R[i]}, {plane[j].UR.x, L[j]}),
            dp[j][0]);
      }
      if (left_k2 < r + EPS && right_k2 + EPS > r) {
        dp[j][1] = min(
            dp[i][1] + distance({plane[i].UR.x, R[i]}, {plane[j].UR.x, R[j]}),
            dp[j][1]);
      }
      left_k2 = max(left_k2, l), right_k2 = min(right_k2, r);
    }
    if (i == end_rectangle - 1 && end.x == plane[end_rectangle].LL.x) {
      if (end.y < plane[end_rectangle].LL.y ||
          end.y > plane[end_rectangle].UR.y) {
        continue;
      }
      update_res(dp[i][0], {plane[i].UR.x, L[i]});
      update_res(dp[i][1], {plane[i].UR.x, R[i]});
      continue;
    }
    auto l = calc_slope({plane[i].UR.x, L[i]}, end, x_axis);
    auto r = calc_slope({plane[i].UR.x, R[i]}, end, x_axis);
    if (left_k1 < l + EPS && right_k1 + EPS > l) {
      update_res(dp[i][0], {plane[i].UR.x, L[i]});
    }
    if (left_k2 < r + EPS && right_k2 + EPS > r) {
      update_res(dp[i][1], {plane[i].UR.x, R[i]});
    }
  }
  path.emplace_back(end);
  return {path, res};
}
} // namespace Basic
