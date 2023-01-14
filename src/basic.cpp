#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>

#include "basic.h"

using std::max;
using std::min;

namespace Basic {
auto solve(const vector<Rectangle> &rectangles, const Point &start,
           const Point &end) -> std::pair<Path, double> {
  constexpr auto INF = std::numeric_limits<double>::max(), EPS = 1e-9;
  int n = rectangles.size();
  vector<double> L(n), R(n);
  vector<std::array<double, 2>> dp(n + 1, {INF, INF});
  Path path{start};
  for (int i = 0; i < n - 1; ++i) {
    L[i] = max(rectangles[i].LL.y, rectangles[i + 1].LL.y);
    R[i] = min(rectangles[i].UR.y, rectangles[i + 1].UR.y);
  }
  auto left_slope = -INF;
  auto right_slope = INF;
  auto res = INF;
  int start_rectangle =
      std::ranges::find_if(rectangles.begin(), rectangles.end(),
                           [&start](const auto &r) {
                             return isPointInsideRectangle(start, r);
                           }) -
      rectangles.begin();
  assert(start_rectangle < n);
  int end_rectangle =
      std::ranges::find_if(
          rectangles.begin(), rectangles.end(),
          [&end](const auto &r) { return isPointInsideRectangle(end, r); }) -
      rectangles.begin();
  assert(end_rectangle < n);
  for (int i = start_rectangle; i < end_rectangle; ++i) {
    if (i == start_rectangle && start.x == rectangles[i].UR.x) {
      dp[i][0] = distance(start, {rectangles[i].UR.x, L[i]});
      dp[i][1] = distance(start, {rectangles[i].UR.x, R[i]});
      if (start.y < rectangles[i].LL.y || start.y > rectangles[i].UR.y) {
        left_slope = INF, right_slope = -INF;
        break;
      }
    }
    auto l = calc_slope(start, {rectangles[i].UR.x, L[i]});
    auto r = calc_slope(start, {rectangles[i].UR.x, R[i]});
    if (left_slope < l + EPS && right_slope + EPS > l) {
      dp[i][0] = distance(start, {rectangles[i].UR.x, L[i]});
    }
    if (left_slope < r + EPS && right_slope + EPS > r) {
      dp[i][1] = distance(start, {rectangles[i].UR.x, R[i]});
    }
    left_slope = max(left_slope, l), right_slope = min(right_slope, r);
  }
  auto k = calc_slope(start, end);
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
      auto l =
          calc_slope({rectangles[i].UR.x, L[i]}, {rectangles[j].UR.x, L[j]});
      auto r =
          calc_slope({rectangles[i].UR.x, L[i]}, {rectangles[j].UR.x, R[j]});
      if (left_k1 < l + EPS && right_k1 + EPS > l) {
        dp[j][0] = min(dp[i][0] + distance({rectangles[i].UR.x, L[i]},
                                           {rectangles[j].UR.x, L[j]}),
                       dp[j][0]);
      }
      if (left_k1 < r + EPS && right_k1 + EPS > r) {
        dp[j][1] = min(dp[i][0] + distance({rectangles[i].UR.x, L[i]},
                                           {rectangles[j].UR.x, R[j]}),
                       dp[j][1]);
      }
      left_k1 = max(left_k1, l), right_k1 = min(right_k1, r);
      l = calc_slope({rectangles[i].UR.x, R[i]}, {rectangles[j].UR.x, L[j]}),
      r = calc_slope({rectangles[i].UR.x, R[i]}, {rectangles[j].UR.x, R[j]});
      if (left_k2 < l + EPS && right_k2 + EPS > l) {
        dp[j][0] = min(dp[i][1] + distance({rectangles[i].UR.x, R[i]},
                                           {rectangles[j].UR.x, L[j]}),
                       dp[j][0]);
      }
      if (left_k2 < r + EPS && right_k2 + EPS > r) {
        dp[j][1] = min(dp[i][1] + distance({rectangles[i].UR.x, R[i]},
                                           {rectangles[j].UR.x, R[j]}),
                       dp[j][1]);
      }
      left_k2 = max(left_k2, l), right_k2 = min(right_k2, r);
    }
    if (i == end_rectangle - 1 && end.x == rectangles[end_rectangle].LL.x) {
      if (end.y < rectangles[end_rectangle].LL.y ||
          end.y > rectangles[end_rectangle].UR.y) {
        continue;
      }
      update_res(dp[i][0], {rectangles[i].UR.x, L[i]});
      update_res(dp[i][1], {rectangles[i].UR.x, R[i]});
      continue;
    }
    auto l = calc_slope({rectangles[i].UR.x, L[i]}, end);
    auto r = calc_slope({rectangles[i].UR.x, R[i]}, end);
    if (left_k1 < l + EPS && right_k1 + EPS > l) {
      update_res(dp[i][0], {rectangles[i].UR.x, L[i]});
    }
    if (left_k2 < r + EPS && right_k2 + EPS > r) {
      update_res(dp[i][1], {rectangles[i].UR.x, R[i]});
    }
  }
  path.emplace_back(end);
  return {path, res};
}

auto solve3D(const Plane &plane, const Point3D &start, const Point3D &end)
    -> std::pair<Path3D, double> {
  auto plane_copy = plane;
  const auto move = plane_copy[0].LL;
  if (plane_copy[0].LL.x != 0 || plane_copy[0].LL.y != 0 ||
      plane_copy[0].LL.z != 0) {
    for (auto &i : plane_copy) {
      i.LL = i.LL - move;
      i.LR = i.LR - move;
      i.UR = i.UR - move;
    }
  }
  for (auto it = plane_copy.begin(); it != plane_copy.end(); ++it) {
    if (it->LR.y != it->LL.y) {
      // y * cos(theta) - z * sin(theta) = it->LL.y
      double theta;
      const auto a = it->LR.y, b = it->LR.z, c = it->LL.y;
      auto EPS = 1e-9;
      if (std::abs(a + c) > EPS &&
          std::abs(a * a + a * c + b * b -
                   b * std::sqrt(a * a + b * b - c * c)) > EPS) {
        theta = 2 * std::atan((std::sqrt(a * a + b * b - c * c) - b) / (a + c));
      } else if (std::abs(a + c) > EPS &&
                 std::abs(b * std::sqrt(a * a + b * b - c * c) + a * a + a * c +
                          b * b) > EPS) {
        theta = 2 * std::atan((-sqrt(a * a + b * b - c * c) - b) / (a + c));
      } else if (std::abs(b) > EPS && a * a + b * b > EPS &&
                 std::abs(a + c) <= EPS) {
        theta = 2 * std::atan(a / b);
      } else {
        // ERROR!
        theta = 0;
      }
      auto update_y = [=](double &y, double z) {
        y = y * std::cos(theta) - z * std::sin(theta);
      };
      auto update_z = [=](double y, double &z) {
        z = y * std::sin(theta) + z * std::cos(theta);
      };
      auto update = [&](Rectangle3D &r) {
        auto tmp_y = r.LL.y;
        update_y(r.LL.y, r.LL.z);
        update_z(tmp_y, r.LL.z);
        tmp_y = r.LR.y;
        update_y(r.LR.y, r.LR.z);
        update_z(tmp_y, r.LR.z);
        tmp_y = r.UR.y;
        update_y(r.UR.y, r.UR.z);
        update_z(tmp_y, r.UR.z);
      };
      std::for_each(it, plane_copy.end(), update);
    }
    if (it->LR.z != 0) {
      auto theta = std::atan(it->LR.z / it->LR.x);
      if (it->LR.x * std::sin(theta) + it->LR.x * std::cos(theta) < it->LL.x) {
        theta = -theta;
      }
      auto update_x = [=](double &x) {
        // Notice: This is NOT bug
        x = x * std::sin(theta) + x * std::cos(theta);
      };
      auto update_z = [=](double x, double &z) {
        z = z * std::cos(theta) - x * std::sin(theta);
      };
      auto update = [&](Rectangle3D &r) {
        update_z(r.LL.x, r.LL.z);
        update_x(r.LL.x);
        update_z(r.LR.x, r.LR.z);
        update_x(r.LR.x);
        update_z(r.UR.x, r.UR.z);
        update_x(r.UR.x);
      };
      std::for_each(it, plane_copy.end(), update);
    }
  }
  auto move_point = [](const Point3D &p, const Rectangle3D &from,
                       const Rectangle3D &to) {
    auto va = from.LL - from.LR, vb = from.UR - from.LR, vc = p - from.LR;
    auto a = (vc.y * vb.z - vb.y * vc.z) / (va.y * vb.z - vb.y * va.z);
    if (std::isnan(a)) {
      a = (vc.x * vb.y - vb.x * vc.y) / (va.x * vb.y - vb.x * va.y);
    }
    if (std::isnan(a)) {
      a = (vc.x * vb.z - vb.x * vc.z) / (va.x * vb.z - vb.x * va.z);
    }
    auto b = (va.y * vc.z - vc.y * va.z) / (va.y * vb.z - vb.y * va.z);
    if (std::isnan(b)) {
      b = (va.x * vc.y - vc.x * va.y) / (va.x * vb.y - vb.x * va.y);
    }
    if (std::isnan(b)) {
      b = (va.x * vc.z - vc.x * va.z) / (va.x * vb.z - vb.x * va.z);
    }
    return (to.LL - to.LR) * a + (to.UR - to.LR) * b + to.LR;
  };
  auto start_rectangle =
      std::find_if(
          plane.begin(), plane.end(),
          [&](const auto &r) { return isPointInsideRectangle3D(start, r); }) -
      plane.begin();
  auto start_copy =
      move_point(start, plane[start_rectangle], plane_copy[start_rectangle]);
  auto end_rectangle = std::find_if(plane.begin(), plane.end(),
                                    [&](const auto &r) {
                                      return isPointInsideRectangle3D(end, r);
                                    }) -
                       plane.begin();
  auto end_copy =
      move_point(end, plane[end_rectangle], plane_copy[end_rectangle]);
  vector<Rectangle> rectangles(plane.size());
  for (int i = 0; i < plane.size(); ++i) {
    rectangles[i] = {{plane_copy[i].LL.x, plane_copy[i].LL.y},
                     {plane_copy[i].UR.x, plane_copy[i].UR.y}};
  }
  const Point s = {start_copy.x, start_copy.y};
  const Point e = {end_copy.x, end_copy.y};
  auto [path, distance] = solve(rectangles, s, e);
  Path3D path3D(path.size());
  for (int i = 0; i < path.size(); ++i) {
    // '&path = path' is an error of clang++
    auto tmp = std::find_if(rectangles.begin(), rectangles.end(),
                            [&path = path, i = i](const auto &r) {
                              return isPointInsideRectangle(path[i], r);
                            }) -
               rectangles.begin();
    path3D[i] =
        move_point({path[i].x, path[i].y, 0}, plane_copy[tmp], plane[tmp]);
  }
  return {path3D, distance};
}
} // namespace Basic
