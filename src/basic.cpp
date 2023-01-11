#include <algorithm>
#include <array>
#include <cassert>
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
  auto isPointInsideRectangle = [](const Point &p, const Rectangle &r) -> bool {
    return p.x >= r.LL.x && p.x <= r.UR.x && p.y >= r.LL.y && p.y <= r.UR.y;
  };
  int start_rectangle =
      std::ranges::find_if(rectangles.begin(), rectangles.end(),
                           [&start, &isPointInsideRectangle](const auto &r) {
                             return isPointInsideRectangle(start, r);
                           }) -
      rectangles.begin();
  assert(start_rectangle < n);
  int end_rectangle =
      std::ranges::find_if(rectangles.begin(), rectangles.end(),
                           [&end, &isPointInsideRectangle](const auto &r) {
                             return isPointInsideRectangle(end, r);
                           }) -
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

// TODO!
auto solve3D(const Plane &plane, const Point3D &start, const Point3D &end)
    -> std::pair<Path3D, double> {
  vector<Rectangle> rectangles(plane.size());
  for (int i = 0; i < plane.size(); ++i) {
    rectangles[i] = {{plane[i].LL.x, plane[i].LL.y},
                     {plane[i].UR.x, plane[i].UR.y}};
  }
  const Point s = {start.x, start.y};
  const Point e = {end.x, end.y};
  auto [path, distance] = solve(rectangles, s, e);
  Path3D path3D(path.size());
  for (int i = 0; i < path.size(); ++i) {
    path3D[i] = {path[i].x, path[i].y, 0};
  }
  return {path3D, distance};
}
} // namespace Basic
