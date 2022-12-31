#include <array>
#include <cassert>

#include "basic.h"

using std::max;
using std::min;

namespace Basic {
auto solve(const Plane &plane, const Point &start, const Point &end)
    -> std::pair<Path, double> {
  const auto INF = 1e11, EPS = 1e-9;
  if (start.x == end.x) {
    return {{start, end}, distance(start, end)};
  }
  assert(start.x < end.x);
  int n = plane.size();
  vector<double> L(n), R(n);
  vector<std::array<double, 2>> dp(n + 1, {INF, INF});
  Path path{start};
  for (int i = 0; i < n - 1; ++i) {
    L[i] = max(plane[i].UL.y, plane[i + 1].UL.y);
    R[i] = min(plane[i].LR.y, plane[i + 1].LR.y);
  }
  auto left_slope = -INF;
  auto right_slope = INF;
  auto res = INF;
  int start_rectangle, end_rectangle;
  for (int i = n - 1; i >= 0; --i) {
    if (plane[i].UL.x <= start.x && plane[i].LR.x >= start.x &&
        plane[i].UL.y <= start.y && plane[i].LR.y >= start.y) {
      start_rectangle = i;
    }
    if (plane[i].UL.x <= end.x && plane[i].LR.x >= end.x &&
        plane[i].UL.y <= end.y && plane[i].LR.y >= end.y) {
      end_rectangle = i;
    }
  }
  for (int i = start_rectangle; i < end_rectangle; ++i) {
    if (i == start_rectangle && start.x == plane[i].LR.x) {
      dp[i][0] = distance(start, {plane[i].LR.x, L[i]});
      dp[i][1] = distance(start, {plane[i].LR.x, R[i]});
      if (start.y < plane[i].UL.y || start.y > plane[i].LR.y) {
        left_slope = INF, right_slope = -INF;
        break;
      }
    }
    auto l = calc_slope(start, {plane[i].LR.x, L[i]});
    auto r = calc_slope(start, {plane[i].LR.x, R[i]});
    if (left_slope < l + EPS && right_slope + EPS > l) {
      dp[i][0] = distance(start, {plane[i].LR.x, L[i]});
    }
    if (left_slope < r + EPS && right_slope + EPS > r) {
      dp[i][1] = distance(start, {plane[i].LR.x, R[i]});
    }
    left_slope = max(left_slope, l), right_slope = min(right_slope, r);
  }
  auto k = calc_slope(start, end);
  if (left_slope < k + EPS && right_slope + EPS > k) {
    res = distance(start, end);
  }
  auto update_res = [&res, &end, &path](const double dp_value,
                                        const Point &p) -> void {
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
      auto l = calc_slope({plane[i].LR.x, L[i]}, {plane[j].LR.x, L[j]});
      auto r = calc_slope({plane[i].LR.x, L[i]}, {plane[j].LR.x, R[j]});
      if (left_k1 < l + EPS && right_k1 + EPS > l) {
        dp[j][0] = min(
            dp[i][0] + distance({plane[i].LR.x, L[i]}, {plane[j].LR.x, L[j]}),
            dp[j][0]);
      }
      if (left_k1 < r + EPS && right_k1 + EPS > r) {
        dp[j][1] = min(
            dp[i][0] + distance({plane[i].LR.x, L[i]}, {plane[j].LR.x, R[j]}),
            dp[j][1]);
      }
      left_k1 = max(left_k1, l), right_k1 = min(right_k1, r);
      l = calc_slope({plane[i].LR.x, R[i]}, {plane[j].LR.x, L[j]}),
      r = calc_slope({plane[i].LR.x, R[i]}, {plane[j].LR.x, R[j]});
      if (left_k2 < l + EPS && right_k2 + EPS > l) {
        dp[j][0] = min(
            dp[i][1] + distance({plane[i].LR.x, R[i]}, {plane[j].LR.x, L[j]}),
            dp[j][0]);
      }
      if (left_k2 < r + EPS && right_k2 + EPS > r) {
        dp[j][1] = min(
            dp[i][1] + distance({plane[i].LR.x, R[i]}, {plane[j].LR.x, R[j]}),
            dp[j][1]);
      }
      left_k2 = max(left_k2, l), right_k2 = min(right_k2, r);
    }
    if (i == end_rectangle - 1 && end.x == plane[end_rectangle].UL.x) {
      if (end.y < plane[end_rectangle].UL.y ||
          end.y > plane[end_rectangle].LR.y) {
        continue;
      }
      update_res(dp[i][0], {plane[i].LR.x, L[i]});
      update_res(dp[i][1], {plane[i].LR.x, R[i]});
      // res = min(res, dp[i][0] + distance({plane[i].LR.x, L[i]}, end));
      // res = min(res, dp[i][1] + distance({plane[i].LR.x, R[i]}, end));
      continue;
    }
    auto l = calc_slope({plane[i].LR.x, L[i]}, end);
    auto r = calc_slope({plane[i].LR.x, R[i]}, end);
    if (left_k1 < l + EPS && right_k1 + EPS > l) {
      update_res(dp[i][0], {plane[i].LR.x, L[i]});
      // res = min(res, dp[i][0] + distance({plane[i].LR.x, L[i]}, end));
    }
    if (left_k2 < r + EPS && right_k2 + EPS > l) {
      update_res(dp[i][1], {plane[i].LR.x, R[i]});
      // res = min(res, dp[i][1] + distance({plane[i].LR.x, R[i]}, end));
    }
  }
  path.emplace_back(end);
  return {path, res};
}
} // namespace Basic
