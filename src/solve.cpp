#include <algorithm>
#include <array>
#include <bits/ranges_algo.h>
#include <cassert>
#include <cmath>
#include <limits>

#include "solve.h"

using std::abs;
using std::atan;
using std::cos;
using std::isnan;
using std::max;
using std::min;
using std::sin;
using std::sqrt;
using std::ranges::find_if;
using std::ranges::for_each;

using bg::distance;

namespace Solve {
auto calcOverlapLine(const Line3D &l1, const Line3D &l2) -> Line3D {
  if (!arePointsCollinear(l1.first, l1.second, l2.first, l2.second) ||
      l1.first == l1.second || l2.first == l2.second) {
    return {};
  }

  vector<Point3D> overlaps_points;
  for (const auto &p : {l1.first, l1.second, l2.first, l2.second}) {
    auto k = (p.x - l1.first.x) / (l1.second.x - l1.first.x);
    if (std::isnan(k)) {
      k = (p.y - l1.first.y) / (l1.second.y - l1.first.y);
    }
    if (std::isnan(k)) {
      k = (p.z - l1.first.z) / (l1.second.z - l1.first.z);
    }
    if (k > -EPS && k < 1 + EPS) {
      k = (p.x - l2.first.x) / (l2.second.x - l2.first.x);
      if (std::isnan(k)) {
        k = (p.y - l2.first.y) / (l2.second.y - l2.first.y);
      }
      if (std::isnan(k)) {
        k = (p.z - l2.first.z) / (l2.second.z - l2.first.z);
      }
      if (k > -EPS && k < 1 + EPS) {
        overlaps_points.emplace_back(p);
      }
    }
  }

  if (overlaps_points.size() > 1) {
    return {overlaps_points[0], overlaps_points[1]};
  }
  return {};
}

auto calcOverlapRectangle(const Rectangle3D &r1, const Rectangle3D &r2)
    -> Line3D {
  std::array<Point3D, 4> points1{r1.LL, r1.LR, r1.UR, r1.LL + r1.UR - r1.LR};
  std::array<Point3D, 4> points2{r2.LL, r2.LR, r2.UR, r2.LL + r2.UR - r2.LR};

  auto isEdgeOverlap = [](const Point3D &p1, const Point3D &q1,
                          const Point3D &p2, const Point3D &q2) {
    auto line = calcOverlapLine(Line3D(p1, q1), Line3D(p2, q2));
    return distance(line.first, line.second) > EPS;
  };

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (isEdgeOverlap(points1[i], points1[(i + 1) % 4], points2[j],
                        points2[(j + 1) % 4])) {
        return calcOverlapLine(Line3D(points1[i], points1[(i + 1) % 4]),
                               Line3D(points2[j], points2[(j + 1) % 4]));
      }
    }
  }
  return {};
}

auto solve(const vector<Rectangle> &rectangles, Point start, Point end)
    -> std::pair<Path, double> {
  constexpr auto INF = std::numeric_limits<double>::max();
  const int n = rectangles.size();
  vector<double> LOW(n);
  vector<double> HIGH(n);
  vector<std::array<double, 2>> dp(n + 1, {INF, INF});

  for (int i = 0; i < n - 1; ++i) {
    LOW[i] = max(rectangles[i].LL.y, rectangles[i + 1].LL.y);
    HIGH[i] = min(rectangles[i].UR.y, rectangles[i + 1].UR.y);
  }

  auto left_slope = -INF;
  auto right_slope = INF;
  auto res = INF;

  const int start_rectangle = find_if(rectangles.begin(), rectangles.end(),
                                      [&start](const auto &r) {
                                        return isPointInsideRectangle(start, r);
                                      }) -
                              rectangles.begin();
  assert(start_rectangle < n);

  const int end_rectangle = find_if(rectangles.begin(), rectangles.end(),
                                    [&end](const auto &r) {
                                      return isPointInsideRectangle(end, r);
                                    }) -
                            rectangles.begin();
  assert(end_rectangle < n);

  std::array<Path, 2> path;
  Path res_path;

  auto add_path = [&](Path &path, const Point &from, const Point &to) {
    std::input_iterator auto from_rectangle =
        find_if(rectangles.begin(), rectangles.end(),
                [&](const auto &r) { return isPointInsideRectangle(from, r); });
    std::input_iterator auto to_rectangle =
        find_if(rectangles.begin(), rectangles.end(),
                [&](const auto &r) { return isPointInsideRectangle(to, r); });
    if (from_rectangle->UR.x != from.x) {
      path.emplace_back(from);
    }
    for_each(from_rectangle, to_rectangle, [&](const auto &r) {
      const auto [x1, y1] = from;
      const auto [x2, y2] = to;
      const auto x = r.UR.x;
      path.push_back(Point(x, (x - x2) / (x1 - x2) * (y1 - y2) + y2));
    });
    path.emplace_back(to);
  };

  for (int i = start_rectangle; i < end_rectangle; ++i) {
    if (i == start_rectangle && abs(start.x - rectangles[i].UR.x) < EPS) {
      dp[i][0] = distance(start, Point(rectangles[i].UR.x, LOW[i]));
      add_path(path[0], start, Point(rectangles[i].UR.x, LOW[i]));
      dp[i][1] = distance(start, Point(rectangles[i].UR.x, HIGH[i]));
      add_path(path[1], start, Point(rectangles[i].UR.x, HIGH[i]));
      if (start.y - rectangles[i].LL.y < EPS ||
          rectangles[i].UR.y - start.y < EPS) {
        left_slope = INF, right_slope = -INF;
        break;
      }
    }

    auto l = calc_slope(start, Point(rectangles[i].UR.x, LOW[i]));
    auto r = calc_slope(start, Point(rectangles[i].UR.x, HIGH[i]));

    if (left_slope < l + EPS && right_slope + EPS > l) {
      dp[i][0] = distance(start, Point(rectangles[i].UR.x, LOW[i]));
      add_path(path[0], start, Point(rectangles[i].UR.x, LOW[i]));
    }
    if (left_slope < r + EPS && right_slope + EPS > r) {
      dp[i][1] = distance(start, Point(rectangles[i].UR.x, HIGH[i]));
      add_path(path[1], start, Point(rectangles[i].UR.x, HIGH[i]));
    }

    left_slope = max(left_slope, l), right_slope = min(right_slope, r);
  }

  auto k = calc_slope(start, end);
  if (left_slope < k + EPS && right_slope + EPS > k) {
    res = distance(start, end);
    res_path.clear();
    add_path(res_path, start, end);
    return {res_path, res};
  }

  auto update_dp = [&](double &dp_value, const double length, const Point from,
                       const Point to, const unsigned index) {
    auto tmp = length + distance(from, to);
    if (dp_value > tmp) {
      dp_value = tmp;
      if (std::input_iterator auto it = find_if(
              path[index].begin(), path[index].end(),
              [&](const auto &p) {
                return abs(p.x - from.x) < EPS && abs(p.y - from.y) < EPS;
              });
          it != path[index].end()) {
        path[index].erase(it, path[index].end());
        add_path(path[index], from, to);
      }
    }
  };

  auto update_res = [&](double &res, const double length, const Point from,
                        const unsigned index) {
    auto tmp = length + distance(from, end);
    if (res > tmp) {
      res = tmp;
      std::input_iterator auto it =
          find_if(path[index].begin(), path[index].end(), [&](const auto &p) {
            return abs(p.x - from.x) < EPS && abs(p.y - from.y) < EPS;
          });
      res_path.assign(path[index].begin(), it);
      add_path(res_path, *it, end);
    }
  };

  for (int i = start_rectangle; i < end_rectangle; ++i) {
    auto left_k1 = -INF;
    auto right_k1 = INF;
    auto left_k2 = -INF;
    auto right_k2 = INF;

    for (int j = i + 1; j < end_rectangle; ++j) {
      auto l = calc_slope(Point(rectangles[i].UR.x, LOW[i]),
                          Point(rectangles[j].UR.x, LOW[j]));
      auto r = calc_slope(Point(rectangles[i].UR.x, LOW[i]),
                          Point(rectangles[j].UR.x, HIGH[j]));

      if (left_k1 < l + EPS && right_k1 + EPS > l) {
        update_dp(dp[j][0], dp[i][0], Point(rectangles[i].UR.x, LOW[i]),
                  Point(rectangles[j].UR.x, LOW[j]), 0);
      }
      if (left_k1 < r + EPS && right_k1 + EPS > r) {
        update_dp(dp[j][1], dp[i][0], Point(rectangles[i].UR.x, LOW[i]),
                  Point(rectangles[j].UR.x, HIGH[j]), 1);
      }
      left_k1 = max(left_k1, l), right_k1 = min(right_k1, r);

      l = calc_slope(Point(rectangles[i].UR.x, HIGH[i]),
                     Point(rectangles[j].UR.x, LOW[j])),
      r = calc_slope(Point(rectangles[i].UR.x, HIGH[i]),
                     Point(rectangles[j].UR.x, HIGH[j]));

      if (left_k2 < l + EPS && right_k2 + EPS > l) {
        update_dp(dp[j][0], dp[i][1], Point(rectangles[i].UR.x, HIGH[i]),
                  Point(rectangles[j].UR.x, LOW[j]), 0);
      }
      if (left_k2 < r + EPS && right_k2 + EPS > r) {
        update_dp(dp[j][1], dp[i][1], Point(rectangles[i].UR.x, HIGH[i]),
                  Point(rectangles[j].UR.x, HIGH[j]), 1);
      }
      left_k2 = max(left_k2, l), right_k2 = min(right_k2, r);
    }

    if (i == end_rectangle - 1 &&
        abs(end.x - rectangles[end_rectangle].LL.x) < EPS) {
      if (end.y - rectangles[end_rectangle].LL.y < EPS ||
          rectangles[end_rectangle].UR.y - end.y < EPS) {
        continue;
      }

      update_res(res, dp[i][0], Point(rectangles[i].UR.x, LOW[i]), 0);
      update_res(res, dp[i][1], Point(rectangles[i].UR.x, HIGH[i]), 1);
      continue;
    }

    auto l = calc_slope(Point(rectangles[i].UR.x, LOW[i]), end);
    auto r = calc_slope(Point(rectangles[i].UR.x, HIGH[i]), end);

    if (left_k1 < l + EPS && right_k1 + EPS > l) {
      update_res(res, dp[i][0], Point(rectangles[i].UR.x, LOW[i]), 0);
    }
    if (left_k2 < r + EPS && right_k2 + EPS > r) {
      update_res(res, dp[i][1], Point(rectangles[i].UR.x, HIGH[i]), 1);
    }
  }
  return {res_path, res};
}

auto solve3D(Plane &plane, const Point3D &start, const Point3D &end)
    -> std::pair<Path3D, double> {
  if (plane.size() > 1) {
    auto &base_r = plane.front();
    const auto base_line = calcOverlapRectangle(base_r, plane.at(1));
    Line3D last_line;
    {
      const auto &r = plane.front();
      const std::array<Point3D, 4> ps{r.LL, r.LR, r.UR, r.LL + r.UR - r.LR};
      for (int i = 0; i < 4; ++i) {
        if (const auto line = Line3D(ps[i], ps[(i + 1) % 4]);
            isParallel(line, base_line) && !isSameLine(line, base_line)) {
          last_line = line;
          break;
        }
      }
    }
    for (auto &r : plane) {
      const std::array<Point3D, 4> ps{r.LL, r.LR, r.UR, r.LL + r.UR - r.LR};
      for (int i = 0; i < 4; ++i) {
        if (const auto line = Line3D(ps[i], ps[(i + 1) % 4]);
            isParallel(line, base_line) && !isSameLine(line, last_line)) {
          last_line = line;
          Rectangle3D tmp;
          if (isSameAxis(Line3D(ps[i], ps[(i + 1) % 4]), base_line)) {
            tmp.LR = ps[i];
            tmp.UR = ps[(i + 1) % 4];
          } else {
            tmp.LR = ps[(i + 1) % 4];
            tmp.UR = ps[i];
          }
          if (distance(ps[(i + 2) % 4], tmp.LR) <
              distance(ps[(i + 3) % 4], tmp.LR)) {
            tmp.LL = ps[(i + 2) % 4];
          } else {
            tmp.LL = ps[(i + 3) % 4];
          }
          r = tmp;
          break;
        }
      }
    }
  }

  auto plane_copy = plane;

  for (auto it = plane_copy.begin(); it != plane_copy.end(); ++it) {
    const auto move = it->LL;
    if (abs(move.x) > EPS || abs(move.y) > EPS || abs(move.z) > EPS) {
      for_each(plane_copy.begin(), plane_copy.end(), [&](auto &r) {
        r.LL = r.LL - move;
        r.LR = r.LR - move;
        r.UR = r.UR - move;
      });
    }

    if (it->LR.y != it->LL.y) {
      // y * cos(theta) - z * sin(theta) = it->LL.y
      double theta = NAN;
      const auto a = it->LR.y;
      const auto b = it->LR.z;
      const auto c = it->LL.y;

      if (abs(a + c) > EPS &&
          abs(a * a + a * c + b * b - b * sqrt(a * a + b * b - c * c)) > EPS) {
        theta = 2 * atan((sqrt(a * a + b * b - c * c) - b) / (a + c));
      } else if (abs(a + c) > EPS && abs(b * sqrt(a * a + b * b - c * c) +
                                         a * a + a * c + b * b) > EPS) {
        theta = 2 * atan((-sqrt(a * a + b * b - c * c) - b) / (a + c));
      } else if (abs(b) > EPS && a * a + b * b > EPS && abs(a + c) <= EPS) {
        theta = 2 * atan(a / b);
      } else {
        // ERROR!
        theta = 0;
      }

      auto update_y = [=](double &y, double z) {
        y = y * cos(theta) - z * sin(theta);
      };
      auto update_z = [=](double y, double &z) {
        z = y * sin(theta) + z * cos(theta);
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

      for_each(it, plane_copy.end(), update);
    }

    if (it->LR.z != 0) {
      auto theta = atan(it->LR.z / it->LR.x);
      if (it->LR.x * sin(theta) + it->LR.x * cos(theta) < it->LL.x) {
        theta = -theta;
      }

      auto update_x = [=](double &x, double z) {
        x = z * sin(theta) + x * cos(theta);
      };
      auto update_z = [=](double x, double &z) {
        z = z * cos(theta) - x * sin(theta);
      };
      auto update = [&](Rectangle3D &r) {
        auto tmp_x = r.LL.x;
        update_x(r.LL.x, r.LL.z);
        update_z(tmp_x, r.LL.z);
        tmp_x = r.LR.x;
        update_x(r.LR.x, r.LR.z);
        update_z(tmp_x, r.LR.z);
        tmp_x = r.UR.x;
        update_x(r.UR.x, r.UR.z);
        update_z(tmp_x, r.UR.z);
      };

      for_each(it, plane_copy.end(), update);
    }

    if (it->UR.z != 0) {
      auto theta = atan(it->UR.z / it->UR.y);

      auto update_y = [=](double &y, double z) {
        y = y * cos(theta) - z * sin(theta);
      };
      auto update_z = [=](double y, double &z) {
        z = y * sin(theta) + z * cos(theta);
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

      for_each(it, plane_copy.end(), update);
    }

    if (it->LR.x < 0) {
      auto update = [&](Rectangle3D &r) {
        r.LL.x = -r.LL.x;
        r.LR.x = -r.LR.x;
        r.UR.x = -r.UR.x;
      };

      for_each(it, plane_copy.end(), update);
    }

    if (it->UR.y < 0) {
      auto update = [&](Rectangle3D &r) {
        r.LL.y = -r.LL.y;
        r.LR.y = -r.LR.y;
        r.UR.y = -r.UR.y;
      };

      for_each(it, plane_copy.end(), update);
    }
  }

  auto move_point = [](const Point3D &p, const Rectangle3D &from,
                       const Rectangle3D &to) {
    // a * A + b * B = C
    auto A = from.LL - from.LR;
    auto B = from.UR - from.LR;
    auto C = p - from.LR;

    auto a = (C.x * B.y - B.x * C.y) / (A.x * B.y - B.x * A.y);
    if (isnan(a)) {
      a = (C.y * B.z - B.y * C.z) / (A.y * B.z - B.y * A.z);
    }
    if (isnan(a)) {
      a = (C.x * B.z - B.x * C.z) / (A.x * B.z - B.x * A.z);
    }

    auto b = (A.x * C.y - C.x * A.y) / (A.x * B.y - B.x * A.y);
    if (isnan(b)) {
      b = (A.y * C.z - C.y * A.z) / (A.y * B.z - B.y * A.z);
    }
    if (isnan(b)) {
      b = (A.x * C.z - C.x * A.z) / (A.x * B.z - B.x * A.z);
    }

    return (to.LL - to.LR) * a + (to.UR - to.LR) * b + to.LR;
  };

  auto start_rectangle = find_if(plane.begin(), plane.end(),
                                 [&](const auto &r) {
                                   return isPointInsideRectangle3D(start, r);
                                 }) -
                         plane.begin();
  auto start_copy =
      move_point(start, plane[start_rectangle], plane_copy[start_rectangle]);

  auto end_rectangle =
      find_if(plane.begin(), plane.end(),
              [&](const auto &r) { return isPointInsideRectangle3D(end, r); }) -
      plane.begin();
  auto end_copy =
      move_point(end, plane[end_rectangle], plane_copy[end_rectangle]);

  vector<Rectangle> rectangles(plane.size());
  for (int i = 0; i < plane.size(); ++i) {
    rectangles[i] = {Point(plane_copy[i].LL.x, plane_copy[i].LL.y),
                     Point(plane_copy[i].UR.x, plane_copy[i].UR.y)};
  }

  const auto s = Point(start_copy.x, start_copy.y);
  const auto e = Point(end_copy.x, end_copy.y);

  auto [path, distance] = solve(rectangles, s, e);

  Path3D path3D(path.size());
  for (int i = 0; i < path.size(); ++i) {
    // '&path = path' is an error of clang++
    auto tmp = find_if(rectangles.begin(), rectangles.end(),
                       [&path = path, i = i](const auto &r) {
                         return isPointInsideRectangle(path[i], r);
                       }) -
               rectangles.begin();
    path3D[i] = move_point(Point3D(path[i].x, path[i].y, 0), plane_copy[tmp],
                           plane[tmp]);
  }

  return {path3D, distance};
}
} // namespace Solve
