#include <map>
#include <queue>
#include <random>

#include <boost/geometry/algorithms/detail/equals/interface.hpp>

#include "find_path.h"

using Solve::cross;
using Solve::dot;

using bg::distance;

inline namespace FindPath {
auto isParallel(const Line3D &l1, const Line3D &l2) -> bool {
  auto dir1 = l1.second - l1.first;
  auto dir2 = l2.second - l2.first;
  auto c = cross(dir1, dir2);
  return bg::equals(c, Point3D());
}

auto arePointsCollinear(const Point3D &p1, const Point3D &p2, const Point3D &p3,
                        const Point3D &p4) -> bool {
  return bg::equals(cross(p2 - p1, p3 - p1), Point3D()) &&
         bg::equals(cross(p2 - p1, p4 - p1), Point3D());
}

auto calcOverlapLine(const Line3D &l1, const Line3D &l2) -> Line3D {
  if (!arePointsCollinear(l1.first, l1.second, l2.first, l2.second) ||
      bg::equals(l1.first, l1.second) || bg::equals(l2.first, l2.second)) {
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
    if (k > -Solve::EPS && k < 1 + Solve::EPS) {
      k = (p.x - l2.first.x) / (l2.second.x - l2.first.x);
      if (std::isnan(k)) {
        k = (p.y - l2.first.y) / (l2.second.y - l2.first.y);
      }
      if (std::isnan(k)) {
        k = (p.z - l2.first.z) / (l2.second.z - l2.first.z);
      }
      if (k > -Solve::EPS && k < 1 + Solve::EPS) {
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
    return distance(line.first, line.second) > Solve::EPS;
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

auto Dijkstra(const vector<PointWithRectangleIndex> &pris,
              const Rectangle3DList &list) -> vector<int> {
  constexpr auto INF = std::numeric_limits<double>::max();
  const int START = pris.size() - 2;
  const int END = pris.size() - 1;

  vector<bool> vis(pris.size(), false);
  vector<double> dis(vis.size(), INF);
  vector<int> prev(vis.size(), -1);
  vector<int> path;
  std::priority_queue<DijkstraNode> pq;

  dis[START] = 0;
  pq.push({START, 0});

  auto isRectangleOverlap = [](const Rectangle3D &r1, const Rectangle3D &r2) {
    auto line = calcOverlapRectangle(r1, r2);
    return distance(line.first, line.second) > Solve::EPS;
  };

  while (!pq.empty()) {
    const auto id = pq.top().id;
    pq.pop();
    if (vis[id]) {
      continue;
    }
    vis[id] = true;

    for (int i = 0; i < pris.size(); ++i) {
      if (pris[i].r == pris[id].r ||
          isRectangleOverlap(list[pris[i].r], list[pris[id].r])) {
        if (const auto tmp = distance(pris[i].p, pris[id].p);
            dis[i] > dis[id] + tmp) {
          dis[i] = dis[id] + tmp;
          prev[i] = id;
          pq.push({i, dis[i]});
        }
      }
    }
  }

  int cur = prev[END];
  while (cur != START && cur != -1) {
    path.push_back(cur);
    cur = prev[cur];
  }
  std::reverse(path.begin(), path.end());
  return path;
}

auto rand_0_to_1() -> double {
  std::mt19937_64 rng{std::random_device{}()};
  std::uniform_real_distribution<double> dist{0.0, 1.0};
  return dist(rng);
}

auto find_rectangle_indexs(const Rectangle3DList &list, const Point3D &start,
                           const Point3D &end) -> vector<int> {
  constexpr auto block_size = 1.0;

  vector<PointWithRectangleIndex> points;
  std::map<int, int> pointToRectangleIndex;

  for (int i = 0; i < list.size(); ++i) {
    const auto &r = list[i];
    for (int j = 0;
         j < distance(r.LL, r.LR) * distance(r.LR, r.UR) / block_size; ++j) {
      const auto x = rand_0_to_1();
      const auto y = rand_0_to_1();
      const auto point = (r.LL - r.LR) * x + (r.UR - r.LR) * y + r.LR;
      points.push_back({point, i});
      pointToRectangleIndex[points.size() - 1] = i;
    }
  }

  const int start_rectangle =
      find_if(list.begin(), list.end(),
              [&start](const auto &r) {
                return Solve::isPointInsideRectangle3D(start, r);
              }) -
      list.begin();

  const int end_rectangle =
      find_if(list.begin(), list.end(),
              [&end](const auto &r) {
                return Solve::isPointInsideRectangle3D(end, r);
              }) -
      list.begin();

  points.push_back({start, start_rectangle});
  points.push_back({end, end_rectangle});

  const auto path = Dijkstra(points, list);
  vector<int> indexs{start_rectangle};
  for (const auto i : path) {
    indexs.emplace_back(pointToRectangleIndex[i]);
  }
  indexs.emplace_back(end_rectangle);
  return indexs;
}

auto lerp_point3D(const Point3D &a, const Point3D &b, double t) -> Point3D {
  return (a - b) * t + b;
}

auto calc_path(const Rectangle3DList &list,
               const vector<vector<int>> &rectangles, const Point3D &start,
               const Point3D &end, const vector<Line3D> &lines,
               const vector<double> &input) -> std::pair<Path3D, double> {
  auto n = rectangles.size();
  if (n == 1) {
    return Solve::solve3D(list, start, end);
  }
  Path3D path{};
  double dis = 0.0;
  Rectangle3DList rects{};
  for (const auto r : rectangles.front()) {
    rects.push_back(list[r]);
  }
  auto [tmp_path, tmp_dis] = Solve::solve3D(
      rects, start,
      lerp_point3D(lines.front().first, lines.front().second, input.front()));
  for_each(tmp_path.begin(), tmp_path.end(),
           [&](auto &p) { path.emplace_back(p); });
  dis += tmp_dis;
  for (int i = 1; i < n - 1; ++i) {
    rects.clear();
    for (const auto r : rectangles[i]) {
      rects.push_back(list[r]);
    }
    std::tie(tmp_path, tmp_dis) = Solve::solve3D(
        rects,
        lerp_point3D(lines[i - 1].first, lines[i - 1].second, input[i - 1]),
        lerp_point3D(lines[i].first, lines[i].second, input[i]));
    for_each(tmp_path.begin() + 1, tmp_path.end(),
             [&](auto &p) { path.emplace_back(p); });
    dis += tmp_dis;
  }
  rects.clear();
  for (const auto r : rectangles.back()) {
    rects.push_back(list[r]);
    std::tie(tmp_path, tmp_dis) = Solve::solve3D(
        rects,
        lerp_point3D(lines.back().first, lines.back().second, input.back()),
        end);
    for_each(tmp_path.begin() + 1, tmp_path.end(),
             [&](auto &p) { path.emplace_back(p); });
    dis += tmp_dis;
  }
  return {path, dis};
}

auto simulated_annealing(const Rectangle3DList &list,
                         const vector<vector<int>> &rectangles,
                         const Point3D &start, const Point3D &end,
                         const vector<Line3D> &lines, int iterations,
                         double initial_temp, double cooling_rate)
    -> vector<double> {
  auto n = lines.size();
  vector<double> current(n, 0.5);

  auto current_energy =
      calc_path(list, rectangles, start, end, lines, current).second;
  auto best = current;
  auto best_energy = current_energy;

  std::mt19937_64 rng{std::random_device{}()};

  for (int i = 0; i < iterations; ++i) {
    auto new_solution = current;
    new_solution[rng() % n] = rand_0_to_1();

    auto new_energy =
        calc_path(list, rectangles, start, end, lines, new_solution).second;

    auto energy_diff = new_energy - current_energy;
    auto temp = initial_temp * pow(cooling_rate, i);

    if (energy_diff < 0 || exp(-energy_diff / temp) > rand_0_to_1()) {
      current = new_solution;
      current_energy = new_energy;
    }

    if (current_energy < best_energy) {
      best = current;
      best_energy = current_energy;
    }
  }
  return best;
}
} // namespace FindPath

auto find_path(const Rectangle3DList &list, const Point3D &start,
               const Point3D &end) -> std::pair<Path3D, double> {
  auto indexs = find_rectangle_indexs(list, start, end);
  vector<vector<int>> rectangles;
  vector<Line3D> lines;
  for (int i = 0; i < indexs.size(); ++i) {
    vector<int> rects{indexs[i]};
    Line3D line;
    for (++i; i < indexs.size(); ++i) {
      if (auto overlap_line =
              calcOverlapRectangle(list[rects.back()], list[indexs[i]]);
          isParallel(line, overlap_line)) {
        line = overlap_line;
        rects.emplace_back(indexs[i]);
      } else {
        lines.emplace_back(overlap_line);
        i--;
        break;
      }
    }
    rectangles.emplace_back(rects);
  }
  if (lines.empty()) {
    return Solve::solve3D(list, start, end);
  }
  auto tmp = simulated_annealing(list, rectangles, start, end, lines, 10000,
                                 1.0, 0.999);
  return calc_path(list, rectangles, start, end, lines, tmp);
}