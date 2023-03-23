#include <map>
#include <queue>
#include <random>

#include "find_path.h"

using Solve::cross;
using Solve::dot;

using bg::distance;

inline namespace FindPath {
auto Dijkstra(const vector<PointWithRectangleIndex> &pris,
              const Rectangle3DList &list, const vector<vector<bool>> &edge)
    -> vector<int> {
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

  while (!pq.empty()) {
    const auto id = pq.top().id;
    pq.pop();
    if (vis[id]) {
      continue;
    }
    vis[id] = true;

    for (int i = 0; i < pris.size(); ++i) {
      if (edge[pris[i].r][pris[id].r]) {
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

auto find_rectangle_index(const Rectangle3DList &list, const Point3D &start,
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

  auto isRectangleOverlap = [](const Rectangle3D &r1, const Rectangle3D &r2) {
    auto line = Solve::calcOverlapRectangle(r1, r2);
    return distance(line.first, line.second) > EPS;
  };

  vector<vector<bool>> edge(list.size(), vector<bool>(list.size(), false));
  for (int i = 0; i < list.size(); ++i) {
    for (int j = i + 1; j < list.size(); ++j) {
      if (isRectangleOverlap(list[i], list[j])) {
        edge[i][j] = edge[j][i] = true;
      }
    }
  }
  for (int i = 0; i < list.size(); ++i) {
    edge[i][i] = true;
  }
  const auto path = Dijkstra(points, list, edge);

  vector<int> index{start_rectangle};
  for (const auto i : path) {
    index.emplace_back(pointToRectangleIndex[i]);
  }
  index.emplace_back(end_rectangle);
  return index;
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

auto find_path(Rectangle3DList &list, const Point3D &start, const Point3D &end)
    -> std::pair<Path3D, double> {
  auto indexs = find_rectangle_index(list, start, end);
  vector<vector<int>> rectangles;
  vector<Line3D> lines;
  for (int i = 0; i < indexs.size(); ++i) {
    vector<int> rects{indexs[i]};
    Line3D line;
    for (++i; i < indexs.size(); ++i) {
      auto r = list[rects.back()];
      if (auto overlap_line = Solve::calcOverlapRectangle(r, list[indexs[i]]);
          Solve::isParallel(line, overlap_line)) {
        line = overlap_line;
        rects.push_back(indexs[i]);
      } else {
        lines.emplace_back(overlap_line);
        i--;
        break;
      }
    }
    rectangles.emplace_back(rects);
  }
  if (lines.empty()) {
    Rectangle3DList tmp;
    for (const auto i : indexs) {
      tmp.push_back(list[i]);
    }
    return Solve::solve3D(tmp, start, end);
  }
  auto tmp = simulated_annealing(list, rectangles, start, end, lines, 10000,
                                 1.0, 0.999);
  return calc_path(list, rectangles, start, end, lines, tmp);
}