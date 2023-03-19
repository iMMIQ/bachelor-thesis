#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include "input.h"
#include "solve.h"

inline namespace Input {
auto read_model(const std::string &file) -> vector<Triangle> {
  Assimp::Importer importer;

  const auto *scene = importer.ReadFile(file, aiProcess_Triangulate);

  vector<Triangle> result;

  for (int i = 0; i < scene->mNumMeshes; ++i) {
    const auto *mesh = scene->mMeshes[i];

    for (int j = 0; j < mesh->mNumFaces; ++j) {
      const auto &face = mesh->mFaces[j];

      const auto t = Triangle(Point3D(mesh->mVertices[face.mIndices[0]].x,
                                      mesh->mVertices[face.mIndices[0]].y,
                                      mesh->mVertices[face.mIndices[0]].z),
                              Point3D(mesh->mVertices[face.mIndices[1]].x,
                                      mesh->mVertices[face.mIndices[1]].y,
                                      mesh->mVertices[face.mIndices[1]].z),
                              Point3D(mesh->mVertices[face.mIndices[2]].x,
                                      mesh->mVertices[face.mIndices[2]].y,
                                      mesh->mVertices[face.mIndices[2]].z));

      result.push_back(t);
    }
  }

  return result;
}

auto isPerpendicular(const Line3D &a, const Line3D &b) -> bool {
  return std::abs(Solve::dot(a.first - a.second, b.first - b.second)) < EPS;
}

auto isRectangle(const Triangle &a, const Triangle &b) -> bool {
  const auto va = {a.A, a.B, a.C};
  const auto vb = {b.A, b.B, b.C};
  Point3D same_point1, same_point2;
  int same_count = 0;
  for (const auto &p : va) {
    for (const auto &q : vb) {
      if (p == q) {
        switch (same_count) {
        case 0:
          same_point1 = p;
          break;
        case 1:
          same_point2 = p;
          break;
        default:
          return false;
        }
        same_count++;
        break;
      }
    }
  }
  if (same_count != 2) {
    return false;
  }
  Point3D a_point, b_point;
  for (const auto &p : va) {
    if (p != same_point1 && p != same_point2) {
      a_point = p;
      break;
    }
  }
  for (const auto &p : vb) {
    if (p != same_point1 && p != same_point2) {
      b_point = p;
      break;
    }
  }
  if (!isPerpendicular(Line3D(same_point1, a_point),
                       Line3D(same_point2, a_point)) ||
      !isPerpendicular(Line3D(same_point1, b_point),
                       Line3D(same_point2, b_point))) {
    return false;
  }
  return Solve::isCoplanar(same_point1, same_point2, a_point, b_point);
}

auto transform_tri2rect3D(const vector<Triangle> &triangles)
    -> Rectangle3DList {
  Rectangle3DList res;
  auto n = triangles.size();
  vector<bool> vis(n, false);
  for (int i = 0; i < n; ++i) {
    if (vis[i]) {
      continue;
    }
    for (int j = i + 1; j < n; ++j) {
      if (vis[j]) {
        continue;
      }
      if (isRectangle(triangles[i], triangles[j])) {
        if (isPerpendicular(Line3D(triangles[i].B, triangles[i].A),
                            Line3D(triangles[i].C, triangles[i].A))) {
          res.emplace_back(triangles[i].B, triangles[i].C, triangles[i].A);
        } else if (isPerpendicular(Line3D(triangles[i].A, triangles[i].B),
                                   Line3D(triangles[i].C, triangles[i].B))) {
          res.emplace_back(triangles[i].A, triangles[i].C, triangles[i].B);
        } else {
          res.emplace_back(triangles[i].A, triangles[i].B, triangles[i].C);
        }
        vis[i] = vis[j] = true;
        break;
      }
    }
  }

  auto mid_point = [](const Point3D &a, const Point3D &b) {
    return Point3D((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2);
  };

  auto calc_area = [](const Triangle &t) {
    auto a = distance(t.A, t.B);
    auto b = distance(t.B, t.C);
    auto c = distance(t.C, t.A);
    auto s = (a + b + c) / 2;
    return sqrt(s * (s - a) * (s - b) * (s - c));
  };

  constexpr auto MIN_AREA = 0.1;
  for (int i = 0; i < n; ++i) {
    if (!vis[i]) {
      std::queue<Triangle> q;
      q.push(triangles[i]);
      while (!q.empty()) {
        auto t = q.front();
        q.pop();
        if (calc_area(t) > MIN_AREA) {
          auto LR = mid_point(t.A, t.B);
          auto LL = mid_point(t.A, t.C);
          auto UR = Solve::point_projection(LR, Line3D(t.B, t.C));
          auto UL = Solve::point_projection(LL, Line3D(t.B, t.C));
          res.emplace_back(LL, UR, LR);
          q.emplace(t.A, LR, LL);
          q.emplace(t.B, UR, LR);
          q.emplace(t.C, UL, LL);
        }
      }
    }
  }
  return res;
}
} // namespace Input
