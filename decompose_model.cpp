#include "data.h"

// unused functions
auto decompose_model(const vector<Cube> &cubes) -> vector<Plane> {
  decltype(decompose_model(cubes)) res(1);
  for (const auto &cube : cubes) {
    res[0].emplace_back(cube.bottom);
  }
  return res;
}