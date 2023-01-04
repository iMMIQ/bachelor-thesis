#include <iostream>

#include "basic.h"

using std::cin;
using std::cout;
using std::endl;

auto main([[maybe_unused]] int argc, [[maybe_unused]] char **argv) -> int {
  int n;
  cin >> n;
  Plane plane(n);
  for (int i = 0; i < n; ++i) {
    cin >> plane[i].LL >> plane[i].UR >> plane[i].LR;
  }
  Point3D start, end;
  cin >> start >> end;
  auto ans = Basic::solve(plane, start, end);
  cout << "path:\n";
  for (const auto &i : ans.first) {
    cout << i << endl;
  }
  cout << "distance: " << ans.second << endl;
  return 0;
}