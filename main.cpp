#include <iostream>

#include "basic.h"

using std::cin;
using std::cout;
using std::endl;
using std::swap;

auto main(int argc, char **argv) -> int {
  int n;
  cin >> n;
  Plane rectangles(n);
  for (int i = 0; i < n; ++i) {
    cin >> rectangles[i].UL.x >> rectangles[i].UL.y >> rectangles[i].LR.x >>
        rectangles[i].LR.y;
  }
  Point start, end;
  cin >> start.x >> start.y >> end.x >> end.y;
  if (start.x > end.x) {
    swap(start, end);
  }
  auto ans = Basic::solve(rectangles, start, end);
  for (const auto &i : ans.first) {
    cout << i.x << " " << i.y << endl;
  }
  cout << ans.second << endl;
  return 0;
}