#include "../include/planner.hpp"

// 利用给定的实例（Instance），使用距离表（DistTable）和 LaCAM 算法，计算并返回一个解（Solution）。
Solution solve(const Instance &ins, int verbose, const Deadline *deadline,
               int seed)
{
  // distance table
  auto D = DistTable(ins);
  info(1, verbose, deadline,
       "set distance table, multi-thread init: ", DistTable::MULTI_THREAD_INIT);

  // lacam
  auto lacam = LaCAM(&ins, &D, verbose, deadline, seed);
  info(1, verbose, deadline, "start lacam");
  return lacam.solve();
}
