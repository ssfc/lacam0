#include "../include/dist_table.hpp"

bool DistTable::MULTI_THREAD_INIT = true;

DistTable::DistTable(const Instance &ins)
    : K(ins.G.V.size()), table(ins.N, std::vector<int>(K, K))
{
  setup(&ins);
}

DistTable::DistTable(const Instance *ins)
    : K(ins->G.V.size()), table(ins->N, std::vector<int>(K, K))
{
  setup(ins);
}

// 为每个目标点做一次图的多源广度优先搜索（BFS），以预先计算每个节点到各目标点的最短距离。方法支持多线程并发初始化和单线程惰性初始化两种方式.
void DistTable::setup(const Instance *ins)
{
  if (MULTI_THREAD_INIT) {
    auto bfs = [&](const int i) {
      auto g_i = ins->goals[i];
      auto Q = std::queue<Vertex *>({g_i});
      table[i][g_i->id] = 0;
      while (!Q.empty()) {
        auto n = Q.front();
        Q.pop();
        const int d_n = table[i][n->id];
        for (auto &m : n->neighbors) {
          const int d_m = table[i][m->id];
          if (d_n + 1 >= d_m) continue;
          table[i][m->id] = d_n + 1;
          Q.push(m);
        }
      }
    };

    auto pool = std::vector<std::future<void>>();
    for (size_t i = 0; i < ins->N; ++i) {
      pool.emplace_back(std::async(std::launch::async, bfs, i));
    }
  } else {
    // lazy BFS
    for (size_t i = 0; i < ins->N; ++i) {
      OPEN.push_back(std::queue<Vertex *>());
      auto n = ins->goals[i];
      OPEN[i].push(n);
      table[i][n->id] = 0;
    }
  }
}

int DistTable::get(const int i, const int v_id)
{
  if (table[i][v_id] < K) return table[i][v_id];

  /*
   * BFS with lazy evaluation
   * c.f., Reverse Resumable A*
   * https://www.aaai.org/Papers/AIIDE/2005/AIIDE05-020.pdf
   *
   * sidenote:
   * tested RRA* but lazy BFS was much better in performance
   */

  while (!OPEN[i].empty()) {
    auto &&n = OPEN[i].front();
    OPEN[i].pop();
    const int d_n = table[i][n->id];
    for (auto &&m : n->neighbors) {
      const int d_m = table[i][m->id];
      if (d_n + 1 >= d_m) continue;
      table[i][m->id] = d_n + 1;
      OPEN[i].push(m);
    }
    if (n->id == v_id) return d_n;
  }
  return K;
}

int DistTable::get(const int i, const Vertex *v) { return get(i, v->id); }
