#include "../include/lacam.hpp"

bool LaCAM::ANYTIME = false;
float LaCAM::RANDOM_INSERT_PROB1 = 0.001;
float LaCAM::RANDOM_INSERT_PROB2 = 0.001;

// 函数对象（仿函数）的比较运算符，专门用来比较两个HNode*（指向HNode结构的指针）的“大小”。
bool CompareHNodePointers::operator()(const HNode *l, const HNode *r) const
{
  const auto N = l->Q.size();
  for (size_t i = 0; i < N; ++i) {
    if (l->Q[i] != r->Q[i]) return l->Q[i]->id < r->Q[i]->id;
  }
  return false;
}

//  HNode 类的构造函数，主要作用是基于给定的参数（配置、距离表、父节点、代价等）初始化一个新的搜索树节点。
HNode::HNode(Config _Q, DistTable *D, HNode *_parent, int _g, int _h)
    : Q(_Q),
      parent(_parent),
      neighbors(),
      g(_g),
      h(_h),
      f(g + h),
      depth(parent == nullptr ? 0 : parent->depth + 1),
      priorities(Q.size()),
      order(Q.size(), 0),
      search_tree()
{
  if (parent != nullptr) parent->neighbors.insert(this);
  search_tree.push(new LNode());
  const int N = Q.size();
  for (int i = 0; i < N; ++i) {
    // set priorities
    if (parent == nullptr) {
      // initialize
      priorities[i] = (float)D->get(i, Q[i]) / 10000;
    } else {
      // dynamic priorities, akin to PIBT
      if (D->get(i, Q[i]) != 0) {
        priorities[i] = parent->priorities[i] + 1;
      } else {
        priorities[i] = parent->priorities[i] - (int)parent->priorities[i];
      }
    }
  }

  // set order
  auto cmp = [&](int i, int j) { return priorities[i] > priorities[j]; };
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), cmp);
}

// 确保内存安全地释放 search_tree 队列中的所有动态分配对象，防止内存泄漏。
HNode::~HNode()
{
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}

//  LNode 类的默认构造函数，其作用是初始化 LNode 类的成员变量。
LNode::LNode() : who(), where(), depth(0) {}

// 根据给定的父节点、一个整数和一个顶点指针，构造新的 LNode 对象。
LNode::LNode(LNode *parent, int i, Vertex *v)
    : who(parent->who), where(parent->where), depth(parent->depth + 1)
{
  who.push_back(i);
  where.push_back(v);
}

LNode::~LNode(){};

// 初始化 LaCAM 类的成员变量，为后续算法运行做准备。
LaCAM::LaCAM(const Instance *_ins, DistTable *_D, int _verbose,
             const Deadline *_deadline, int _seed)
    : ins(_ins),
      D(_D),
      deadline(_deadline),
      seed(_seed),
      MT(seed),
      rrd(0, 1),
      verbose(_verbose),
      pibt(ins, D, seed),
      H_goal(nullptr),
      OPEN(),
      loop_cnt(0)
{
}

LaCAM::~LaCAM() {}

// 在给定的时间限制内，为所有智能体从起点到终点找到一组可行（或最优）的路径方案。
Solution LaCAM::solve()
{
  // 输出算法启动信息。
  solver_info(1, "LaCAM begins");

  // setup search
  // 用于记录已经探索过的配置（哈希表，避免重复扩展）。
  auto EXPLORED = std::unordered_map<Config, HNode *, ConfigHasher>();
  HNodes GC_HNodes; // 用于后续内存回收，保存所有高层节点指针。

  // insert initial node
  auto H_init = new HNode(ins->starts, D); // 新建一个以起点为内容的高层节点H_init。
  OPEN.push_front(H_init); // 将其插入OPEN表（待扩展节点队列）。
  EXPLORED[H_init->Q] = H_init; // 标记为已探索，且加入垃圾回收管理队列。
  GC_HNodes.push_back(H_init);

  // search loop
  // 主搜索循环
  solver_info(2, "search iteration begins");
  // 只要OPEN表不空，且没有超时，循环继续。
  while (!OPEN.empty() && !is_expired(deadline))
  {
    ++loop_cnt;

    // random insert
    // 增加搜索多样性（random restart/插入），利用概率在 OPEN 表头插入初始节点或其他随机节点。
    if (H_goal != nullptr)
    {
      auto r = rrd(MT);
      if (r < RANDOM_INSERT_PROB2 / 2)
      {
        OPEN.push_front(H_init);
      }
      else if (r < RANDOM_INSERT_PROB2)
      {
        auto H = OPEN[get_random_int(MT, 0, OPEN.size() - 1)];
        OPEN.push_front(H);
      }
    }

    // do not pop here!
    // 取 OPEN 表头节点（注意：这里只取不弹，后面有条件才弹）。
    auto H = OPEN.front();  // high-level node

    // check uppwer bounds
    // 如果当前已找到目标（H_goal非空），且当前节点 g 值不优于已知目标，则剪枝并把初始节点重新加入 OPEN，跳过当前分支。
    if (H_goal != nullptr && H->g >= H_goal->g)
    {
      OPEN.pop_front();
      solver_info(5, "prune, g=", H->g, " >= ", H_goal->g);
      OPEN.push_front(H_init);
      continue;
    }

    // check goal condition
    // 如果是第一次到达所有agent目标，则设置H_goal。在非anytime模式下，找到后直接退出。
    if (H_goal == nullptr && is_same_config(H->Q, ins->goals)) {
      H_goal = H;
      solver_info(2, "found solution, g=", H->g, ", depth=", H->depth);
      if (!ANYTIME) break;
      continue;
    }

    // extract constraints
    // 若当前高层节点 search tree 为空，无子节点可扩展，则弹出 OPEN 并跳过。
    if (H->search_tree.empty()) {
      OPEN.pop_front();
      continue;
    }
    auto L = H->search_tree.front();
    H->search_tree.pop();

    // low level search
    // 扩展当前节点的低层树，随机化动作，逐步推进各智能体的动作组合（类似多队列 BFS 或多智能体交替扩展）。
    if (L->depth < H->Q.size()) {
      const auto i = H->order[L->depth];
      auto &&C = H->Q[i]->actions;
      std::shuffle(C.begin(), C.end(), MT);  // randomize
      for (auto u : C) H->search_tree.push(new LNode(L, i, u));
    }

    // create successors at the high-level search
    auto Q_to = Config(ins->N, nullptr);
    auto res = set_new_config(H, L, Q_to);
    delete L;
    if (!res) continue;

    // check explored list
    auto iter = EXPLORED.find(Q_to);
    if (iter == EXPLORED.end()) {
      // new one -> insert
      auto H_new = new HNode(Q_to, D, H, get_g_val(H, Q_to), get_h_val(Q_to));
      OPEN.push_front(H_new);
      EXPLORED[H_new->Q] = H_new;
      GC_HNodes.push_back(H_new);
    } else {
      // known configuration
      auto H_known = iter->second;
      rewrite(H, H_known);

      if (rrd(MT) >= RANDOM_INSERT_PROB1) {
        OPEN.push_front(iter->second);  // usual
      } else {
        solver_info(3, "random restart");
        OPEN.push_front(H_init);  // sometimes
      }
    }
  }

  // backtrack
  Solution solution;
  {
    auto H = H_goal;
    while (H != nullptr) {
      solution.push_back(H->Q);
      H = H->parent;
    }
    std::reverse(solution.begin(), solution.end());
  }

  // solution
  if (solution.empty()) {
    if (OPEN.empty()) {
      solver_info(2, "fin. unsolvable instance");
    } else {
      solver_info(2, "fin. reach time limit");
    }
  } else {
    if (OPEN.empty()) {
      solver_info(2, "fin. optimal solution, g=", H_goal->g,
                  ", depth=", H_goal->depth);
    } else {
      solver_info(2, "fin. suboptimal solution, g=", H_goal->g,
                  ", depth=", H_goal->depth);
    }
  }

  // end processing
  for (auto &&H : GC_HNodes) delete H;  // memory management

  return solution;
}

// 据当前高层节点和低层节点，生成一个新的多智能体联合状态配置 Q_to，并通过底层的策略（如 PIBT 算法）进一步调整配置的可行性和细节。
bool LaCAM::set_new_config(HNode *H, LNode *L, Config &Q_to)
{
  for (uint d = 0; d < L->depth; ++d) Q_to[L->who[d]] = L->where[d];
  return pibt.set_new_config(H->Q, Q_to, H->order);
}

// 在“任意时刻（ANYTIME）”搜索模式下，动态修正高层节点间的可达关系，并重新优化相关路径开销。
void LaCAM::rewrite(HNode *H_from, HNode *H_to)
{
  if (!ANYTIME) return;

  // update neighbors
  H_from->neighbors.insert(H_to);

  // Dijkstra
  std::queue<HNode *> Q({H_from});  // queue is sufficient
  while (!Q.empty()) {
    auto n_from = Q.front();
    Q.pop();
    for (auto n_to : n_from->neighbors) {
      auto g_val = n_from->g + get_edge_cost(n_from->Q, n_to->Q);
      if (g_val < n_to->g) {
        if (n_to == H_goal) {
          solver_info(2, "cost update: g=", H_goal->g, " -> ", g_val,
                      ", depth=", H_goal->depth, " -> ", n_from->depth + 1);
        }
        n_to->g = g_val;
        n_to->f = n_to->g + n_to->h;
        n_to->parent = n_from;
        n_to->depth = n_from->depth + 1;
        Q.push(n_to);
        if (H_goal != nullptr && n_to->f < H_goal->f) {
          OPEN.push_front(n_to);
          solver_info(4, "reinsert: g=", n_to->g, " < ", H_goal->g);
        }
      }
    }
  }
}

// 当前路径总代价 = 父节点路径总代价 + 本次跳转花费
int LaCAM::get_g_val(HNode *H_parent, const Config &Q_to)
{
  return H_parent->g + get_edge_cost(H_parent->Q, Q_to);
}

// 给定当前所有智能体的状态配置 Q，计算一个乐观的（但可能小于实际值的）从当前配置到目标配置的总代价估计，用作A*等启发式搜索算法的h值。
int LaCAM::get_h_val(const Config &Q)
{
  auto c = 0;
  for (size_t i = 0; i < ins->N; ++i) c += D->get(i, Q[i]);
  return c;
}

// 在多智能体路径规划中，两个状态（配置）之间转移的代价计算函数
int LaCAM::get_edge_cost(const Config &Q1, const Config &Q2)
{
  auto cost = 0;
  for (size_t i = 0; i < ins->N; ++i) {
    if (Q1[i] != ins->goals[i] || Q2[i] != ins->goals[i]) {
      cost += 1;
    }
  }
  return cost;
}
