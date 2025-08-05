#include "../include/post_processing.hpp"


#ifdef _WIN32
#include <string>
#include <array>
#include <memory>
#include <iostream>

std::string get_cpu_name() {
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&_pclose)> pipe(_popen("wmic cpu get Name", "r"), _pclose);
  if (!pipe) return "";
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  // 处理结果，仅保留12100F或10400F
  if (result.find("12100F") != std::string::npos) return "12100F";
  if (result.find("10400F") != std::string::npos) return "10400F";
  return "Unknown";
}
#endif


// 在多智能体路径规划（Multi-Agent Path Finding, MAPF）问题中，检查给定的路径解（solution）是否是一个可行解。
bool is_feasible_solution(const Instance &ins, const Solution &solution,
                          const int verbose)
{
  if (solution.empty()) return true;

  // check start locations
  if (!is_same_config(solution.front(), ins.starts)) {
    info(1, verbose, "invalid starts");
    return false;
  }

  // check goal locations
  if (!is_same_config(solution.back(), ins.goals)) {
    info(1, verbose, "invalid goals");
    return false;
  }

  for (size_t t = 1; t < solution.size(); ++t) {
    for (size_t i = 0; i < ins.N; ++i) {
      auto v_i_from = solution[t - 1][i];
      auto v_i_to = solution[t][i];
      // check connectivity
      if (v_i_from != v_i_to &&
          std::find(v_i_to->neighbors.begin(), v_i_to->neighbors.end(),
                    v_i_from) == v_i_to->neighbors.end()) {
        info(1, verbose, "invalid move");
        return false;
      }

      // check conflicts
      for (size_t j = i + 1; j < ins.N; ++j) {
        auto v_j_from = solution[t - 1][j];
        auto v_j_to = solution[t][j];
        // vertex conflicts
        if (v_j_to == v_i_to) {
          info(1, verbose, "vertex conflict between agent-", i, " and agent-",
               j, " at vertex-", v_i_to->id, " at timestep ", t);
          return false;
        }
        // swap conflicts
        if (v_j_to == v_i_from && v_j_from == v_i_to) {
          info(1, verbose, "edge conflict");
          return false;
        }
      }
    }
  }

  return true;
}


// 是统计并打印多智能体路径问题（MAPF）求解后的各项关键性能指标
void print_stats(const int verbose, const Deadline *deadline,
                 const Instance &ins, const Solution &solution,
                 const double comp_time_ms)
{
  auto ceil = [](float x) { return std::ceil(x * 100) / 100; };
  auto dist_table = DistTable(ins);
  const auto makespan = get_makespan(solution);
  const auto makespan_lb = get_makespan_lower_bound(ins, dist_table);
  const auto sum_of_costs = get_sum_of_costs(solution);
  const auto sum_of_costs_lb = get_sum_of_costs_lower_bound(ins, dist_table);
  const auto sum_of_loss = get_sum_of_loss(solution);
  info(1, verbose, deadline, "solved", "\tcomp_time_ms: ", comp_time_ms,
       "\tmakespan: ", makespan, " (lb=", makespan_lb,
       ", ub=", ceil((float)makespan / makespan_lb), ")",
       "\tsum_of_costs: ", sum_of_costs, " (lb=", sum_of_costs_lb,
       ", ub=", ceil((float)sum_of_costs / sum_of_costs_lb), ")",
       "\tsum_of_loss: ", sum_of_loss, " (lb=", sum_of_costs_lb,
       ", ub=", ceil((float)sum_of_loss / sum_of_costs_lb), ")");
}


// for log of map_name
static const std::regex r_map_name = std::regex(R"(.+/(.+))");


// 将多智能体路径规划（MAPF）实验结果写入日志文件
void make_log(const Instance &ins, const Solution &solution,
              const std::string &output_name, const double comp_time_ms,
              const std::string &map_name, const std::string &scen_name, const int seed, const bool log_short)
{
  // map name
  std::smatch results;
  const auto map_recorded_name =
      (std::regex_match(map_name, results, r_map_name)) ? results[1].str()
                                                        : map_name;

  // for instance-specific values
  auto dist_table = DistTable(ins);

  // log for visualizer
  auto get_x = [&](int k) { return k % ins.G.width; };
  auto get_y = [&](int k) { return k / ins.G.width; };
  std::ofstream log;
  log.open(output_name, std::ios::out);
  log << "agents=" << ins.N << "\n";
  log << "map_file=" << map_recorded_name << "\n";
  log << "solver=planner\n";
  log << "solved=" << !solution.empty() << "\n";
  log << "soc=" << get_sum_of_costs(solution) << "\n";
  log << "soc_lb=" << get_sum_of_costs_lower_bound(ins, dist_table) << "\n";
  log << "makespan=" << get_makespan(solution) << "\n";
  log << "makespan_lb=" << get_makespan_lower_bound(ins, dist_table) << "\n";
  log << "sum_of_loss=" << get_sum_of_loss(solution) << "\n";
  log << "sum_of_loss_lb=" << get_sum_of_costs_lower_bound(ins, dist_table)
      << "\n";
  log << "comp_time=" << comp_time_ms << "\n";
  log << "seed=" << seed << "\n";
  if (log_short) return;
  log << "starts=";
  for (size_t i = 0; i < ins.N; ++i) {
    auto k = ins.starts[i]->index;
    log << "(" << get_x(k) << "," << get_y(k) << "),";
  }
  log << "\ngoals=";
  for (size_t i = 0; i < ins.N; ++i) {
    auto k = ins.goals[i]->index;
    log << "(" << get_x(k) << "," << get_y(k) << "),";
  }
  log << "\nsolution=\n";
  for (size_t t = 0; t < solution.size(); ++t) {
    log << t << ":";
    auto C = solution[t];
    for (auto v : C) {
      log << "(" << get_x(v->index) << "," << get_y(v->index) << "),";
    }
    log << "\n";
  }
  log.close();

  // save result to csv
  std::string to_csv_path = "experimental_results.csv";
  std::ofstream to_csv(to_csv_path, std::ios::app);  // 以追加模式打开文件
  if (!to_csv.is_open()) {
    std::cerr << "Error opening csv!" << std::endl;
    return;
  }

  to_csv << -1 << ","; // id

  to_csv << map_recorded_name << ","; // map_name

  to_csv << scen_name << ","; // agent file name
  to_csv << ins.N << ","; // num of agents

#ifdef _WIN32
  to_csv << get_cpu_name() << ","; // device
#elif __linux__
  to_csv << "12400F" << ","; // device
#else
  to_csv << "Unknown" << ","; // device
#endif

  to_csv << "Lacam" << ","; // high level solver name
  to_csv << "PIBT" << ","; // low level solver name
  to_csv << -1 << ","; // disappear or not
  to_csv << -1 << ","; // 是否使用CAT break tie
  to_csv << -1 << ","; // random seed

  size_t total_cost_disappear_at_goal = get_sum_of_loss(solution);
  to_csv << total_cost_disappear_at_goal << ",";
  auto plan_time = comp_time_ms;
  to_csv << plan_time << ",";
  to_csv << "NULL" << ","; // comment
  to_csv << "https://github.com/ssfc/lacam0" << ","; // method source

  // 获取当前时间点
  auto now = std::chrono::system_clock::now();
  // 转换为 time_t 格式
  std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
  // 输出时间
  to_csv << std::put_time(std::localtime(&currentTime), "%Y-%m-%d %H:%M:%S")
         << "\n";
}
