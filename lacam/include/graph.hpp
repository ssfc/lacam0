/*
 * graph definition
 */
#pragma once
#include "utils.hpp"


// 封装了一个点的位置、唯一编号、邻接关系以及所有相关动作，是图结构建模和搜索算法的基础数据结构。
struct Vertex
{
  const int id;     // index for V in Graph
  const int index;  // index for U (width * y + x) in Graph
  const int x;
  const int y;
  std::vector<Vertex *> neighbors;
  std::vector<Vertex *> actions;  // neighbor + self

  Vertex(int _id, int _index, int _x, int _y);
};

using Vertices = std::vector<Vertex *>;
using Config = std::vector<Vertex *>;  // locations for all agents
using Path = std::vector<Vertex *>;    // path
using Paths = std::vector<Path>;

struct Graph {
  Vertices V;  // without nullptr
  Vertices U;  // with nullptr, i.e., |U| = width * height
  int width;   // grid width
  int height;  // grid height
  Graph(int w = 0, int h = 0);
  Graph(const std::string &filename);  // taking map filename
  ~Graph();

  int size() const;  // the number of vertices, |V|
  void save(const std::string &output_name) const;
};

bool is_connected(const Graph *G);
bool is_connected(const Graph &G);

inline int manhattanDist(Vertex *a, Vertex *b)
{
  return std::abs(a->x - b->x) + std::abs(a->y - b->y);
}

bool is_same_config(
    const Config &C1,
    const Config &C2);  // check equivalence of two configurations

// hash function of configuration
// c.f.
// https://stackoverflow.com/questions/10405030/c-unordered-map-fail-when-used-with-a-vector-as-key
struct ConfigHasher {
  uint operator()(const Config &C) const;
};

std::ostream &operator<<(std::ostream &os, const Vertex *v);
std::ostream &operator<<(std::ostream &os, const Config &Q);
std::ostream &operator<<(std::ostream &os, const Paths &paths);
std::ostream &operator<<(std::ostream &os, const Graph &G);
std::ostream &operator<<(std::ostream &os, const Graph *G);
