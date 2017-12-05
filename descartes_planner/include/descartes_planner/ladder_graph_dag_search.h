/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DESCARTES_LADDER_GRAPH_DAG_SEARCH_H
#define DESCARTES_LADDER_GRAPH_DAG_SEARCH_H

#include "descartes_planner/ladder_graph.h"

namespace descartes_planner
{

// Directed Acyclic graph search
class DAGSearch
{
public:
  using predecessor_t = unsigned;
  using size_type = std::size_t;

  explicit DAGSearch(const LadderGraph& graph);

  double run();

  std::vector<predecessor_t> shortestPath() const;

private:

  struct SolutionRung
  {
    // accumulated cost (from rung 0) to vertices in current rung
    std::vector<double> distance;

    // min cost out edge from last rung for vertices in current rung
    // record the optimal vertex's id in last rung
    std::vector<predecessor_t> predecessor;
  };

  inline double& distance(size_type rung_index, size_type vert_index) noexcept
  {
    return solution_[rung_index].distance[vert_index];
  }

  inline predecessor_t& predecessor(size_type rung_index, size_type vert_index) noexcept
  {
    return solution_[rung_index].predecessor[vert_index];
  }

  inline const predecessor_t& predecessor(size_type rung_index, size_type vert_index) const noexcept
  {
    return solution_[rung_index].predecessor[vert_index];
  }

  const LadderGraph& graph_;
  std::vector<SolutionRung> solution_;
};
} // descartes_planner
#endif
