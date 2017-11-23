#include "descartes_planner/ladder_graph_dag_search.h"

namespace descartes_planner
{

DAGSearch::DAGSearch(const LadderGraph &graph)
  : graph_(graph)
{
  // On creating an object, let's allocate everything we need
  solution_.resize(graph.size());

  for (size_t i = 0; i < graph.size(); ++i)
  {
    const auto n_vertices = graph.rungSize(i);
    solution_[i].distance.resize(n_vertices);
    solution_[i].predecessor.resize(n_vertices);

    if(0 == n_vertices)
    {
      ROS_ERROR_STREAM("[DAG init] Empty Rung!!!! rung #" << i << "size " << n_vertices);
    }
  }
}

double DAGSearch::run()
{
  // Cost to the first rung should be set to zero
  std::fill(solution_.front().distance.begin(), solution_.front().distance.end(), 0.0);

  // Other rows initialize to inf
  for (size_type i = 1; i < solution_.size(); ++i)
  {
    std::fill(solution_[i].distance.begin(), solution_[i].distance.end(), std::numeric_limits<double>::max());
  }

  // Now we iterate over the graph in 'topological' order (current rung assign next rung)
  for (size_type rung_id = 0; rung_id < solution_.size() - 1; ++rung_id)
  {
    const auto n_vertices = graph_.rungSize(rung_id);
    const auto next_rung_id = rung_id + 1;

    // For each vertex in current rung
    for (size_t vert_id = 0; vert_id < n_vertices; ++vert_id)
    {
      // cost up until the current vertex
      const auto u_cost = distance(rung_id, vert_id);

      // get all the out edges of the current vertex
      const auto &edges = graph_.getEdges(rung_id)[vert_id];

      // for each out edge
      for (const auto& edge : edges)
      {
        // end vertex of the out edge's new cost = current vertex's cost + out_edge's cost
        auto dv = u_cost + edge.cost;

        if (dv < distance(next_rung_id, edge.idx))
        {
          distance(next_rung_id, edge.idx) = dv;

          // edge.idx = vert's id in next rung
          // update the optimal predecessor vert's id
          predecessor(next_rung_id, edge.idx) = vert_id;
        }
      }
    } // vertex for loop
  } // rung for loop

  return *std::min_element(solution_.back().distance.begin(), solution_.back().distance.end());
}

std::vector<DAGSearch::predecessor_t> DAGSearch::shortestPath() const
{
  // get the id of least-cost vertex (min_idx) in the last rung
  auto min_it = std::min_element(solution_.back().distance.begin(), solution_.back().distance.end());
  auto min_idx = std::distance(solution_.back().distance.begin(), min_it);

  assert(min_idx >= 0);

  std::vector<predecessor_t> path(solution_.size());

  size_type current_rung = path.size() - 1;
  size_type current_index = min_idx;

  for (unsigned i = 0; i < path.size(); ++i)
  {
    auto count = path.size() - 1 - i;
    assert(current_rung == count);
    path[count] = current_index;
    current_index = predecessor(current_rung, current_index);
    current_rung -= 1;
  }

  return path;
}

} // namespace descartes_planner
