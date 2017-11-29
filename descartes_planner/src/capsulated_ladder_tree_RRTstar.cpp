//
// Created by yijiangh on 11/28/17.
//

#include "descartes_planner/capsulated_ladder_tree.h"
#include "descartes_planner/capsulated_ladder_tree_RRTstar.h"

// ladder graph & DAG (solution extraction)
#include "descartes_planner/ladder_graph.h"
#include "descartes_planner/ladder_graph_dag_search.h"

namespace // anon namespace to hide utility functions
{
  Eigen::Affine3d generateSample(size_t rung_id)
  {

  }

  bool checkFeasibility(const Eigen::Affine3d& pose, const descartes_planner::CapRung& cap_rung)
  {

  }
} //end util namespace

namespace descartes_planner
{
CapsulatedLadderTreeRRTstar::CapsulatedLadderTreeRRTstar(
    descartes_core::RobotModel& model,
    const std::vector<ConstrainedSegment>& segs,
    const std::vector<planning_scene::PlanningScenePtr>& planning_scenes)
{
  // sanity check
  assert(segs.size() == planning_scenes.size());

  // intialize cap rungs
  cap_rungs_.reserve(segs.size());
  for(size_t i=0; i < segs.size(); i++)
  {
    CapRung cap_rung;

    cap_rung.orientations_.reserve(segs[i].orientations.size());
    for (auto& orient : segs[i].orientations)
    {
      cap_rung.orientations_.push_back(orient);
    }

    cap_rung.planning_scene_ = planning_scenes[i];

    cap_rungs_.push_back(cap_rung);
  }
}

double CapsulatedLadderTreeRRTstar::solve()
{
  // find initial solution

  // RRT* improve on the tree

}

void CapsulatedLadderTreeRRTstar::extractSolution(std::vector<descartes_core::TrajectoryPtPtr>& sol)
{
  // construct unit ladder graph for each cap rung

  // carry out DAG search on each ladder graph
}

} //end namespace descartes planner
