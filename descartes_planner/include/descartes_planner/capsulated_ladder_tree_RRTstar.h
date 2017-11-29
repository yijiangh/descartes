//
// Created by yijiangh on 11/28/17.
//

#ifndef DESCARTES_CAPSULATED_LADDER_TREE_RRTSTAR_H
#define DESCARTES_CAPSULATED_LADDER_TREE_RRTSTAR_H

#include <moveit/planning_scene/planning_scene.h>
#include <descartes_trajectory/joint_trajectory_pt.h>

// for ConstrainedSegment
#include "graph_builder.h"

#include "capsulated_ladder_tree.h"

namespace descartes_planner
{
// RRT* on capsulated ladder tree
class CapsulatedLadderTreeRRTstar
{
 public:
  explicit CapsulatedLadderTreeRRTstar(
      const std::vector<ConstrainedSegment>& segs,
      const std::vector<planning_scene::PlanningScenePtr>& planning_scenes);

  // use RRT* on a ladder tree to get optimal capsulated solution
  // return the cost of the solution, if no sol found, return numerical max
  double solve(descartes_core::RobotModel& model);

  // construct ladder graph for each capsule and apply DAG search to get full trajectory solution
  void extractSolution(std::vector<descartes_core::TrajectoryPtPtr>& sol);

 private:
  std::vector<CapRung> cap_rungs_;
};
}

#endif //DESCARTES_CAPSULATED_LADDER_TREE_RRTSTAR_H
