//
// Created by yijiangh on 11/28/17.
//

#include "descartes_planner/capsulated_ladder_tree.h"
#include "descartes_planner/capsulated_ladder_tree_RRTstar.h"

// ladder graph & DAG (solution extraction)
#include "descartes_planner/ladder_graph.h"
#include "descartes_planner/ladder_graph_dag_search.h"

// unit process sampling timeout in initial solution searching
const static double UNIT_PROCESS_TIMEOUT = 10.0;
// total timeout for RRTstar
const static double RRTSTAR_TIMEOUT = 120.0;

namespace // anon namespace to hide utility functions
{
  Eigen::Affine3d generateSample(size_t rung_id)
  {

  }

  // if pose is feasible, return true and start and end joint solution
  // otherwise return empty joint solution
  bool checkFeasibility(const Eigen::Affine3d& pose, const descartes_planner::CapRung& cap_rung,
                        std::vector<double>& st_jt, std::vector<double>& end_jt)
  {

    st_jt.clear();
    end_jt.clear();
    return false;
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
  size_t rung_id = 0;
  CapVert* ptr_prev_vert = NULL;

  for(auto& cap_rung : cap_rungs_)
  {
    const auto unit_search_start = ros::Time::now();
    auto unit_search_update = unit_search_start;

    do
    {
      Eigen::Affine3d pose = generateSample(rung_id);
      std::vector<double> start_jt;
      std::vector<double> end_jt;

      if(checkFeasibility(pose, cap_rung, start_jt, end_jt))
      {
        // initial solution found, make cap_vert
        CapVert cap_vert(rung_id, start_jt, end_jt);
        cap_vert.setParentVertPtr(ptr_prev_vert);
        cap_rung.cap_verts_.push_back(cap_vert);
        break;
      }

      unit_search_update = ros::Time::now();
    }
    while((unit_search_update - unit_search_start).toSec() < UNIT_PROCESS_TIMEOUT);

    if(0 == cap_rung.cap_verts_.size())
    {
      ROS_ERROR_STREAM("[CapRRTstar] process #" << rung_id
                                                << " fails to find intial feasible sol with timeout "
                                                << UNIT_PROCESS_TIMEOUT);

      return std::numeric_limits<double>::max();
    }

    rung_id++;
  } // for cap_rungs

  // return last vert's cost

  // RRT* improve on the tree

}

void CapsulatedLadderTreeRRTstar::extractSolution(std::vector<descartes_core::TrajectoryPtPtr>& sol)
{
  // construct unit ladder graph for each cap rung

  // carry out DAG search on each ladder graph
}

} //end namespace descartes planner
