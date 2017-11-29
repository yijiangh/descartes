//
// Created by yijiangh on 11/28/17.
//

#include "descartes_planner/capsulated_ladder_tree.h"
#include "descartes_planner/capsulated_ladder_tree_RRTstar.h"

// ladder graph & DAG (solution extraction)
#include "descartes_planner/ladder_graph.h"
#include "descartes_planner/ladder_graph_dag_search.h"

// pose conversion
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>

#include <random>

// unit process sampling timeout in initial solution searching
const static double UNIT_PROCESS_TIMEOUT = 30.0;
// total timeout for RRTstar
const static double RRTSTAR_TIMEOUT = 120.0;

namespace // anon namespace to hide utility functions
{
// copy from graph_builder.cpp
// Generate evenly sampled point at some discretization 'ds' between start and stop.
// ds must be > 0
std::vector<Eigen::Vector3d> discretizePositions(const Eigen::Vector3d& start, const Eigen::Vector3d& stop, const double ds)
{
  auto dist = (stop - start).norm();

  size_t n_intermediate_points = 0;
  if (dist > ds)
  {
    n_intermediate_points = static_cast<size_t>(std::lround(dist / ds));
  }

  const auto total_points = 2 + n_intermediate_points;

  std::vector<Eigen::Vector3d> result;
  result.reserve(total_points);

  for (std::size_t i = 0; i < total_points; ++i)
  {
    const double r = i / static_cast<double>(total_points - 1);
    Eigen::Vector3d point = start + (stop - start) * r;
    result.push_back(point);
  }
  return result;
}

// copy from graph_builder.cpp
Eigen::Affine3d makePose(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation,
                         const double z_axis_angle)
{
  Eigen::Affine3d m = Eigen::Affine3d::Identity();
  m.matrix().block<3,3>(0,0) = orientation;
  m.matrix().col(3).head<3>() = position;

  Eigen::AngleAxisd z_rot (z_axis_angle, Eigen::Vector3d::UnitZ());

  return m * z_rot;
}

std::vector<Eigen::Affine3d> generateSample(size_t rung_id, const descartes_planner::CapRung& cap_rung)
{
  // sample int for orientation
  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

  int o_sample;
  if(cap_rung.orientations_.size() > 1)
  {
    std::uniform_int_distribution<int> orient_distr(0, cap_rung.orientations_.size()-1);
    o_sample = orient_distr(gen);
  }
  else
  {
    o_sample = 0;
  }
  Eigen::Matrix3d orientation_sample = cap_rung.orientations_[o_sample];

//  ROS_INFO_STREAM("[CAPRRT] orient sample " << o_sample << "/" << cap_rung.orientations_.size());

  // sample [0,1] for axis, z_axis_angle = b_rand * 2 * Pi
  std::uniform_real_distribution<double> x_axis_distr(0.0, 1.0);
  double x_axis_sample = x_axis_distr(gen) * 2 * M_PI;
//  ROS_INFO_STREAM("[CAPRRT] x_axis sample " << x_axis_sample);

  std::vector<Eigen::Affine3d> poses;
  poses.reserve(cap_rung.path_pts_.size());
  for(auto& pt : cap_rung.path_pts_)
  {
    poses.push_back(makePose(pt, orientation_sample, x_axis_sample));
  }

  return poses;
}

// if pose is feasible, return true and start and end joint solution
// otherwise return empty joint solution
bool checkFeasibility(
    descartes_core::RobotModel& model,
    const std::vector<Eigen::Affine3d>& poses, descartes_planner::CapRung& cap_rung,
    std::vector<double>& st_jt, std::vector<double>& end_jt)
{
  model.setPlanningScene(cap_rung.planning_scene_);

  // TODO: add conflict index, first check conflict indices
  size_t pose_id = 0;
  for(auto& pose : poses)
  {
    std::vector<std::vector<double>> joint_poses;
    model.getAllIK(pose, joint_poses);

    if(joint_poses.empty())
    {
      cap_rung.conflict_id_.push_back(pose_id);
      st_jt.clear();
      end_jt.clear();
      return false;
    }
    else
    {
      if(0 == pose_id)
      {
        // turn packed joint solution in a contiguous array
        for (const auto& sol : joint_poses)
        {
          st_jt.insert(st_jt.end(), sol.cbegin(), sol.cend());
        }
      }
      if(poses.size()-1 == pose_id)
      {
        // turn packed joint solution in a contiguous array
        for (const auto& sol : joint_poses)
        {
          end_jt.insert(end_jt.end(), sol.cbegin(), sol.cend());
        }
      }
    }

    pose_id++;
  }

  // all poses are valid (have feasible ik solutions)!
  return true;
}
} //end util namespace

namespace descartes_planner
{
CapsulatedLadderTreeRRTstar::CapsulatedLadderTreeRRTstar(
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

    auto points = discretizePositions(segs[i].start, segs[i].end, segs[i].linear_disc);
    cap_rung.path_pts_ = points;

    cap_rung.orientations_.reserve(segs[i].orientations.size());
    for (auto& orient : segs[i].orientations)
    {
      cap_rung.orientations_.push_back(orient);
    }

    cap_rung.planning_scene_ = planning_scenes[i];

    cap_rungs_.push_back(cap_rung);
  }
}

double CapsulatedLadderTreeRRTstar::solve(descartes_core::RobotModel& model)
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
      std::vector<Eigen::Affine3d> poses = generateSample(rung_id, cap_rung);
      std::vector<double> start_jt;
      std::vector<double> end_jt;

      if(checkFeasibility(model, poses, cap_rung, start_jt, end_jt))
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
      // random sampling fails to find a solution
      //if discrete_enumeration fails

      // else
      ROS_ERROR_STREAM("[CapRRTstar] process #" << rung_id
                                                << " fails to find intial feasible sol within timeout "
                                                << UNIT_PROCESS_TIMEOUT);

      return std::numeric_limits<double>::max();
    }

    ROS_INFO_STREAM("[CLTRRT]ik solution found for process #" << rung_id);
    rung_id++;
  } // for cap_rungs

  // return last vert's cost
//  double last_cost = cap_rungs_.back().cap_verts_.back().getCost();
  return 0.0;

  // RRT* improve on the tree

}

void CapsulatedLadderTreeRRTstar::extractSolution(std::vector<descartes_core::TrajectoryPtPtr>& sol)
{
  // construct unit ladder graph for each cap rung

  // carry out DAG search on each ladder graph
}

} //end namespace descartes planner
