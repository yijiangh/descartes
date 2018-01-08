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

// unit process sampling timeout in initial solution searching
const static double UNIT_PROCESS_TIMEOUT = 30.0;
// total timeout for RRTstar
const static double RRTSTAR_TIMEOUT = 60.0;

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

static int randomSampleInt(int lower, int upper)
{
  std::random_device rd;
  std::mt19937 gen(rd());

  if(upper > lower)
  {
    std::uniform_int_distribution<int> int_distr(lower, upper);
    return int_distr(gen);
  }
  else
  {
    return lower;
  }
}

static double randomSampleDouble(double lower, double upper)
{
  std::random_device rd;
  std::mt19937 gen(rd());

  if(upper > lower)
  {
    std::uniform_real_distribution<double> double_distr(lower, upper);
    return double_distr(gen);
  }
  else
  {
    return lower;
  }
}

std::vector<Eigen::Affine3d> generateSample(const descartes_planner::CapRung& cap_rung,
                                            descartes_planner::CapVert& cap_vert)
{
  // sample int for orientation
  int o_sample = randomSampleInt(0, cap_rung.orientations_.size()-1);

  Eigen::Matrix3d orientation_sample = cap_rung.orientations_[o_sample];

  // sample [0,1] for axis, z_axis_angle = b_rand * 2 * Pi

  double x_axis_sample = randomSampleDouble(0.0, 1.0) * 2 * M_PI;

  std::vector<Eigen::Affine3d> poses;
  poses.reserve(cap_rung.path_pts_.size());
  for(auto& pt : cap_rung.path_pts_)
  {
    poses.push_back(makePose(pt, orientation_sample, x_axis_sample));
  }

  cap_vert.z_axis_angle_ = x_axis_sample;
  cap_vert.orientation_ = orientation_sample;
  return poses;
}

// if pose is feasible, return true and start and end joint solution
// otherwise return empty joint solution
bool checkFeasibility(
    descartes_core::RobotModel& model,
    const std::vector<Eigen::Affine3d>& poses, descartes_planner::CapRung& cap_rung,
    descartes_planner::CapVert& cap_vert)
{
  // sanity check
  assert(poses.size() == cap_rung.path_pts_.size());

  model.setPlanningScene(cap_rung.planning_scene_);
  std::vector<double> st_jt;
  std::vector<double> end_jt;

  // first check conflict indices
  std::vector<size_t> full_ids(poses.size());
  std::iota(full_ids.begin(), full_ids.end(), 0); // fill 0 ~ n-1
  std::set<size_t> full_ids_set(full_ids.begin(), full_ids.end());

  std::set<size_t> last_check_ids;
  std::set_difference(full_ids_set.begin(), full_ids_set.end(),
                      cap_rung.conflict_ids_.begin(), cap_rung.conflict_ids_.end(),
                      std::inserter(last_check_ids, last_check_ids.end()));

  std::vector<std::vector<double>> joint_poses;
  for(size_t c_id = 0; c_id < poses.size(); c_id++)
  {
    joint_poses.clear();
    model.getAllIK(poses[c_id], joint_poses);

    if(joint_poses.empty())
    {
      return false;
    }
    else
    {
      if(0 == c_id)
      {
        // turn packed joint solution in a contiguous array
        for (const auto& sol : joint_poses)
        {
          st_jt.insert(st_jt.end(), sol.cbegin(), sol.cend());
        }
        cap_vert.start_joint_data_ = st_jt;
      }
      if(poses.size()-1 == c_id)
      {
        // turn packed joint solution in a contiguous array
        for (const auto& sol : joint_poses)
        {
          end_jt.insert(end_jt.end(), sol.cbegin(), sol.cend());
        }
        cap_vert.end_joint_data_ = end_jt;
      }
    }
  }// end loop over poses

  // all poses are valid (have feasible ik solutions)!
  return true;
}

bool domainDiscreteEnumerationCheck(
    descartes_core::RobotModel& model,
    descartes_planner::CapRung& cap_rung,
    descartes_planner::CapVert& cap_vert)
{
  // direct enumeration of domain
  const std::vector<Eigen::Vector3d> pts = cap_rung.path_pts_;
  std::vector<Eigen::Affine3d> poses;
  poses.reserve(cap_rung.path_pts_.size());

  const auto n_angle_disc = std::lround( 2 * M_PI / cap_rung.z_axis_disc_);
  const auto angle_step = 2 * M_PI / n_angle_disc;

  for (const auto& orientation : cap_rung.orientations_)
  {
    for (long i = 0; i < n_angle_disc; ++i)
    {
      const auto angle = angle_step * i;
      poses.clear();

      for(const auto& pt : pts)
      {
        poses.push_back(makePose(pt, orientation, angle));
      }

      if(checkFeasibility(model, poses, cap_rung, cap_vert))
      {
        cap_vert.orientation_ = orientation;
        cap_vert.z_axis_angle_ = angle;
        return true;
      }
    }
  }

  return false;
}

static bool comparePtrCapVert(const descartes_planner::CapVert* lhs, const descartes_planner::CapVert* rhs)
{
  return (lhs->getCost() < rhs->getCost());
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

    // end effector path pts
    auto points = discretizePositions(segs[i].start, segs[i].end, segs[i].linear_disc);
    cap_rung.path_pts_ = points;

    // feasible orientations
    cap_rung.orientations_.reserve(segs[i].orientations.size());
    for (auto& orient : segs[i].orientations)
    {
      cap_rung.orientations_.push_back(orient);
    }

    // planning scene
    cap_rung.planning_scene_ = planning_scenes[i];

    // conflict indices is empty initially

    // input z axis disc
    cap_rung.z_axis_disc_ = segs[i].z_axis_disc;

    // linear speed
    cap_rung.linear_vel_ = segs[i].linear_vel;

    cap_rungs_.push_back(cap_rung);
  }
}

CapsulatedLadderTreeRRTstar::~CapsulatedLadderTreeRRTstar()
{
  for(auto& cap_rung : cap_rungs_)
  {
    for(auto& ptr_vert : cap_rung.ptr_cap_verts_)
    {
      if(NULL != ptr_vert)
      {
        delete ptr_vert;
        ptr_vert = NULL;
      }
    }
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
    CapVert* ptr_cap_vert = new CapVert(model.getDOF());

    do
    {
      std::vector<Eigen::Affine3d> poses = generateSample(cap_rung, *ptr_cap_vert);

      if(checkFeasibility(model, poses, cap_rung, *ptr_cap_vert))
      {
        ptr_cap_vert->setParentVertPtr(ptr_prev_vert);
        ptr_cap_vert->rung_id_ = rung_id;
        cap_rung.ptr_cap_verts_.push_back(ptr_cap_vert);
        ptr_prev_vert = ptr_cap_vert;
        break;
      }

      unit_search_update = ros::Time::now();
    }
    while((unit_search_update - unit_search_start).toSec() < UNIT_PROCESS_TIMEOUT);

    if(cap_rung.ptr_cap_verts_.empty())
    {
      // random sampling fails to find a solution
      ROS_ERROR_STREAM("[CapRRTstar] process #" << rung_id
                                                << " fails to find intial feasible sol within timeout "
                                                << UNIT_PROCESS_TIMEOUT << "secs");
      return std::numeric_limits<double>::max();
//      if(domainDiscreteEnumerationCheck(model, cap_rung, cap_vert))
//      {
//        // enumeration solution found!
//        cap_vert.setParentVertPtr(ptr_prev_vert);
//        cap_vert.rung_id_ = rung_id;
//        cap_rung.cap_verts_.push_back(cap_vert);
//      }
//      else
//      {
//        ROS_ERROR_STREAM("[CapRRTstar] process #" << rung_id
//                                                 << " fails to find initial feasible sol with input z_axis disc num: "
//                                                 << std::lround( 2 * M_PI / cap_rung.z_axis_disc_)
//                                                  << ", orientations num: " << cap_rung.orientations_.size()
//                                                  << ", path pts size" << cap_rung.path_pts_.size());
//        return std::numeric_limits<double>::max();
//
    }

    ROS_INFO_STREAM("[CLTRRT] ik solutions found for process #" << rung_id);
    rung_id++;
  } // loop over cap_rungs

  double initial_sol_cost = cap_rungs_.back().ptr_cap_verts_.back()->getCost();
  ROS_INFO_STREAM("[CLTRRT] initial sol found! cost: " << initial_sol_cost);
  ROS_INFO_STREAM("[CLTRRT] RRT* improvement starts.");

  // RRT* improve on the tree
  const auto rrt_start_time = ros::Time::now();

  while((ros::Time::now() - rrt_start_time).toSec() < RRTSTAR_TIMEOUT)
  {
    // sample cap_rung
    int rung_id_sample = randomSampleInt(0, cap_rungs_.size()-1);
    auto& cap_rung_sample = cap_rungs_[rung_id_sample];

    // sample cap_vert
    CapVert* ptr_new_vert = new CapVert(model.getDOF());
    auto poses = generateSample(cap_rung_sample, *ptr_new_vert);

    if(checkFeasibility(model, poses, cap_rung_sample, *ptr_new_vert))
    {
      // find nearest node in tree
      double c_min = std::numeric_limits<double>::max();
      CapVert* ptr_nearest_vert = NULL;

      if(rung_id_sample > 0)
      {
        for(auto& ptr_near_vert : this->cap_rungs_[rung_id_sample-1].ptr_cap_verts_)
        {
          double new_near_cost = ptr_near_vert->getCost() + ptr_new_vert->distance(ptr_near_vert);
          if(c_min > new_near_cost)
          {
            ptr_nearest_vert = ptr_near_vert;
            c_min = new_near_cost;
          }
        }
      }

      // add new vert into CL tree (rung_id, parent_vert)
      ptr_new_vert->rung_id_ = rung_id_sample;
      ptr_new_vert->setParentVertPtr(ptr_nearest_vert);
      cap_rung_sample.ptr_cap_verts_.push_back(ptr_new_vert);

      // update vert next (repair tree)
      if(rung_id_sample < this->cap_rungs_.size()-1)
      {
        double new_vert_cost = ptr_new_vert->getCost();
        for(auto& ptr_next_vert : this->cap_rungs_[rung_id_sample+1].ptr_cap_verts_)
        {
          double old_next_cost = ptr_next_vert->getCost();
          double new_next_cost = new_vert_cost + ptr_next_vert->distance(ptr_new_vert);
          if(old_next_cost > new_next_cost)
          {
            ptr_next_vert->setParentVertPtr(ptr_new_vert);
          }
        }
      }
    } // end if capvert feasible
  }

  CapVert* ptr_last_cap_vert = *std::min_element(this->cap_rungs_.back().ptr_cap_verts_.begin(),
                                                 this->cap_rungs_.back().ptr_cap_verts_.end(), comparePtrCapVert);
  double rrt_cost = ptr_last_cap_vert->getCost();

  ROS_INFO_STREAM("[CLTRRT] RRT* sol cost " << rrt_cost
                                            << " after " << RRTSTAR_TIMEOUT << " secs.");
  return rrt_cost;
}

void CapsulatedLadderTreeRRTstar::extractSolution(descartes_core::RobotModel& model,
                                                  std::vector<descartes_core::TrajectoryPtPtr>& sol,
                                                  std::vector<descartes_planner::LadderGraph>& graphs,
                                                  std::vector<int>& graph_indices,
                                                  const bool use_saved_graph)
{
  const auto graph_build_start = ros::Time::now();

  if(!use_saved_graph)
  {
    graphs.clear();
    graph_indices.clear();

    // find min cap_vert on last cap_rung
    CapVert* ptr_last_cap_vert = *std::min_element(this->cap_rungs_.back().ptr_cap_verts_.begin(),
                                                   this->cap_rungs_.back().ptr_cap_verts_.end(), comparePtrCapVert);
    while (ptr_last_cap_vert != NULL)
    {
      // construct unit ladder graph for each cap rungpath_pts_
      const auto cap_rung = cap_rungs_[ptr_last_cap_vert->rung_id_];
      double traverse_length = (cap_rung.path_pts_.front() - cap_rung.path_pts_.back()).norm();
      const auto dt = traverse_length / cap_rung.linear_vel_;

      model.setPlanningScene(cap_rung.planning_scene_);
      auto unit_ladder_graph = sampleSingleConfig(model,
                                                  cap_rungs_[ptr_last_cap_vert->rung_id_].path_pts_,
                                                  dt,
                                                  ptr_last_cap_vert->orientation_,
                                                  ptr_last_cap_vert->z_axis_angle_);

      graphs.insert(graphs.begin(), unit_ladder_graph);
      graph_indices.insert(graph_indices.begin(), unit_ladder_graph.size());
      ptr_last_cap_vert = ptr_last_cap_vert->getParentVertPtr();
    }
  }
  else
  {
    graph_indices.clear();
    for(const auto& graph : graphs)
    {
      graph_indices.push_back(graph.size());
    }
  }

  // unify unit ladder graphs into one
  descartes_planner::LadderGraph unified_graph(model.getDOF());
  for(auto& graph : graphs)
  {
    assert(unified_graph.dof() == graph.dof());
    descartes_planner::appendInTime(unified_graph, graph);
  }

  // carry out DAG search on each ladder graph
  descartes_planner::DAGSearch search(unified_graph);
  double cost = search.run();
  auto path_idxs = search.shortestPath();

  sol.clear();
  for (size_t j = 0; j < path_idxs.size(); ++j)
  {
    const auto idx = path_idxs[j];
    const auto* data = unified_graph.vertex(j, idx);
    const auto& tm = unified_graph.getRung(j).timing;
    auto pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::JointTrajectoryPt(
        std::vector<double>(data, data + 6), tm));
    sol.push_back(pt);
  }

  auto graph_build_end = ros::Time::now();
  ROS_INFO_STREAM("[CLTRRT] Graph construction and searching took: "
                      << (graph_build_end - graph_build_start).toSec() << " seconds");
}
} //end namespace descartes planner
