#include "descartes_planner/graph_builder.h"
namespace descartes_planner
{
  typedef boost::function<double(const double*, const double*)> CostFunction;
}
#include "descartes_planner/planning_graph_edge_policy.h"

namespace // anon namespace to hide utility functions
{

using PositionVector = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

// Generate evenly sampled point at some discretization 'ds' between start and stop.
// ds must be > 0
PositionVector discretizePositions(const Eigen::Vector3d& start, const Eigen::Vector3d& stop, const double ds)
{
  auto dist = (stop - start).norm();

  size_t n_intermediate_points = 0;
  if (dist > ds)
    n_intermediate_points = static_cast<size_t>(std::lround(dist / ds));

  const auto total_points = 2 + n_intermediate_points;

  PositionVector result;
  result.reserve(total_points);

  for (std::size_t i = 0; i < total_points; ++i)
  {
    const double r = i / static_cast<double>(total_points - 1);
    Eigen::Vector3d point = (stop - start) * r;
    result.push_back(point);
  }
}

Eigen::Affine3d makePose(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation,
                         const double z_axis_angle)
{
  Eigen::Affine3d m = Eigen::Affine3d::Identity();
  m.matrix().block<3,3>(0,0) = orientation;
  m.matrix().col(3).head<3>() = position;

  Eigen::AngleAxisd z_rot (z_axis_angle, Eigen::Vector3d::UnitZ());

  return m * z_rot;
}

descartes_planner::LadderGraph sampleSingleConfig(const descartes_core::RobotModel& model, const PositionVector& ps,
                                                  const double dt, const Eigen::Matrix3d& orientation,
                                                  const double z_axis_angle)
{
  descartes_planner::LadderGraph graph {model.getDOF()};
  graph.resize(ps.size());

  const descartes_core::TimingConstraint timing (dt);

  // Solve IK for each point
  for (std::size_t i = 0; i < ps.size(); ++i) //const auto& point : ps)
  {
    const auto& point = ps[i];
    const Eigen::Affine3d pose = makePose(point, orientation, z_axis_angle);
    std::vector<std::vector<double>> joint_poses;
    model.getAllIK(pose, joint_poses);

    graph.assignRung(i, descartes_core::TrajectoryID::make_nil(), timing, joint_poses);
  }

  // now we have a graph with data in the 'rungs' and we need to compute the edges
  for (std::size_t i = 0; i < graph.size() - 1; ++i)
  {
    const auto start_idx = i;
    const auto end_idx = i + 1;
    const auto& joints1 = graph.getRung(start_idx).data;
    const auto& joints2 = graph.getRung(end_idx).data;
    const auto& tm = graph.getRung(end_idx).timing;
    const auto dof = model.getDOF();

    const auto start_size = joints1.size() / dof;
    const auto end_size = joints2.size() / dof;

    descartes_planner::DefaultEdgesWithTime builder (start_size, end_size, dof, tm.upper,
                                                     model.getJointVelocityLimits());
    for (size_t i = 0; i < start_size; i++) // from rung
    {
      const auto start_index = i * dof;

      for (size_t j = 0; j < end_size; j++) // to rung
      {
        const auto end_index = j * dof;

        builder.consider(&joints1[start_index], &joints2[end_index], j);
      }
      builder.next(i);
    }
    std::vector<descartes_planner::LadderGraph::EdgeList> edges = builder.result();
    graph.assignEdges(i, std::move(edges));
  } // end edge loop


  return graph;
}

void concatenate(descartes_planner::LadderGraph& dest, const descartes_planner::LadderGraph& src)
{
  assert(dest.size() > 0);
  assert(src.size() > 0);
  assert(dest.size() == src.size()); // same number of rungs
  // TODO
  // Combines the joints and edges from one graph, src, into another, dest
  // The joints copy straight over, but the index of the points needs to be transformed
  for (std::size_t i = 0; i < src.size() - 1; ++i)
  {
    // Copy the joints
    auto& dest_joints = dest.getRung(i).data;
    const auto& src_joints = src.getRung(i).data;
    dest_joints.insert(dest_joints.end(), src_joints.begin(), src_joints.end());

    // Copy the edges and transform them
    const auto next_rung_size = dest.rungSize(i + 1);
    auto& dest_edges = dest.getEdges(i);
    const auto& src_edges = dest.getEdges(i);
    for (const auto& edge : src_edges)
    {
      auto edge_copy = edge;
      for (auto& e : edge_copy)
        e.idx += next_rung_size;
      dest_edges.push_back(edge_copy);
    }
  }
}

} // end anon utility function ns

descartes_planner::LadderGraph descartes_planner::sampleConstrainedPaths(const descartes_core::RobotModel& model,
                                                                         const ConstrainedSegment& segment)
{
  // Determine the linear points
  auto points = discretizePositions(segment.start, segment.end, segment.linear_disc);
  // Compute the number of angle steps
  static const auto min_angle = -M_PI_2;
  static const auto max_angle = M_PI_2;
  const auto n_angle_disc = std::lround( (max_angle - min_angle) / segment.z_axis_disc);
  const auto angle_step = (max_angle - min_angle) / n_angle_disc;
  // Compute the expected time step for each linear point
  const auto dt = (segment.end - segment.start).norm() / segment.linear_vel;

  LadderGraph graph {model.getDOF()};
  graph.resize(points.size()); // there will be a ladder rung for each point that we must solve

  // We will build up our graph one configuration at a time: a configuration is a single orientation and z angle disc
  for (const auto& orientation : segment.orientations)
  {
    for (long i = 0; i < n_angle_disc; ++i)
    {
      const auto angle = angle_step * i;
      LadderGraph single_config_graph = sampleSingleConfig(model, points, dt, orientation, angle);
      concatenate(graph, single_config_graph);
    }
  }

  return graph;
}
