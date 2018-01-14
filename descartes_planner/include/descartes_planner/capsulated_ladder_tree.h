//
// Created by yijiangh on 11/28/17.
//

#ifndef DESCARTES_CAPSULATED_LADDER_TREE_H
#define DESCARTES_CAPSULATED_LADDER_TREE_H

// for Constrained Segment
#include "graph_builder.h"

#include <moveit/planning_scene/planning_scene.h>

namespace descartes_planner
{

class CapVert
{
 public:
  explicit CapVert(const double dof) : dof_(dof)
  {
  }

  ~CapVert()
  {}

  inline double distance(CapVert* v) const
  {
    // sanity check
    if(NULL == v)
    {
      return 0.0;
    }

    assert(v->dof_ == this->dof_);

    // directed distance prev.end -> this.start
    const auto dof = this->dof_;
    double cost = std::numeric_limits<double>::max();
    const auto n_prev_end = v->end_joint_data_.size() / dof;
    const auto n_this_start = this->start_joint_data_.size() / dof;

    for(size_t i=0; i < n_prev_end; i++)
    {
      const auto prev_end_index = i * dof;

      for (size_t j=0; j < n_this_start; j++)
      {
        const auto this_start_index = j * dof;

        std::vector<double> delta_buffer;

        for (size_t k = 0; k < dof; k++)
        {
          delta_buffer.push_back(std::abs(v->end_joint_data_[prev_end_index+k]
                                         - this->start_joint_data_[this_start_index+k]));
        }

        double tmp_cost = std::accumulate(delta_buffer.begin(), delta_buffer.end(), 0.0);

        if(tmp_cost < cost)
        {
          cost = tmp_cost;
        }
      }
    }

    return cost;
  }

  inline double getToParentCost() { return this->to_parent_cost_; }

  inline CapVert* getParentVertPtr() const { return this->ptr_prev_cap_vert_; }

  inline void setParentVertPtr(CapVert* ptr_v)
  {
    this->ptr_prev_cap_vert_ = ptr_v;
    this->to_parent_cost_ = this->distance(ptr_v);
  }

  // accumulated cost (the function g)
  inline double getCost() const
  {
    // trace back tree to compute cost
    CapVert* ptr_prev_v = this->ptr_prev_cap_vert_;
    double cost = this->to_parent_cost_;

    while(NULL != ptr_prev_v)
    {
      if(NULL != ptr_prev_v->ptr_prev_cap_vert_)
      {
        cost += ptr_prev_v->getToParentCost();
      }
      ptr_prev_v = ptr_prev_v->ptr_prev_cap_vert_;
    }

    return cost;
  }

 public:
  int dof_;

  // joint values stored in one contiguous array
  std::vector<double> start_joint_data_;
  std::vector<double> end_joint_data_;

  size_t rung_id_;
  double z_axis_angle_;
  Eigen::Matrix3d orientation_;

  double to_parent_cost_;
  CapVert* ptr_prev_cap_vert_;
};

struct CapRung
{
  std::vector<CapVert*> ptr_cap_verts_;

  std::vector<Eigen::Vector3d> path_pts_;
  std::vector<Eigen::Matrix3d> orientations_;
  planning_scene::PlanningScenePtr planning_scene_;

  // TODO: this is temporal patch to add element that is being printed
  planning_scene::PlanningScenePtr planning_scene_completed_;

  std::set<size_t> conflict_ids_;
  double z_axis_disc_;
  double linear_vel_;

  inline Eigen::Affine3d makePose(double rand_o, double rand_a) const
  {
    // sanity check - 0 <= rand_o, rand_a < 1
    if(!(0<= rand_o && rand_o < 1 && 0 <= rand_a && rand_a < 1))
    {
      ROS_ERROR("[Cap Ladder Tree] sample num should be in [0,1)!!");
    }
    assert(0<= rand_o && rand_o < 1 && 0 <= rand_a && rand_a < 1);

    // return pose
  }
};
}

#endif //DESCARTES_CAPSULATED_LADDER_TREE_H
