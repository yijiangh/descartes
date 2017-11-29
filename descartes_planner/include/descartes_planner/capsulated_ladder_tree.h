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
  explicit CapVert(const size_t rung_id,
                   const std::vector<double> &st_jt, const std::vector<double> &end_jt)
      : start_joint_data_(st_jt), end_joint_data_(end_jt), to_parent_cost_(0), ptr_prev_cap_vert_(NULL)
  {
  }

  ~CapVert()
  {
    if (NULL != ptr_prev_cap_vert_)
    {
      delete ptr_prev_cap_vert_;
      ptr_prev_cap_vert_ = NULL;
    }
  }

  inline double distance(CapVert* v) const
  {
    // sanity check
    if(NULL == v)
    {
      return 0.0;
    }

    assert(v->dof_ = this->dof_);

    double cost = std::numeric_limits<double>::max();
    const auto n_prev = v->end_joint_data_.size() / v->dof_;
    const auto n_this = this->end_joint_data_.size() / this->dof_;

    for(size_t i=0; i < n_this; i++)
    {
      for (size_t j = 0; j < n_prev; j++)
      {
        std::vector<double> delta_buffer;
        // just use
        for (size_t k = 0; k < dof_; k++)
        {
          delta_buffer[i] = std::abs(this->start_joint_data_[i] - v->end_joint_data_[i]);
        }

        double tmp_cost = std::accumulate(delta_buffer.cbegin(), delta_buffer.cend(), 0.0);
        if(tmp_cost < cost)
        {
          cost = tmp_cost;
        }
      }
    }

    return cost;
  }

  inline double getToParentCost() { return this->to_parent_cost_; }

  inline const CapVert* getParentVertPtr() const { return this->ptr_prev_cap_vert_; }

  inline void setParentVertPtr(CapVert* ptr_v)
  {
    this->ptr_prev_cap_vert_ = ptr_v;
    this->to_parent_cost_ = this->distance(ptr_v);
  }

  // accumulated cost (the function g)
  inline double getCost()
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

 private:
  int dof_;

  // joint values stored in one contiguous array
  std::vector<double> start_joint_data_;
  std::vector<double> end_joint_data_;

  double to_parent_cost_;
  CapVert* ptr_prev_cap_vert_;
};

struct CapRung
{
  std::vector<CapVert> cap_verts_;

  std::vector<Eigen::Vector3d> path_pts_;
  std::vector<Eigen::Matrix3d> orientations_;
  planning_scene::PlanningScenePtr planning_scene_;
  std::vector<size_t> conflict_id_;

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
