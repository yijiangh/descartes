//
// Created by yijiangh on 11/28/17.
//

#ifndef DESCARTES_CAPSULATED_LADDER_TREE_H
#define DESCARTES_CAPSULATED_LADDER_TREE_H

#include <moveit/planning_scene/planning_scene.h>

namespace descartes_planner
{

class CapVert
{
 public:
  CapVert(const size_t rung_id,
          const std::vector<double>& st_jt, const std::vector<double>& end_jt)
      : start_joint_(st_jt), end_joint_(end_jt), to_parent_cost_(0), ptr_prev_cap_vert_(NULL)
  {
  }

  ~CapVert()
  {
    if(NULL != ptr_prev_cap_vert_)
    {
      delete ptr_prev_cap_vert_;
      ptr_prev_cap_vert_ = NULL;
    }
  }

  inline double distance(CapVert& v) const
  {
    std::vector<double> delta_buffer;
    // just use
    for (size_t i = 0; i < dof_; ++i)
    {
      delta_buffer[i] = std::abs(this->start_joint[i] - v.end_joint[i]);
    }

    double cost = std::accumulate(delta_buffer_.cbegin(), delta_buffer_.cend(), 0.0);
  }

  inline double getToParentCost() { return this->to_parent_cost_; }

  inline const CapVert* getParentVertPtr() const { return this->ptr_prev_cap_vert_; }

  inline void setParentVertPtr(CapVert* ptr_v) { this->ptr_prev_cap_vert_ = ptr_v; }

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
  std::vector<double> start_joint_;
  std::vector<double> end_joint_;

  double to_parent_cost_;
  CapVert* ptr_prev_cap_vert_;
};

struct CapRung
{
  std::vector<CapVert> cap_verts;
  planning_scene::PlanningScenePtr planning_scene;
  std::vector<int> conflict_id;
};
}

#endif //DESCARTES_CAPSULATED_LADDER_TREE_H
