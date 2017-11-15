//
// Created by yijiangh on 11/11/17.
//

#include <descartes_parser/descartes_parser.h>

#include <descartes_msgs/LadderGraphEdge.h>
#include <descartes_msgs/LadderGraphEdgeList.h>
#include <descartes_msgs/LadderGraphRung.h>
#include <descartes_msgs/LadderGraph.h>
#include <descartes_msgs/LadderGraphList.h>

namespace descartes_parser
{

descartes_msgs::LadderGraph convertToLadderGraphMsg(const descartes_planner::LadderGraph* ptr_graph)
{
  descartes_msgs::LadderGraph graph_msg;
  graph_msg.rungs.reserve(ptr_graph->size());

  graph_msg.dof = ptr_graph->dof();

  std::size_t i = 0;
  do
  {
    descartes_planner::Rung rung = ptr_graph->getRung(i);
    descartes_msgs::LadderGraphRung rung_msg;

    rung_msg.dt = rung.timing.lower;
    rung_msg.data = rung.data;
    rung_msg.edges.reserve(rung.edges.size());

    // copy edge list
    for(auto& e_list : rung.edges)
    {
      descartes_msgs::LadderGraphEdgeList edge_list_msg;
      edge_list_msg.edge_list.reserve(e_list.size());

      for(descartes_planner::Edge e : e_list)
      {
        descartes_msgs::LadderGraphEdge edge_msg;
        edge_msg.cost = e.cost;
        edge_msg.idx = e.idx;

        edge_list_msg.edge_list.push_back(edge_msg);
      }

      rung_msg.edges.push_back(edge_list_msg);
    }

    graph_msg.rungs.push_back(rung_msg);

    i++;
  } while(i < ptr_graph->size());

  return graph_msg;
}

descartes_msgs::LadderGraphList convertToLadderGraphMsg(const std::vector<descartes_planner::LadderGraph*>& graphs)
{
  descartes_msgs::LadderGraphList graph_list_msg;

  for(const auto& graph_ptr : graphs)
  {
    graph_list_msg.graph_list.push_back(convertToLadderGraphMsg(graph_ptr));
  }

  return graph_list_msg;
}

descartes_planner::LadderGraph* convertToLadderGraphPtr(const descartes_msgs::LadderGraph& graph_msg)
{
  auto* ptr_graph = new descartes_planner::LadderGraph(graph_msg.dof);
  ptr_graph->resize(graph_msg.rungs.size());

  for(std::size_t i=0; i < graph_msg.rungs.size(); i++)
  {
    const descartes_msgs::LadderGraphRung& r_msg = graph_msg.rungs[i];

    const descartes_core::TimingConstraint timing(r_msg.dt);

    ptr_graph->assignRung(i, descartes_core::TrajectoryID::make_nil(), timing, r_msg.data);

    std::vector<std::vector<descartes_planner::Edge>> rung_edges;
    rung_edges.reserve(r_msg.edges.size());

    // assign egde_list
    for(const auto& elist_msg : r_msg.edges)
    {
      std::vector<descartes_planner::Edge> elist;
      elist.clear();
      elist.reserve(elist_msg.edge_list.size());

      for(const auto& e_msg : elist_msg.edge_list)
      {
        descartes_planner::Edge e;
        e.cost = e_msg.cost;
        e.idx = e_msg.idx;

        elist.push_back(e);
      }
      rung_edges.push_back(elist);
    }

    ptr_graph->assignEdges(i, std::move(rung_edges));
  }

  return ptr_graph;
}

std::vector<descartes_planner::LadderGraph*> convertToLadderGraphPtrList(const descartes_msgs::LadderGraphList& graph_list_msg)
{
  std::vector<descartes_planner::LadderGraph*> graph_ptr_list;

  graph_ptr_list.reserve(graph_list_msg.graph_list.size());

  for(const auto& graph_msg : graph_list_msg.graph_list)
  {
    graph_ptr_list.push_back(convertToLadderGraphPtr(graph_msg));
  }

  return graph_ptr_list;
}
} // end namespace descartes_parser