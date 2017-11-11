//
// Created by yijiangh on 11/11/17.
//

#include <descartes_parser/descartes_parser.h>

# include <descartes_msgs/LadderGraphEdge.h>
# include <descartes_msgs/LadderGraphEdgeList.h>
# include <descartes_msgs/LadderGraphRung.h>

namespace descartes_parser
{

descartes_msgs::LadderGraph convertToLadderGraphMsg(const descartes_planner::LadderGraph& graph)
{
  descartes_msgs::LadderGraph graph_msg;
  graph_msg.rungs.reserve(graph.size());

  graph_msg.dof = graph.dof();

  std::size_t i = 0;
  do
  {
    descartes_planner::Rung rung = graph.getRung(i);
    descartes_msgs::LadderGraphRung rung_msg;

    rung_msg.dt = rung.timing.lower;
    rung_msg.data = rung.data;
    rung_msg.edges.reserve(rung.edges.size());

    // copy edge list
    for(auto e_list : rung.edges)
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
  } while(i < graph.size());

  return graph_msg;
}

}
