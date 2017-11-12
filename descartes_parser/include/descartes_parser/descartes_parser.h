#ifndef DESCARTES_PARSER_H
#define DESCARTES_PARSER_H

#include <descartes_planner/ladder_graph.h>

// msgs
#include <descartes_msgs/LadderGraph.h>
#include <descartes_msgs/LadderGraphList.h>

namespace descartes_parser
{

descartes_msgs::LadderGraph convertToLadderGraphMsg(const descartes_planner::LadderGraph& graph);
descartes_msgs::LadderGraphList convertToLadderGraphMsg(const std::vector<descartes_planner::LadderGraph>& graphs);

descartes_planner::LadderGraph convertToLadderGraph(const descartes_msgs::LadderGraph& graph_msg);
std::vector<descartes_planner::LadderGraph> convertToLadderGraphList(const descartes_msgs::LadderGraphList& graph_list_msg);
}

#endif // DESCARTES_PARSER_H