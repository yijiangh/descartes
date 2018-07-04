#ifndef DESCARTES_PARSER_H
#define DESCARTES_PARSER_H

#include <descartes_planner/ladder_graph.h>

// msgs
#include <descartes_msgs/LadderGraph.h>
#include <descartes_msgs/LadderGraphList.h>

namespace descartes_parser
{

descartes_msgs::LadderGraph convertToLadderGraphMsg(const descartes_planner::LadderGraph& graph,
                                                    std::vector<int> graph_indices = std::vector<int>());

descartes_msgs::LadderGraphList convertToLadderGraphListMsg(const std::vector<descartes_planner::LadderGraph>& graphs,
                                                            std::vector<std::vector<int>> graph_indices = std::vector<std::vector<int>>());

descartes_planner::LadderGraph convertToLadderGraph(const descartes_msgs::LadderGraph& graph_msg);

descartes_planner::LadderGraph convertToLadderGraph(const descartes_msgs::LadderGraph& graph_msg, std::vector<int>& partition_ids);

std::vector<descartes_planner::LadderGraph> convertToLadderGraphList(const descartes_msgs::LadderGraphList& graph_list_msg);

std::vector<descartes_planner::LadderGraph> convertToLadderGraphList(
    const descartes_msgs::LadderGraphList& graph_list_msg, std::vector<std::vector<int>>& partition_ids);

}

#endif // DESCARTES_PARSER_H
