#ifndef DESCARTES_PARSER_H
#define DESCARTES_PARSER_H

# include <descartes_planner/ladder_graph.h>

# include <descartes_msgs/LadderGraph.h>

namespace descartes_parser
{

descartes_msgs::LadderGraph convertToLadderGraphMsg(const descartes_planner::LadderGraph& graph);

descartes_planner::LadderGraph convertToLadderGraph(const descartes_msgs::LadderGraph& graph_msg);

}

#endif // DESCARTES_PARSER_H