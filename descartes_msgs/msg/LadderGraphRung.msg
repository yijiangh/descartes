# id - not used at this moment

# time_constraints
float64 dt

# joint values stored in one contiguous array
float64[] data

# edge list stores connection and cost info to the
# next rung
descartes_msgs/LadderGraphEdgeList[] edges
