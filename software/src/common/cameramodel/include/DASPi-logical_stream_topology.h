// DASPi-logical_stream_topology.h
#pragma once
struct LogicalStreamTopology {
    int owningFaceIndex{-1};
    int localEdgeIndex{-1};     // -1 for non-overlap
    int neighborFaceIndex{-1};  // -1 for non-overlap
    int edgeIndex{-1};          // -1 for non-overlap
};
