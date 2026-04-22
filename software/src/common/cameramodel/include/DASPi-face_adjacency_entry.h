// DASPi-face_adjacency_entry.h
#pragma once

namespace DASPi{
struct FaceAdjacencyEntry {
    int neighborFaceIndex{-1};
    int edgeIndex{-1};
    int localEdgeIndex{-1}; // which local edge of this face [0..N-1]
};
};//DASPi
