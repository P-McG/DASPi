// DASPi-face_adjacency_entry.h
#pragma once

namespace DASPi {

struct FaceAdjacencyEntry {
    int neighborFaceIndex{-1};

    int edgeIndex{-1};

    // Which local edge of this face [0..N-1]
    int localEdgeIndex{-1};

    // Which local edge of the neighbouring face [0..N-1]
    int neighborLocalEdgeIndex{-1};

    // True if this face traverses the shared edge in the same direction
    // as the neighbouring face.
    //
    // For a consistently wound closed triangle mesh, this should usually
    // be false for adjacent faces.
    bool sameDirectionAsNeighbor{false};
};

} // namespace DASPi
