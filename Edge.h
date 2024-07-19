#pragma once

#include "Coordinate.h"
#include <string>
#include <vector>

struct Edge
{
    using Edges = std::vector<Edge>;
    Coordinate source, destination;

    Edge() = default;
    Edge(const Coordinate& src, const Coordinate& dst);
    Edges GetCrossingEdges(void) const noexcept;
    Edge CrossingDiagonalEdge(void) const noexcept;
    Edge InvertedEdge(void) const noexcept;
    std::string ToString(void) const;
    std::string ToStringLog(void) const;
    bool IsSelfLoopEdge(void) const noexcept;
    bool IsDiagonalEdge(void) const noexcept;
    friend std::ostream& operator<<(std::ostream& out, const Edge& edge);

    bool operator == (const Edge&) const noexcept;
    bool operator != (const Edge&) const noexcept;
    struct Hasher{size_t operator()(const Edge& edge) const noexcept;};
    struct Equal{bool operator()(const Edge& e1, const Edge& e2) const noexcept;};
};