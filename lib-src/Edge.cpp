#include "Edge.h"
#include <boost/unordered_map.hpp>

Edge::Edge(const Coordinate& src, const Coordinate& dst): source(src), destination(dst) {}


std::string Edge::ToString(void) const
{
    return source.ToString() + " ---> " + destination.ToString();
}

std::string Edge::ToStringLog(void) const
{
    return "<" + source.ToString() + ", " +  destination.ToString() + ">";
}

std::ostream& operator << (std::ostream& out,const Edge& edge)
{
    return out << edge.ToString();
}

Edge::Edges Edge::GetCrossingEdges(void) const noexcept
{
    Edges crossing_edges = {this->InvertedEdge()};
    if(IsDiagonalEdge())
    {
        const auto e = CrossingDiagonalEdge();
        crossing_edges.push_back(e);
        crossing_edges.push_back(e.InvertedEdge());
    }
    return crossing_edges;
}

bool Edge::IsSelfLoopEdge(void) const noexcept
{
    return source == destination;
}

bool Edge::IsDiagonalEdge(void) const noexcept
{
    return (std::abs(source.row - destination.row) == 1) && (std::abs(source.column - destination.column) == 1);
}

Edge Edge::CrossingDiagonalEdge(void) const noexcept
{
    return {{source.row + (destination.row - source.row), source.column}, {source.row, source.column + (destination.column - source.column)}};
}

Edge Edge::InvertedEdge(void) const noexcept
{
    return {destination, source};
}

bool Edge::operator == (const Edge &other) const noexcept
{
    return source == other.source && destination == other.destination;
}

bool Edge::operator != (const Edge &other) const noexcept
{
    return !(*this == other);
}

size_t Edge::Hasher::operator()(const Edge& edge) const noexcept
{
    std::size_t seed = 0;
    Coordinate::Hasher coordinate_hasher;
    boost::hash_combine(seed, coordinate_hasher(edge.source));
    boost::hash_combine(seed, coordinate_hasher(edge.destination));
    return seed;
}

bool Edge::Equal::operator()(const Edge &e1, const Edge &e2) const noexcept
{
    return e1 == e2;
}