#include "Snapshot.h"
#include "Constants.h"
#include "Edge.h"
#include "Graph.h"
#include "Types.h"
#include <sstream>
#include <string>
#include <utility>

Snapshot::Snapshot(const CoordinateSet& V, const EdgeSet& open, const EdgeSet& maybe_open, const EdgeSet& maybe_blocked):
V(V), open(open), maybe_open(maybe_open), maybe_blocked(maybe_blocked) {}

Snapshot::Snapshot(const Snapshot& other): V(other.V), open(other.open), maybe_open(other.maybe_open), maybe_blocked(other.maybe_blocked){}
Snapshot::Snapshot(Snapshot&& other): V(std::forward<CoordinateSet>(other.V)), open(std::forward<EdgeSet>(other.open)), 
maybe_open(std::forward<EdgeSet>(other.maybe_open)), maybe_blocked(std::forward<EdgeSet>(other.maybe_blocked)){}

Graph Snapshot::Create(void) const
{
    EdgeWeightFunction W;
    EdgeSet E{open};
    EdgeSet e_maybe_open{maybe_open};
    E.merge(e_maybe_open);
    assert(e_maybe_open.empty()); // verify no edge is contained in either open and maybe open

    for(const auto& e: E)
    {
        W[e] = EDGE_UNIT_COST_WEIGHT;
    }

    EdgeSet e_maybe_blocked(maybe_blocked);    
    for(const auto& e: e_maybe_blocked)
    {
        W[e] = INF;
    }

    E.merge(e_maybe_blocked);
    assert(e_maybe_blocked.empty()); // verify no edge is contained in either open and maybe blocked

    return Graph(V, E, W);
}

Graph Snapshot::Create(bool include_maybe_open, bool include_maybe_blocked) const
{
    EdgeSet E{open};

    if(include_maybe_open)
    {
        EdgeSet e_maybe_open{maybe_open};
        E.merge(e_maybe_open);
        assert(e_maybe_open.empty()); // verify no edge is contained in either open and maybe open
    }

    if(include_maybe_blocked)
    {
        EdgeSet e_maybe_blocked(maybe_blocked);
        E.merge(e_maybe_blocked);
        assert(e_maybe_blocked.empty()); // verify no edge is contained in either open and maybe blocked
    }

    return Graph(V, E);
}

bool Snapshot::IsMaybeOpenEdge(const Edge& e) const
{
    return maybe_open.find(e) != maybe_open.end();
}

bool Snapshot::IsMaybeBlockedEdge(const Edge& e) const
{
    return maybe_blocked.find(e) != maybe_blocked.end();
}

bool Snapshot::IsOpenEdge(const Edge& e) const
{
    return open.find(e) != open.end();
}

std::string Snapshot::ToString(void) const
{
    std::stringstream ss;

    ss << EdgeSetToString(open, "Eopen") << '\n';
    ss << EdgeSetToString(maybe_open, "Emaybe_open") << '\n';
    ss << EdgeSetToString(maybe_blocked, "Emaybe_blocked") << '\n';

    return ss.str();
}

std::string Snapshot::EdgeSetToString(const EdgeSet& s, const std::string& s_name) const
{
    std::stringstream ss;
    
    ss << s_name << '\n';
    int i = 1;

    for(const auto& e: s)
    {
        ss << std::to_string(i) << ")\t" << e.ToString() << '\n';
        i++;
    }

    return ss.str();
}

std::ostream& operator << (std::ostream& out, const Snapshot& snap)
{
    return out << snap.ToString();
}

Snapshot& Snapshot::operator = (const Snapshot& other)
{
    if(this != &other)
    {
        V = other.V;
        open = other.open;
        maybe_open = other.maybe_open;
        maybe_blocked = other.maybe_blocked;
    }
    return *this;
}

Snapshot& Snapshot::operator = (Snapshot&& other)
{
    if(this != &other)
    {
        V = std::forward<CoordinateSet>(other.V);
        open = std::forward<EdgeSet>(other.open);
        maybe_open = std::forward<EdgeSet>(other.maybe_open);
        maybe_blocked = std::forward<EdgeSet>(other.maybe_blocked);
    }
    return *this;
}