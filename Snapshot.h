#pragma once

#include "Graph.h"
#include "Types.h"

class Snapshot
{
public:
    Snapshot(const CoordinateSet& V, const EdgeSet& open, const EdgeSet& maybe_open, const EdgeSet& maybe_blocked);
    Snapshot(const Snapshot& other);
    Snapshot(Snapshot&& other);
    virtual ~Snapshot() = default;

    Graph Create(void) const;
    Graph Create(bool include_maybe_open, bool include_maybe_blocked) const;

    bool IsMaybeOpenEdge(const Edge& e) const;
    bool IsOpenEdge(const Edge& e) const;
    bool IsMaybeBlockedEdge(const Edge& e) const;

    inline int GetNumberOfMaybeOpenEdge(void) const {return maybe_open.size();}
    inline int GetNumberOfMaybeBlockedEdge(void) const {return maybe_blocked.size();}
    inline int GetNumberOfOpenEdge(void) const {return open.size();}
    inline const EdgeSet& GetMaybeOpenEdges(void) const {return maybe_open;}
    inline const EdgeSet& GetMaybeBlockedEdges(void) const {return maybe_blocked;}

    std::string ToString(void) const;
    friend std::ostream& operator << (std::ostream&, const Snapshot&);
    Snapshot& operator = (const Snapshot& other);
    Snapshot& operator = (Snapshot&& other);

protected:
    CoordinateSet V;
    EdgeSet open, maybe_open, maybe_blocked;

    std::string EdgeSetToString(const EdgeSet& s, const std::string& s_name) const;
};