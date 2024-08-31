#pragma once

#include "Coordinate.h"
#include "Snapshot.h"
#include "Types.h"
class Map;

class Neighborhood
{
public:
    static CoordinateSet FourPrincipleDirection(const Coordinate& c, size_t R=1);
    static CoordinateSet EightPrincipleDirection(const Coordinate& c, size_t R=1);
};

class Heuristic
{
public:
    static float ManhattanDistance(const Coordinate& c1, const Coordinate& c2);
    static float ChebyshevDistance(const Coordinate& c1, const Coordinate& c2);
};

class ObjectiveFunction
{
public:
    static long SumOfCost(const Paths&);
    static long Makespan(const Paths&);
    static long PathLength(const Path&);
};

class Validator
{
public:
    static Agents ValidAgents(const Agents& src, const Map& m);
    static bool IsLegalPlan(const Snapshot& snap, const Paths& ps, const Agents& as, bool verbose=true);
    static Conflicts FindConflicts(const Paths& paths);
    static bool NoConflictsExists(const Paths& ps, bool verbose=true);
    static bool ExistsConflict(const Paths& ps);
    static bool AreAllAgentsReachedTheirGoals(const Agents& as, const Paths& ps, bool verbose=false);
    static bool AreAllPathsContainsOnlyValidTransitions(const Snapshot& snap, const Paths& ps, const Agents&, bool verbose=true);
    static void Free(const Conflicts&);

    static Coordinate GetCoordinateAt(const Path& p, int timestep);
    static Edge GetEdgeAt(const Path& p, int timestep);
    static void AddVertexConflictsAt(Conflicts& cs, int timestep, CoordinateMap& cmap);
    static void AddEdgeConflictsAt(Conflicts& conflicts, int timestep, EdgeMap& emap);
};