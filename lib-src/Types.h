#pragma once

#include <boost/unordered/unordered_set.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <tuple>
#include <vector>
#include <set>
#include "Agent.h"
#include "Coordinate.h"
#include "Edge.h"
#include "Terrain.h"
#include "TimeInterval.h"
#include "IConflict.h"
#include "Constraint.h"

using CoordinateSet = boost::unordered_set<Coordinate, Coordinate::Hasher, Coordinate::Equal>;
using CoordinateMap = boost::unordered_map<Coordinate, std::vector<int>, Coordinate::Hasher, Coordinate::Equal>;
using ConflictsMap = boost::unordered_map<Coordinate, int, Coordinate::Hasher, Coordinate::Equal>;
using EdgeToPenalty = boost::unordered_map<Edge, float, Edge::Hasher, Edge::Equal>;
using EdgeSet = boost::unordered::unordered_set<Edge, Edge::Hasher, Edge::Equal>;
using EdgeSet = boost::unordered_set<Edge, Edge::Hasher, Edge::Equal>;
using EdgeMap = boost::unordered_map<Edge, std::tuple<int, bool>, Edge::Hasher, Edge::Equal>;
using GridRow = std::vector<Terrain>;
using Grid = std::vector<GridRow>;
using Path = std::vector<Coordinate>;
using Paths = std::vector<Path>;
using NeighborhoodFunction = CoordinateSet(*)(const Coordinate& c, size_t R);
using HeuristicFunction = float(*)(const Coordinate& c1, const Coordinate& c2);
using AdjacencyList = boost::unordered::unordered_map<Coordinate, CoordinateSet, Coordinate::Hasher, Coordinate::Equal>;
using EdgeWeights = boost::unordered::unordered_map<Edge, float, Edge::Hasher, Edge::Equal>;
using Intervals = std::set<TimeInterval, TimeInterval::Comparator>;
using Configurations = boost::unordered::unordered_map<Coordinate, Intervals, Coordinate::Hasher, Coordinate::Equal>;
using Agents = std::vector<Agent>;
using AgentsIndicesSet = boost::unordered::unordered_set<int>;
using PlanResult = std::tuple<bool, Paths, unsigned long, float>;
using ScenarioResult = std::tuple<bool, Paths, float, int, unsigned long>;
using Conflicts = std::vector<IConflict*>;
using EdgeWeightUpdate = std::tuple<Edge, float>;
using EdgeWeightsUpdates = std::vector<EdgeWeightUpdate>;
using Constraints = boost::unordered::unordered_set<Constraint, Constraint::Hasher, Constraint::Equal>;
using EdgeWeightFunction = boost::unordered::unordered_map<Edge, float, Edge::Hasher, Edge::Equal>;
using Edges = std::vector<Edge>;