#include "Graph.h"
#include "Edge.h"
#include "Types.h"
#include <utility>

Graph::Graph(): V(), E(), W(), Adj(){}

Graph::Graph(const CoordinateSet& V, const EdgeSet& E): V(V), E(E), W(), Adj(BuildAdjacencyList(E))
{
    InitializeWeights();
}

Graph::Graph(const CoordinateSet& V, const EdgeSet& E, const EdgeWeightFunction& W): V(V), E(E), W(W), Adj(BuildAdjacencyList(E)){}

Graph::Graph(const Graph& other): V(other.V), E(other.E), W(other.W), Adj(other.Adj){}

Graph::Graph(Graph&& other): V(std::forward<CoordinateSet>(other.V)), E(std::forward<EdgeSet>(other.E)), W(other.W), Adj(std::forward<AdjacencyList>(other.Adj)){}

AdjacencyList Graph::BuildAdjacencyList(const EdgeSet& E) const
{
    AdjacencyList adj;

    for(const auto& e: E)
    {
        if(adj.find(e.source) == adj.end())
            adj[e.source] = {};
        adj[e.source].insert(e.destination);
    }
    
    return adj;
}

void Graph::InitializeWeights(void)
{
    for(const auto& e: E)
        W[e] = EDGE_UNIT_COST_WEIGHT;
}

CoordinateSet Graph::SuccessorsOf(const Coordinate& u) const
{
    CoordinateSet s;

    if(Adj.find(u) != Adj.end())
    {
        for(const auto& v: Adj.at(u))
            s.insert(v);
    }

    return s;
}

float Graph::WeightOf(const Edge& e) const
{
    return (W.find(e) == W.end()) ? INF : W.at(e);
}

void Graph::RemoveEdge(const Edge& e)
{
    E.erase(e);
    Adj[e.source].erase(e.destination);
    W.erase(e);
}

void Graph::AddEdge(const Edge& e)
{
    E.insert(e);
    Adj[e.source].insert(e.destination);
    W[e] = EDGE_UNIT_COST_WEIGHT;
}

void Graph::UpdateEdgeWeight(const Edge& e, const float new_weight)
{
    AddEdge(e);
    W[e] = new_weight;
}

Graph& Graph::operator = (const Graph& other)
{
    if(this != &other)
    {
        V = other.V;
        E = other.E;
        W = other.W;
        Adj = other.Adj;
    }
    return *this;
}

Graph& Graph::operator = (Graph&& other)
{
    if(this != &other)
    {
        V = std::forward<CoordinateSet>(other.V);
        E = std::forward<EdgeSet>(other.E);
        W = std::forward<EdgeWeightFunction>(other.W);
        Adj = std::forward<AdjacencyList>(other.Adj);
    }
    return *this;
}