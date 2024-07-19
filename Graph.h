#pragma once

#include "Edge.h"
#include "Types.h"

class Graph
{
public:
    Graph();
    Graph(const CoordinateSet& V, const EdgeSet& E);
    Graph(const CoordinateSet& V, const EdgeSet& E, const EdgeWeightFunction& W);
    Graph(const Graph& other);
    Graph(Graph&& other);
    virtual ~Graph() = default;

    CoordinateSet SuccessorsOf(const Coordinate& u) const;
    float WeightOf(const Edge& e) const;
    void RemoveEdge(const Edge& e);
    void AddEdge(const Edge& e);
    void UpdateEdgeWeight(const Edge& e, float new_weight);
    inline const EdgeSet& GetEdges(void) const {return E;}
    inline const CoordinateSet& GetVertices(void) const {return V;}

    bool operator == (const Graph& other) const noexcept{return V == other.V && E == other.E && W == other.W && Adj == other.Adj;}
    Graph& operator = (const Graph& other);
    Graph& operator = (Graph&& other);

protected:
    CoordinateSet V;
    EdgeSet E;
    EdgeWeightFunction W;
    AdjacencyList Adj;
    
    AdjacencyList BuildAdjacencyList(const EdgeSet& E) const;
    void InitializeWeights(void);
};