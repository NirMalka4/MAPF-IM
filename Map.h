#pragma once

#include "Coordinate.h"
#include "Types.h"
#include "Terrain.h"
#include "Graph.h"
#include "Snapshot.h"
#include "Utils.h"
#include <string>
#include <random>

class Map
{
public:
    Map(const std::string& map_file_path, const NeighborhoodFunction& N = Neighborhood::FourPrincipleDirection, size_t R=1);
    virtual ~Map() = default;

    // E={open, maybe_blocked, maybe_open}
    Graph CreateGraph(bool include_maybe_open, bool include_maybe_blocked) const;
    Snapshot CreateSnapshot(int number_of_uncertain_edges) const;

    bool IsOpenEdge(const Edge& e) const;
    bool IsBlockedEdge(const Edge& e) const;
    bool IsMaybeOpenEdge(const Edge& e) const;
    bool IsMaybeBlockedEdge(const Edge& e) const;
    bool IsValidCoordinate(const Coordinate& c) const;

    inline int GetNumberOfOpenEdge(void) const {return open.size();}
    inline int GetNumberOfBlockedEdge(void) const {return blocked.size();}
    inline int GetNumberOfMaybeOpenEdge(void) const {return maybe_open.size();}
    inline int GetNumberOfMaybeBlockedEdge(void) const {return maybe_blocked.size();}
    inline int GetNumberOfUncertiandEdge(void) const {return GetNumberOfMaybeOpenEdge() + GetNumberOfMaybeBlockedEdge();}
    inline Terrain GetTerrain(const Coordinate& c) const {assert(IsValidCoordinate(c)); return grid[c.row][c.column];}
    inline const std::string& GetName(void) const {return name;}
    
    void LogGrid(std::fstream& log, const std::string& log_cell_delimiter, const std::string& log_lines_delimiter) const;

    std::string ToString(const Path& p = {}) const;
    friend std::ostream& operator << (std::ostream& out, const Map& m);

private:
    std::string name;
    NeighborhoodFunction N;
    size_t R;
    Grid grid;
    CoordinateSet coordinates;
    EdgeSet open, blocked, maybe_open, maybe_blocked;
    std::random_device rd;
    std::mt19937 gen;

    Grid BuildGrid(const std::string& map_file_path) const;
    GridRow ParseRow(const std::string& row) const;
    CoordinateSet BuildVerticesSet(void) const;
    CoordinateSet GetSurroundingEnv(const Coordinate& c) const;
    void InitEdgeSet(void);
    
    std::string PaintTerrain(const Terrain t, bool paint_path=false) const;
    std::string GetColumnsSpaces(int column) const;
    std::string GetHeadColumnSpaces(int column) const;
    std::string GetRowsSpaces(int row) const;
    std::string GetShape(void) const;
    std::string GetEdgesStats(void) const;
    void SetColumnNumbers(std::stringstream& ss) const;
    void SetRowNumber(std::stringstream& ss, int row) const;
    void SetTerrains(std::stringstream& ss, const int row, const Path& p) const;

    EdgeSet GetComplementarySet(const EdgeSet& all, const EdgeSet& subset) const;
    EdgeSet Sample(const EdgeSet& s, int nsamples);
};