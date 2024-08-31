#include "Map.h"
#include "Printer.h"
#include "Snapshot.h"
#include "Terrain.h"
#include "Types.h"
#include "Utils.h"
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <cassert>
#include <boost/algorithm/cxx11/copy_if.hpp>
#include <boost/unordered_map.hpp>
#include <iterator>
#include <sstream>
#include <random>
#include <string>

Map::Map(const std::string& map_file_path, const NeighborhoodFunction& N, size_t R): 
name(ExtractFileName<std::string>(map_file_path)), N(N), R(R), grid(BuildGrid(map_file_path)), coordinates(BuildVerticesSet()), open(), blocked(), maybe_open(), maybe_blocked(), rd(), gen(rd())
{
    InitEdgeSet();
}

GridRow Map::ParseRow(const std::string& map_row) const
{
    GridRow row;
    constexpr unsigned char NUMBER_OF_CARRIAGE_LITERALS = 32;

    for(auto tile: map_row)
    {
        if(tile > NUMBER_OF_CARRIAGE_LITERALS)
        {
            row.push_back(static_cast<Terrain>(tile));
        }
            
    }

    return row;
}

Grid Map::BuildGrid(const std::string& map_file_path) const
{
    std::ifstream file(map_file_path, std::ios::in);
    std::string type,  octile, height, width, map, line;
    std::size_t nrows = -1, ncolumns = -1;
    Grid g;

    if(file.is_open())
    {
        /*
            All maps begin with the lines:
            type octile
            height y
            width x
            map
        */
        file >> type >> octile >> height >> nrows >> width >> ncolumns >> map;

        while(getline(file, line))
        {
            auto&& r = ParseRow(line);
            if(!r.empty())
                g.push_back(r);
        }

        file.close();
    }
    else
    {
        Print(Red, "Illegal map file path. Given: ", map_file_path, "PWD :", std::filesystem::current_path());
        exit(EXIT_FAILURE);
    }

    assert(g.size() == nrows);
    assert(std::any_of(g.begin(), g.end(), [ncolumns](const auto& row){return row.size() == ncolumns;}));
    return g;
}

CoordinateSet Map::BuildVerticesSet(void) const
{
    CoordinateSet V;

    for(size_t i = 0; i < grid.size(); i++)
    {
        for(size_t j = 0; j < grid.front().size(); j++)
        {
            V.emplace(i, j);
        }
    }

    return V;
}

void Map::InitEdgeSet()
{
    for(int i = 0; i < grid.size(); i++)
    {
        for(int j = 0; j < grid.front().size(); j++)
        {
            Coordinate c1{i, j};
            for(const auto& c2: GetSurroundingEnv(c1))
            {
                Edge e{c1, c2};
                if(IsOpenEdge(e))
                    open.insert(e);
                else if(IsBlockedEdge(e))
                    blocked.insert(e);
                else if(IsMaybeOpenEdge(e))
                    maybe_open.insert(e);
                else if(IsMaybeBlockedEdge(e))
                    maybe_blocked.insert(e);
                else
                {
                    Print(Red, "Illegal edge detected. Given: ", e, ". Terrain of e.source = ", grid[e.source.row][e.source.column], ". Terrain of e.destination = ", grid[e.destination.row][e.destination.column], '\n');
                    exit(EXIT_FAILURE);
                }                    
            }
        }
    }
}

CoordinateSet Map::GetSurroundingEnv(const Coordinate& c) const
{
    CoordinateSet filtered;
    for(const auto& c2: N(c, R))
    {
        if(IsValidCoordinate(c2))
            filtered.insert(c2);
    }
    return filtered;
}

bool Map::IsValidCoordinate(const Coordinate& coordinate) const
{
    return coordinate.row >= 0 && coordinate.row < (int)grid.size() && coordinate.column >= 0 && coordinate.column < (int)grid.front().size();
}

bool Map::IsOpenEdge(const Edge& e) const
{
    int x1 = e.source.row, y1 = e.source.column, x2 = e.destination.row, y2 = e.destination.column;
    return  ((grid[x1][y1] == Terrain::empty || grid[x1][y1] == Terrain::maybe_open || grid[x1][y1] == Terrain::maybe_blocked) && grid[x2][y2] == Terrain::empty) ||
            ((grid[x1][y1] == Terrain::maybe_open || grid[x1][y1] == Terrain::maybe_blocked) && (grid[x2][y2] == Terrain::maybe_open || grid[x2][y2] == Terrain::maybe_blocked));
}

bool Map::IsBlockedEdge(const Edge& e) const
{
    int x1 = e.source.row, y1 = e.source.column, x2 = e.destination.row, y2 = e.destination.column;
    return grid[x1][y1] == Terrain::wall || grid[x1][y1] == Terrain::tree || grid[x2][y2] == Terrain::wall || grid[x2][y2] == Terrain::tree;
}

bool Map::IsMaybeOpenEdge(const Edge& e) const
{
    int x1 = e.source.row, y1 = e.source.column, x2 = e.destination.row, y2 = e.destination.column;
    return grid[x1][y1] == Terrain::empty && grid[x2][y2] == Terrain::maybe_open;
}

bool Map::IsMaybeBlockedEdge(const Edge& e) const
{
    int x1 = e.source.row, y1 = e.source.column, x2 = e.destination.row, y2 = e.destination.column;
    return grid[x1][y1] == Terrain::empty && grid[x2][y2] == Terrain::maybe_blocked;
}

std::string Map::PaintTerrain(const Terrain t, const bool paint_path) const
{
    boost::unordered::unordered_map<Terrain, OutputColor> Color = {{Terrain::empty, Default}, {Terrain::agent, Yellow}, {Terrain::goal, Yellow},
                                                                    {Terrain::closed, Red}, {Terrain::open, Green}, 
                                                                    {Terrain::maybe_blocked, Blue}, {Terrain::maybe_open, Purple}};
                                                                    
    OutputColor terrain_color = (paint_path && t == Terrain::empty) ? Yellow : Color[t];

    return "\033[1;" + to_string(terrain_color) + 'm' + static_cast<char>(t) + "\033[0m";
}

std::string Map::GetColumnsSpaces(const int column) const
{
    const auto terrain_length = 1;
    constexpr auto max_tile_length = 4;
    auto number_of_spaces = max_tile_length - terrain_length;
    if(column >= 100)
        number_of_spaces += 1;
    std::string spaces(number_of_spaces, ' ');
    return spaces;
}

std::string Map::GetHeadColumnSpaces(const int column) const
{
    return (column < 10) ? "   " : "  ";
}

std::string Map::GetRowsSpaces(const int row) const
{
    return  (row < 10) ? "   " : (row < 100) ? "  " : " ";
}

std::string Map::GetShape(void) const
{
    return "(" + std::to_string(grid.size()) + ", " + std::to_string(grid.front().size()) + ")";
}

std::string Map::GetEdgesStats(void) const
{
    return  "|Eopen| = " + std::to_string(GetNumberOfOpenEdge()) + 
            ", |Eblocked| = " + std::to_string(GetNumberOfBlockedEdge()) +
            ", |Emaybe_open| = " + std::to_string(GetNumberOfMaybeOpenEdge()) +
            ", |Emaybe_blocked| = " + std::to_string(GetNumberOfMaybeBlockedEdge());
}

void Map::SetColumnNumbers(std::stringstream& ss) const
{
    ss << "    ";
    for(size_t column = 0; column < grid.front().size(); column++)
        ss << column << GetHeadColumnSpaces(column);
    ss << '\n';
}

void Map::SetRowNumber(std::stringstream& ss, const int row) const
{
    ss << row << GetRowsSpaces(row);
}

void Map::SetTerrains(std::stringstream& ss, const int row, const Path& p) const
{
    SetRowNumber(ss, row);

    for(int column = 0; column < grid.front().size(); column++)
    {
        Coordinate current = {row, column};
        Terrain terrain =   (!p.empty() && p.front() == current) ? Terrain::agent :
                            (!p.empty() && p.back() == current) ? Terrain::goal :
                            grid[row][column];

        bool paint_path = !p.empty() && std::find(p.begin(), p.end(), current) != p.end();
        
        ss << PaintTerrain(terrain, paint_path);
        ss << GetColumnsSpaces(column);
    }
    
    ss << '\n';
}

std::string Map::ToString(const Path& p) const
{
    std::stringstream ss;
    SetColumnNumbers(ss);

    for(int row = 0; row < grid.size(); row++)
        SetTerrains(ss, row, p);

    return ss.str();
}

std::ostream& operator << (std::ostream& out, const Map& m)
{
    return out << "Name: " << m.name << '\n' << "Shape: " << m.GetShape() << '\n' << m.GetEdgesStats() << '\n' << m.ToString();
}

Graph Map::CreateGraph(const bool include_maybe_open, const bool include_maybe_blocked) const
{
    EdgeSet e1{open};
    
    if(include_maybe_blocked)
    {
        EdgeSet e2{maybe_blocked};
        e1.merge(e2);
        assert(e2.empty());
    }
    
    if(include_maybe_open)
    {
        EdgeSet e3{maybe_open};
        e1.merge(e3);
        assert(e3.empty());
    }

    return Graph(coordinates, e1);
}

Snapshot Map::CreateSnapshot(const int number_of_uncertain_edges) const
{
    EdgeSet maybe_open_subset(maybe_open.begin(), std::next(maybe_open.begin(), std::min((int)maybe_open.size(), number_of_uncertain_edges)));
    EdgeSet maybe_blocked_subset(maybe_blocked.begin(), std::next(maybe_blocked.begin(), std::min((int)maybe_blocked.size(), number_of_uncertain_edges)));

    assert((int)maybe_open_subset.size() == std::min((int)maybe_open.size(), number_of_uncertain_edges));
    assert((int)maybe_blocked_subset.size() == std::min((int)maybe_blocked.size(), number_of_uncertain_edges));

    // an edge which has not been chosen will be addressed as open (i.e, traversable) edge.
    EdgeSet maybe_open_subset_complementary = GetComplementarySet(maybe_open, maybe_open_subset);
    EdgeSet maybe_blocked_subset_complementary = GetComplementarySet(maybe_blocked, maybe_blocked_subset);

    //
    EdgeSet snapshot_open_edges(open);
    snapshot_open_edges.insert(maybe_open_subset_complementary.begin(), maybe_open_subset_complementary.end());
    snapshot_open_edges.insert(maybe_blocked_subset_complementary.begin(), maybe_blocked_subset_complementary.end());

    assert(snapshot_open_edges.size() == open.size() + maybe_open_subset_complementary.size() + maybe_blocked_subset_complementary.size());

    return Snapshot(coordinates, snapshot_open_edges, maybe_open_subset, maybe_blocked_subset);
}

EdgeSet Map::GetComplementarySet(const EdgeSet& all, const EdgeSet& subset) const
{
    EdgeSet complementary;

    for(const auto& e: all)
    {
        if(subset.find(e) == subset.end())
            complementary.insert(e);
    }

    assert(all.size() == subset.size() + complementary.size()); 

    return complementary; 
}

EdgeSet Map::Sample(const EdgeSet& src, const int nsamples)
{
    EdgeSet samples;
    std::sample(src.begin(), src.end(), std::inserter(samples, samples.begin()), nsamples, gen);
    assert((int)samples.size() == nsamples);
    return samples;            
}

void Map::LogGrid(std::fstream& log, const std::string& log_cell_delimiter, const std::string& log_lines_delimiter) const
{
    log << '#' << name << log_lines_delimiter;
    
    const auto number_of_columns = grid.front().size();

    for(const auto& row: grid)
    {
        
        const auto last_column = number_of_columns - 1;

        for(auto i = 0; i < last_column; i++)
        {
            log << static_cast<char>(row[i]) << log_cell_delimiter;
        }
            
        log << static_cast<char>(row[last_column]) << log_lines_delimiter;
    }

    log << log_lines_delimiter;
}