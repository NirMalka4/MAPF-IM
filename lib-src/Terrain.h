#pragma once

enum Terrain: char
{
    empty = '.', // traversable location
    agent = 'S', // agent start location
    goal = 'G', // agent goal location
    maybe_blocked = '?', // an agent assumes a vertex is NOT-REACHABLE, however actually it is.
    maybe_open = '!', // an agent assumes a vertex is REACHABLE, however actually its not.
    wall = '@', // untraversable location
    tree = 'T',
    water = 'W',
    open = 'O',
    closed = 'C',
    NTerrains
};
