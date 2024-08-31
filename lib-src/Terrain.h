#pragma once

enum Terrain: char
{
    empty = '.',
    agent = 'S',
    goal = 'G',
    maybe_blocked = '?', // an agent assumes a vertex is NOT-REACHABLE, however actually it is.
    maybe_open = '!', // an agent assumes a vertex is REACHABLE, however actually its not.
    wall = '@',
    tree = 'T',
    water = 'W',
    open = 'O',
    closed = 'C',
    NTerrains
};
