#pragma once

enum Terrain: char
{
    empty = '.',
    agent = 'A',
    goal = 'G',
    maybe_blocked = '?', // an agent assumes a vertex is NOT-REACHABLE, however actually it is.
    maybe_open = '!', // an agent assumes a vertex is REACHABLE, however actually its not.
    wall = '@',
    swamp = 'S',
    tree = 'T',
    water = 'W',
    open = 'O',
    closed = 'C',
    NTerrains
};
