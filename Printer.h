#pragma once

#include <sstream>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <sstream>

using std::cout;
using std::endl;
using std::to_string;

typedef enum OutputColor
{
    Default = 0,
    Black = 90,
    Red = 91,
    Green = 92,
    Yellow = 93,
    Blue = 94,
    Purple = 95,
    Cyan = 96,
    White = 97,
}OutputColor;

template<typename First>
static inline void Concat(auto& ss, First&& first)
{
    ss << first;
}

template<typename First, typename ...Rest>
static inline void Concat(auto& ss, First&& first, Rest&& ...rest)
{
    ss << first;
    Concat(ss, rest...);
}

template<typename ...T>
static inline std::string BuildMsg(T ...ts)
{
    std::stringstream ss;
    Concat(ss, ts...);
    return ss.str();
}

template<typename First, typename ...Rest>
static inline void Print(OutputColor const color, First&& first, Rest&& ...rest)
{
    cout << "\033[1;" << to_string(color) << 'm' << first << "\033[0m";
    Print(color, rest...);
}

template<typename First>
static inline void Print(OutputColor const color, First&& msg)
{
    cout << "\033[1;" << to_string(color) << 'm' << msg << "\033[0m";
}

template<typename Iterable>
static inline void Print(const Iterable& i)
{
    unsigned j = 1;
    for(const auto& e: i)
        Print(Default, '#', j++, ")\t", e, '\n');
}

template<class T>
static inline T ExtractBaseName(T const & path, T const & delims = "/\\")
{
    return path.substr(path.find_last_of(delims) + 1);
}

template<class T>
static inline T RemoveExtensions(T const & filename)
{
    typename T::size_type const p(filename.find_last_of('.'));
    return p > 0 && p != T::npos ? filename.substr(0, p) : filename;
}

template<class T>
static inline T ExtractFileName(T const & path)
{
    return RemoveExtensions(ExtractBaseName(path));
}