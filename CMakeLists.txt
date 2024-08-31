cmake_minimum_required(VERSION 3.22)

set(target "MAPF-IM")
set(executable "MAPF-IM-EXE")
set(compile_flags -pipe -Ofast -g)
set(compile_def _REENTRANT _FORTIFY_SOURCE=2 _GLIBCXX_ASSERTIONS LOG)
add_compile_definitions(LOG)
set(linking_flags -rdynamic)
set(linking_libs )

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

project(${target})

# Build MAPF-IM library
file(GLOB_RECURSE lib_includes "lib-src/*.h")
file(GLOB_RECURSE lib_sources "lib-src/*.cpp")

add_library(${target} ${lib_includes} ${lib_sources})

target_link_libraries(${target} PRIVATE ${linking_libs})
target_compile_options(${target} PRIVATE ${compile_flags})
target_link_options(${target} PRIVATE ${linking_flags})
target_compile_definitions(${target} PRIVATE ${compile_defs})

# Build MAPF-IM executable
file(GLOB_RECURSE exe_includes "exe-src/*.h")
file(GLOB_RECURSE exe_sources "exe-src/*.cpp")
add_executable(${executable} ${exe_sources} ${exe_includes})
target_link_libraries(${executable} PRIVATE ${target})
target_compile_options(${executable} PRIVATE ${compile_flags})
target_link_options(${executable} PRIVATE ${linking_flags})
target_compile_definitions(${executable} PRIVATE ${compile_defs})