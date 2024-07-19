cmake_minimum_required(VERSION 3.22)

set(target "MAPF-IM")
set(compile_flags -pipe -Wall -Wextra -Weffc++ -Ofast -g)
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

set(exp_files Experiment/Experiment.cpp)
set(scenario_log_genration_files ScenarioLogGeneration/ScenarioLogGeneration.cpp)
set(lifelongsipp_files Experiment/LifelongSIPPTest.cpp)
set(low_level_test_files Experiment/LowLevelTest.cpp)

file(GLOB_RECURSE includes "*.h")
file(GLOB_RECURSE sources "*.cpp")

message(STATUS ${sources})
list(REMOVE_ITEM sources ${CMAKE_CURRENT_SOURCE_DIR}/${exp_files})
list(REMOVE_ITEM sources ${CMAKE_CURRENT_SOURCE_DIR}/${lifelongsipp_files})
list(REMOVE_ITEM sources ${CMAKE_CURRENT_SOURCE_DIR}/${low_level_test_files})
message(STATUS ${sources})

add_library(${target} ${includes} ${sources})

target_link_libraries(${target} PRIVATE ${linking_libs})
target_compile_options(${target} PRIVATE ${compile_flags})
target_link_options(${target} PRIVATE ${linking_flags})
target_compile_definitions(${target} PRIVATE ${compile_defs})

add_executable(Experiment ${exp_files})
target_link_libraries(Experiment PRIVATE ${target})

add_executable(ScenarioLogGeneration ${scenario_log_genration_files})
target_link_libraries(ScenarioLogGeneration PRIVATE ${target})

add_executable(LifelongSIPPTest ${lifelongsipp_files})
target_link_libraries(LifelongSIPPTest PRIVATE ${target})

add_executable(LowLevelTest ${low_level_test_files})
target_link_libraries(LowLevelTest PRIVATE ${target})