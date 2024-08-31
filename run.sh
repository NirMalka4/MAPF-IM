#!/bin/bash

# Function to display usage information
usage() {
    echo "Usage: $0 [options]"
    echo
    echo "Options:"
    echo "  -m,  --map_file_path <map_file_path>                        Path to the map file (default: room-64-64-8.map)"
    echo "                                                              The script will search for this file in any subdirectory of the current working directory."
    echo
    echo "  -s,  --scenario_file_path <scenario_file_path>              Path to the scenario file (default: room-64-64-8-random-1.scen)"
    echo "                                                              The script will search for this file in any subdirectory of the current working directory."
    echo
    echo "  -o,  --output_directory_path <output_directory_path>        Path to the output directory (default: mapf-im-output)"
    echo "                                                              The script will create this directory if it does not exist in any subdirectory of the current working directory."
    echo
    echo "  -k  <number_of_agents>                                      Number of agents (default: 10)"
    echo
    echo "  -f  <framework_name>                                        Framework name (default: full_id_planner)"
    echo "                                                              Options: full_planner, full_id_planner, local_planner, local_id_planner"
    echo
    echo "  -hl, --high_level_planner_name <high_level_planner_name>    High-level planner name (default: cbs)"
    echo "                                                              Options: pp, cbs"
    echo
    echo "  -ll, --low_level_planner_name <low_level_planner_name>      Low-level planner name (default: sipp)"
    echo "                                                              Options: sipp, ees_sipp"
    echo
    echo "  -p  <policy_name>                                           Policy name (default: baseline)"
    echo "                                                              Options: risk_averse, explorative, hybrid, baseline"
    echo
    echo "  -t  <timeout>                                               Max runtime measured in seconds (default: 300)"
    echo
    echo "  -vp, --visualize_path <visualize_path>                      Whether to visualize the final path an agent traversed (default: 1)"
    echo "                                                              Options: 1 (true), 0 (false)"
    echo
    echo "  -pp, --print_path <print_path>                              Whether to print the vertex an agent traversed in each timestep (default: 1)"
    echo "                                                              Options: 1 (true), 0 (false)"
    echo
    echo "  -h,  --help                                                 Show this help message and exit"
    exit 0
}

# Set default values
map_file_path="maze-32-32-2.map"
scenario_file_path="maze-32-32-2-random-1.scen"
output_directory_path="mapf-im-output"
number_of_agents=10
framework_name="full_id_planner"
high_level_planner_name="cbs"
low_level_planner_name="sipp"
policy_name="baseline"
timeout=300
visualize_path=1
print_path=1

# Parse the command-line arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -m|--map_file_path) map_file_path="$2"; shift ;;
        -s|--scenario_file_path) scenario_file_path="$2"; shift ;;
        -o|--output_directory_path) output_directory_path="$2"; shift ;;
        -k) number_of_agents="$2"; shift ;;
        -f) framework_name="$2"; shift ;;
        -hl|--high_level_planner_name) high_level_planner_name="$2"; shift ;;
        -ll|--low_level_planner_name) low_level_planner_name="$2"; shift ;;
        -p) policy_name="$2"; shift ;;
        -t) timeout="$2"; shift ;;
        -vp|--visualize_path) visualize_path="$2"; shift ;;
        -pp|--print_path) print_path="$2"; shift ;;
        -h|--help) usage ;;
        *) echo "Unknown parameter passed: $1"; usage ;;
    esac
    shift
done

# Function to find a file in any subdirectory
find_file() {
    local file_name="$1"
    local found_file=$(find "$PWD" -type f -name "$file_name" 2>/dev/null | head -n 1)
    if [ -z "$found_file" ]; then
        echo ""
    else
        echo "$found_file"
    fi
}

# Function to find or create a directory in any subdirectory
find_or_create_dir() {
    local dir_name="$1"
    local found_dir=$(find "$PWD" -type d -name "$dir_name" 2>/dev/null | head -n 1)
    if [ -z "$found_dir" ]; then
        echo "Directory '$dir_name' not found. Creating it in the current directory..."
        mkdir -p "$PWD/$dir_name"
        found_dir="$PWD/$dir_name"
    fi
    echo "$found_dir"
}

# Locate the map file
map_file_path=$(find_file "$map_file_path")
if [ -z "$map_file_path" ]; then
    echo "Error: Map file not found in any subdirectory of the current directory."
    exit 1
fi

# Locate the scenario file
scenario_file_path=$(find_file "$scenario_file_path")
if [ -z "$scenario_file_path" ]; then
    echo "Error: Scenario file not found in any subdirectory of the current directory."
    exit 1
fi

# Locate or create the output directory
output_directory_path=$(find_or_create_dir "$output_directory_path")

# Locate the MAPF-IM-EXE executable
executable=$(find_file "MAPF-IM-EXE")

# If the executable is not found, attempt to build it
if [ -z "$executable" ]; then
    echo "MAPF-IM-EXE executable not found."
    if [ -f "./build.sh" ]; then
        echo "Building MAPF-IM library and executable..."
        if ! ./build.sh 2>&1 | tee build_output.log; then
            echo "Error: Failed to build MAPF-IM library and executable."
            echo "Build script output:"
            cat build_output.log
            exit 1
        fi
        executable=$(find_file "MAPF-IM-EXE")
        if [ -z "$executable" ]; then
            echo "Error: MAPF-IM-EXE executable still not found after building. Exiting."
            exit 1
        fi
    else
        echo "Error: build.sh script not found. Cannot build the executable. Exiting."
        exit 1
    fi
fi

# Run the executable with the provided parameters and timeout
$executable "$map_file_path" "$scenario_file_path" "$output_directory_path" "$number_of_agents" "$framework_name" "$high_level_planner_name" "$low_level_planner_name" "$policy_name" "$timeout" "$visualize_path" "$print_path"
