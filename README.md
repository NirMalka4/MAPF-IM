# MAPF-IM: Online Planning for Multi Agent Path Finding in Inaccurate Maps
In MAPF-IM the input graph may be inaccurate, containing non-existent edges or missing edges present in the environment. Agents can verify the existence or non-existence of an edge only by moving close to it. To navigate such maps, we propose an online approach where planning and execution are interleaved. As agents gather new information about the environment over time, they replan accordingly. 
The proposed algorithm were presented at IROS-24 conference and has many practical applications, especially in the field of autonomous robotics.


## Content

This repository includes a script that helps manage the execution of the MAPF-IM library and executable. The script takes care of finding necessary files, building the executable if needed, and running it with the specified settings.

The content related to the MAPF-IM algorithms is organized into the following folders:

- **[benchmarks](./benchmarks/):** Contains input files (maps and scenarios) required by the MAPF-IM algorithms. These files were used in the experiments described in the paper.
  
- **[exe-src](./exe-src/):** Includes the main source file used to run the MAPF-IM algorithms.

- **[lib-src](./lib-src/):** Contains the source code for MAPF-IM, including the frameworks, high/low level planners, and policies proposed in the paper.

- **[mapf-im-output](./mapf-im-output/):** Holds log files from the MAPF-IM algorithm executions, detailing the actions of each agent at each timestep.


### Available Options

- `-m, --map_file_path <map_file_path>`: Path to the map file (default: `room-64-64-8.map`). The script will search for this file in any subdirectory of the current working directory.
- `-s, --scenario_file_path <scenario_file_path>`: Path to the scenario file (default: `room-64-64-8-random-1.scen`). The script will search for this file in any subdirectory of the current working directory.
- `-o, --output_file_path <output_file_path>`: Path to the output directory (default: `mapf-im-output`). If the directory does not exist, the script will create it.
- `-k <number_of_agents>`: Number of agents (default: 10).
- `-f <framework_name>`: Framework name (default: `full_id_planner`). Options: `full_planner`, `full_id_planner`, `local_planner`, `local_id_planner`.
- `-hl, --high_level_planner_name <high_level_planner_name>`: High-level planner name (default: `cbs`). Options: `pp`, `cbs`.
- `-ll, --low_level_planner_name <low_level_planner_name>`: Low-level planner name (default: `sipp`). Options: `sipp`, `ees_sipp`.
- `-p <policy_name>`: Policy name (default: `baseline`). Options: `risk_averse`, `explorative`, `hybrid`, `baseline`.
- `-t <timeout>`: Max runtime measured in seconds (default: 300).
- `-vp, --visualize_path <visualize_path>`: Whether to visualize the final path an agent traversed (default: 1). Options: `1` (true), `0` (false).
- `-pp, --print_path <print_path>`: Whether to print the vertex an agent traversed in each timestep (default: 1). Options: `1` (true), `0` (false).
- `-h, --help`: Display help message and exit.

## Usage example

```bash
./script_name.sh -m "custom_map.map" -s "custom_scenario.scen" -o "output_dir" -k 20 -f "full_id_planner" -hl "cbs" -ll "sipp" -p "risk_averse" -t 500 -vp 0 -pp 1
```

### Build Process

If the `MAPF-IM-EXE` executable is not found, the script will automatically attempt to build it by running `build.sh`. If the build process fails, the script will print detailed error messages to help you diagnose the issue.

## Contributing

Feel free to submit issues or pull requests if you have suggestions for improvements or find any bugs.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
```