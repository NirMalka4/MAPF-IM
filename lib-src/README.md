```markdown
# MAPF-IM Execution Script

This repository contains a script designed to manage the execution of the MAPF-IM library and executable. The script handles various tasks, including locating necessary files, building the executable if it doesn't exist, and running the executable with specified parameters.

## Features

- **Automatic File Search**: The script searches for required files such as the map file and scenario file within the current directory and its subdirectories.
- **Automatic Directory Creation**: If the specified output directory doesn't exist, the script automatically creates it.
- **Automatic Build Process**: If the `MAPF-IM-EXE` executable is not found, the script will attempt to build it using `build.sh`.
- **Error Handling**: The script provides clear error messages if any step fails, including detailed output from the build process if it encounters an error.
- **Customizable Execution**: The script allows users to specify parameters such as the number of agents, the framework name, and the policy name, with sensible default values provided.

## Usage

To run the script, use the following command:

```bash
./script_name.sh [options]
```

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

### Example

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

### Explanation:
- **Available Options**: This section lists all the options available in the script, including their short and long forms, descriptions, and default values.
- **Example**: This provides a practical example of how to run the script with various options.
- **License**: Standard section providing information about the project's licensing.

This `README.md` file is now comprehensive, making it easy for users to understand the script's capabilities, how to use it, and how to contribute.