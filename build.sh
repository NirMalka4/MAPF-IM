#!/bin/bash

# Create the build directory
mkdir -p build

# Navigate to the build directory
cd build

# Run CMake to configure the project
cmake ..

# Compile the project
make