#!/bin/bash

# Check if there is exactly one argument provided
if [ $# -eq 1 ]; then
    # Run the Python script with the provided argument
    python aline_odom_pcd.py $1
    python rename.py $1
    # python evo_to_json.py "${1}/odom.txt" "${1}/pose.json"

else
    echo "Error: Please provide exactly one argument."
    echo "Usage: ./run_test.sh xxx"
fi