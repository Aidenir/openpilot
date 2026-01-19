#!/usr/bin/bash

# Create log directory if it doesn't exist
mkdir -p /data/log

# Run launch_chffrplus.sh with combined stdout/stderr logged to file
./launch_chffrplus.sh 2>&1 | tee "/data/log/openpilot_$(date +%Y-%m-%d_%H-%M-%S).log"
