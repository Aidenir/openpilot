#!/usr/bin/bash

# Create log directory if it doesn't exist
mkdir -p /data/log

# Find the highest numbered log file and increment by 1
LAST_NUM=$(ls /data/log/openpilot_*.log 2>/dev/null | sed 's/.*openpilot_\([0-9]*\)\.log/\1/' | sort -n | tail -1)
NEXT_NUM=$((${LAST_NUM:-0} + 1))

# Run launch_chffrplus.sh with combined stdout/stderr logged to file
./launch_chffrplus.sh 2>&1 | tee "/data/log/openpilot_${NEXT_NUM}.log"
