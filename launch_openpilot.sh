#!/usr/bin/bash

# Create log directory if it doesn't exist
mkdir -p /data/log

# Redirect stdout and stderr to separate files while also printing to terminal
exec > >(tee /data/log/openpilot_stdout.log) 2> >(tee /data/log/openpilot_stderr.log >&2)

exec ./launch_chffrplus.sh
