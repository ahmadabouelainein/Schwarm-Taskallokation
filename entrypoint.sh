#!/usr/bin/env bash
set -e

# ensure output directory exists
mkdir -p /ws/output

# if no arguments passed, drop into a bash shell
if [ $# -eq 0 ]; then
  exec bash
else
  # otherwise, run the given command
  exec "$@"
fi
