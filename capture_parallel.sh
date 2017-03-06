#!/bin/bash

set -e

source settings.sh
DATE=$(date +%Y-%m-%d_%H-%M-%S)
parallel build/corx_rx ${PARAMS} -o "${OUTPUT_PATH}/${DATE}_rx{}.corx" -d {} '>' "${OUTPUT_PATH}/${DATE}_rx{}.log" '2>&1' ::: 0 1 2 3
# Alternatives: --output-as-files or --tmux or --line-buffer --tagstring {}
