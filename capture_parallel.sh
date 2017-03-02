#!/bin/bash

set -e

source settings.sh
DATE=$(date +%Y-%m-%d_%H-%M-%S)
parallel --output-as-files build/corx_rx ${PARAMS} -o ${OUTPUT_PATH}/${DATE}_rx{}.corx -d {} ::: 0 1 2 3
# Alternatives: --tmux or --line-buffer --tagstring {}
