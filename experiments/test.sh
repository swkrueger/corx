#!/bin/bash

set -e

source settings.sh
../build/corx_rx ${PARAMS} --wisdom="$HOME/corx.wisdom" "$@"
