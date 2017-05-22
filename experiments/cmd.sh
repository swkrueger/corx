#!/bin/bash

set -e
set -u

source settings.sh

../src/multicorx_remote.py --remote="${RECEIVERS}" "$@"
