#!/bin/bash

source settings.sh

../src/multicorx_remote.py --remote="${RECEIVERS}" "$@"
