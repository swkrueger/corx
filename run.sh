#!/bin/bash

set -e

source settings.sh

mkdir -p ${CORRS_PATH}
mkdir -p ${CORX_PATH}
sudo mount -t tmpfs -o size=1024M tmpfs ${CORX_PATH}

# sudo ifdown eth0
./capture_parallel.sh
# sudo ifup eth0
