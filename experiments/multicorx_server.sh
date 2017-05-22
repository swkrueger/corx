#!/bin/bash

# Example usage:
#  ./multicorx_server.sh --hostid=A

set -e
set -u

source settings.sh
# source noise.sh

mkdir -p ${CORX_PATH}
mkdir -p ${LOG_PATH}

# create RAM drives
sudo umount ${CORX_PATH} || /bin/true
sudo mount -t tmpfs -o size=${RAMDISK_SIZE} tmpfs ${CORX_PATH}

sudo umount ${LOG_PATH} || /bin/true
sudo mount -t tmpfs -o size=100M tmpfs ${LOG_PATH}

# run!
../src/multicorx.py --num=4 --socket --host=0.0.0.0 --allow-exec --log="${LOG_PATH}/multicorx_$(date +%s).log" "$@"
