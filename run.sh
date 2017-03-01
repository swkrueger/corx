#!/bin/bash

set -e

source settings.sh

mkdir -p ${OUTPUT_PATH}
sudo mount -t tmpfs -o size=1024M tmpfs ${OUTPUT_PATH}

rtl_biast -d 0 -b 1
rtl_biast -d 1 -b 1
rtl_biast -d 2 -b 1
rtl_biast -d 3 -b 1

# sudo ifdown eth0
./capture_parallel.sh
# sudo ifup eth0
