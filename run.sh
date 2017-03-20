#!/bin/bash

# set -e

source settings.sh
source noise.sh

mkdir -p ${CORRS_PATH}
mkdir -p ${CORX_PATH}
mkdir -p ${LOG_PATH}

sudo umount ${CORX_PATH} || /bin/true
sudo mount -t tmpfs -o size=${RAMDISK_SIZE} tmpfs ${CORX_PATH}

noise_init

# while :; do
    for i in `seq 1 ${NUM_NOISE_GENS}`; do
        for j in `seq 1 ${CAPTURE_NOISE_OFF_REPEAT}`; do
            time ./capture_parallel.sh N0
        done
        noise_on $i
        time ./capture_parallel.sh N$i
        noise_off $i
    done
# done
