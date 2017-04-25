#!/bin/bash

set -e
set -u

source settings.sh
source noise.sh

echo "standby" > multicorx_ctrl.fifo

#while :; do
    for i in `seq 0 $((${NUM_NOISE_GENS}-1))`; do
        for j in `seq 1 ${CAPTURE_NOISE_OFF_REPEAT}`; do
            time ./capture_parallel_multicorx.sh NN
        done
        echo "Noise #$i on"
        noise_on $i
        time ./capture_parallel_multicorx.sh N$i
        echo "Noise #$i off"
        noise_off $i
    done
# done

# echo "exit" > multicorx_ctrl.fifo
