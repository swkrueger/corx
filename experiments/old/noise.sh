#!/bin/bash

if [ -z ${NOISE_GPIO} ]; then
    echo "NOISE_GPIO env variable should be set"
    exit -1
fi

SYS_GPIO=/sys/class/gpio
NUM_NOISE_GENS=${#NOISE_GPIO[@]}

function noise_on {
    echo 0 > $SYS_GPIO/gpio${NOISE_GPIO[$1]}/value
}

function noise_off {
    echo 1 > $SYS_GPIO/gpio${NOISE_GPIO[$1]}/value
}

function noise_init {
    for gpio in "${NOISE_GPIO[@]}"; do
        PIN=$SYS_GPIO/gpio$gpio
        if [ ! -d $PIN ]; then
            sudo sh -c "echo $gpio > $SYS_GPIO/export"
        fi
        sudo sh -c "echo out > $PIN/direction"
        sudo sh -c "echo 1 > $PIN/value"
        sudo sh -c "chown odroid:odroid $PIN/value"
    done
}
