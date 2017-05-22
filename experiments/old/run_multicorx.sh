#!/bin/bash

set -e
set -u

source settings.sh
source noise.sh

mkdir -p ${CORX_PATH}
mkdir -p ${LOG_PATH}

sudo umount ${CORX_PATH} || /bin/true
sudo mount -t tmpfs -o size=${RAMDISK_SIZE} tmpfs ${CORX_PATH}

sudo umount ${LOG_PATH} || /bin/true
sudo mount -t tmpfs -o size=100M tmpfs ${LOG_PATH}

noise_init

rm -f multicorx_ctrl.fifo
rm -f multicorx_inactive.fifo
mkfifo multicorx_ctrl.fifo
mkfifo multicorx_inactive.fifo

tail -f multicorx_ctrl.fifo | ../src/multicorx.py --inactive=multicorx_inactive.fifo --log=${LOG_PATH}/multicorx_$(date +%s).log
