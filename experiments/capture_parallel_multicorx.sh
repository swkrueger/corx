#!/bin/bash

set -u
source settings.sh

# Flush named pipe
# dd if=multicorx_inactive.fifo iflag=nonblock of=/dev/null > /dev/null

# Start capture
DATE=$(date +%s)
echo "output ${CORX_PATH}/${DATE}_$1_rx${ODROID_ID}{rxid}.corx" > multicorx_ctrl.fifo
echo "capture" > multicorx_ctrl.fifo

# Wait for line
# read -n 1 < multicorx_inactive.fifo
# Wait for EOF
read < multicorx_inactive.fifo

# Delete old data (for test)
rm ${CORX_PATH}/*.corx
