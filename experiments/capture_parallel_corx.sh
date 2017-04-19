#!/bin/bash

# set -e
set -u

source settings.sh
DATE=$(date +%Y-%m-%d_%H-%M-%S)
# sudo ifdown eth0
echo "Capture"
parallel -j 4 ../build/corx_rx ${PARAMS} --wisdom="$HOME/corx$1.wisdom" --output="${CORX_PATH}/${DATE}_$1_rx${ODROID_ID}{}.corx" --device_index={} '>' "${LOG_PATH}/${DATE}_$1_rx${ODROID_ID}{}.log" '2>&1' ::: 0 1 2 3
# sudo ifup eth0
upload_client/upload.sh ${UPLOAD_SERVER} "${CORX_PATH}/${DATE}_$1_rx*.corx"
# upload_client/check_stop.sh ${UPLOAD_SERVER}
rm ${CORX_PATH}/${DATE}_$1_rx*.corx
if [ $? -ne 0 ]; then
    exit 1
fi
exit 0
