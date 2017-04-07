#!/bin/bash

# set -e
set -u

source settings.sh
DATE=$(date +%Y-%m-%d_%H-%M-%S)
sudo ifdown eth0
echo "Capture"
parallel -j 4 ../build/corx_rx ${PARAMS} --output="${CORX_PATH}/${DATE}_$1_rx${ODROID_ID}{}.corx" --device_index={} '>' "${LOG_PATH}/${DATE}_$1_rx${ODROID_ID}{}.log" '2>&1' ::: 0 1 2 3
sudo ifup eth0
echo "Correlate"
parallel --xapply ../src/correlate.py "${CORX_PATH}/${DATE}_$1_rx${ODROID_ID}{1}.corx" "${CORX_PATH}/${DATE}_$1_rx${ODROID_ID}{2}.corx" -o "${CORRS_PATH}/corr_${DATE}_$1_{1}-{2}.npz" ::: 0 0 0 1 1 2 ::: 1 2 3 2 3 3 >${LOG_PATH}/corr_${DATE}_$1.log
rm ${CORX_PATH}/${DATE}_rx*.corx
upload_client/upload.sh ${UPLOAD_SERVER} "${CORRS_PATH}/corr_${DATE}_$1_*.npz" ${LOG_PATH}/corr_${DATE}_$1.log
upload_client/check_stop.sh ${UPLOAD_SERVER}
if [ $? -ne 0 ]; then
    exit 1
fi
exit 0
