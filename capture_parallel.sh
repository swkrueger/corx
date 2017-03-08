#!/bin/bash

set -e

source settings.sh
DATE=$(date +%Y-%m-%d_%H-%M-%S)
echo "Capture"
parallel -j 4 build/corx_rx ${PARAMS} -o "${CORX_PATH}/${DATE}_rx{}.corx" -d {} '>' "${CORX_PATH}/${DATE}_rx{}.log" '2>&1' ::: 0 1 2 3
echo "Correlate"
parallel --xapply ./correlate.py "${CORX_PATH}/${DATE}_rx{1}.corx" "${CORX_PATH}/${DATE}_rx{2}.corx" -o "${CORRS_PATH}/corr_${DATE}__{1}-{2}.npz" ::: 0 0 0 1 1 2 ::: 1 2 3 2 3 3 >${CORRS_PATH}/corr_${DATE}.log
