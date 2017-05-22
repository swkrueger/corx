#!/bin/bash

source settings.sh

./puppeteer.py \
    --remote="${RECEIVERS}" \
    --runs="${NUM_RUNS}" \
    --corx-path="${CORX_PATH}" \
    --upload-server="${UPLOAD_SERVER}"
