#!/bin/bash
#
# Requires inotifywait. On Debian / Ubuntu:
#     sudo apt install inotify-tools
#

set -u

CORX_DIR="/tmp/uploads"
CORR_DIR="/tmp/corr"

if [ ! -d "${CORX_DIR}" ]; then
    echo "${CORX_DIR} does not exist or isn't a directory"
fi

if [ ! -d "${CORR_DIR}" ]; then
    mkdir -p "${CORR_DIR}"
    echo "Created ${CORR_DIR}"
fi

echo "Waiting..."
inotifywait --quiet --monitor --event=CLOSE_WRITE --event=MOVED_TO --format="%f" "${CORX_DIR}/" |
    ./correlate_server.py
    # while read line; do echo "Received \"$line\""; done
