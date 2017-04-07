#!/bin/bash
#
# Requires inotifywait. On Debian / Ubuntu:
#     sudo apt install inotify-tools
#

set -u

CORX_DIR="/tmp/uploads/"

if [ ! -d "${CORX_DIR}" ]; then
    echo "${CORX_DIR} does not exist or isn't a directory"
fi

inotifywait --quiet --monitor --event=CLOSE_WRITE --event=MOVED_TO --format="%f" "${CORX_DIR}" |
    ./correlate_server.py
    # while read line; do echo "Received \"$line\""; done
