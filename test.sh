#!/bin/bash

set -e

source settings.sh
rtl_biast -b 1 "$@"
./corx_rx ${PARAMS} "$@"
