#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "get stop" | sftp -P 2222 -b- -i "$DIR/id_rsa" "$1" >/dev/null 2>/dev/null || exit 0
echo "Received stop command"
exit 1
