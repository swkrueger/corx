#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
chmod 600 "$DIR/id_rsa"

s="$1"
shift 1

cmd="progress\n"
for f in "$@"; do
    cmd="$cmd\nput $f"
done

printf "$cmd" | sftp -p -P 2222 -b- -i "$DIR/id_rsa" "$s" || exit 1
