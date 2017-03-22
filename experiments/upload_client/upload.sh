#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

s="$1"
shift 1

cmd="progress\n"
for f in "$@"; do
    cmd="$cmd\nput $f"
done

printf "$cmd" | sftp -P 2222 -b- -i "$DIR/id_rsa" "$s" || exit 1
