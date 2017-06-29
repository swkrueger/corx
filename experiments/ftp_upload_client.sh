#!/bin/bash

shopt -s nullglob

ftp-upload --version >/dev/null 2>/dev/null
if [ $? -ne 0 ]; then
    echo "ftp-upload not installed."
    echo "To install, run 'sudo apt install ftp-upload'"
    exit 1
fi

server="$1"
shift 1

files=($@)

ftp-upload -h $server "${files[@]}"
