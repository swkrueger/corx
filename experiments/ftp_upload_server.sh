#!/bin/bash

# set -u

# Note: Upload dir should also be changed in sshd_config
UPLOAD_DIR="/tmp/uploads"
RAMDISK_SIZE="4G"

python -m pyftpdlib -h >/dev/null 2>/dev/null
if [ $? -ne 0 ]; then
    echo "pyftpdlib not installed."
    echo "To install, run 'pip install --user pyftpdlib'"
    exit 1
fi

# mkdir -p /tmp/uploads/{corx,corr,log}
mkdir -p ${UPLOAD_DIR}
if [ -f ${UPLOAD_DIR}/stop ]; then
    rm ${UPLOAD_DIR}/stop
fi

# Create RAM drive
if grep -qs "${UPLOAD_DIR}" /proc/mounts; then
    echo "Skip RAM drive creation: ${UPLOAD_DIR} already mounted"
else
    echo "Create RAM drive of size ${RAMDISK_SIZE} at ${UPLOAD_DIR}"
    sudo mount -t tmpfs -o size=${RAMDISK_SIZE} tmpfs ${UPLOAD_DIR}
fi

# Catch Cntl-C and unmount
trap ' ' INT

# Run SSHD
echo "Run pyftpdlib"
python -m pyftpdlib -p 2121 -w -d "${UPLOAD_DIR}"

# Unmount RAM drive
echo "Unmount RAM drive"
sudo umount ${UPLOAD_DIR}
