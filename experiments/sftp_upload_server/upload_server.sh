#!/bin/bash

# set -u

# Note: Upload dir should also be changed in sshd_config
UPLOAD_DIR="/tmp/uploads"
RAMDISK_SIZE="4G"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

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

# Fix permissions
chmod 0600 ssh_host_rsa_key
chmod 0600 ssh_host_dsa_key

# Catch Cntl-C and unmount
trap ' ' INT

# Run SSHD
echo "Run sshd"
/usr/sbin/sshd -D -f sshd_config -h "$DIR/ssh_host_dsa_key" -h "$DIR/ssh_host_rsa_key" -o "AuthorizedKeysFile $DIR/authorized_keys"

# Unmount RAM drive
echo "Unmount RAM drive"
sudo umount ${UPLOAD_DIR}
