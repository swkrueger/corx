#!/usr/bin/env python

from __future__ import print_function

import fcntl
import os
import re
import select
import subprocess
import sys
import time
from datetime import datetime
from collections import deque

# Constants
CORX_DIR = "/tmp/uploads/"
CORR_DIR = "/tmp/corr/"
LOG_DIR = "/tmp/corr/"
CORRELATOR = "../../src/correlate.py"
NUM_WORKERS = 4
GROUP_SPAN = 10  # maximum difference in timestamp within a group (/2)
STALE_TIMEOUT = 40
# FILENAME_REGEX = r"^[^_]+_[^_]+_([^_]+)_rx([^_]+).corx$"

# Global state
poller = select.epoll()
work_queue = deque([])
running_procs = {}
groups = {}


def task_filename(task, ext='.npz'):
    group_key, filename1, filename2 = task
    rxid1 = extract_rxid(filename1)
    rxid2 = extract_rxid(filename2)
    noise_type = extract_noise_type(filename1)
    filename = "corr_{}_{}_{}-{}{}".format(group_key, noise_type,
                                           rxid1, rxid2, ext)
    return filename


def task_string(task):
    group_key, filename1, filename2 = task
    rxid1 = extract_rxid(filename1)
    rxid2 = extract_rxid(filename2)
    return "({}, {}, {})".format(group_key, rxid1, rxid2)


def run_task(task):
    _, filename1, filename2 = task
    path1 = os.path.join(CORX_DIR, filename1)
    path2 = os.path.join(CORX_DIR, filename2)

    output_filename = task_filename(task, '.npz')
    output_path = os.path.join(CORR_DIR, output_filename)

    command = [CORRELATOR, path1, path2, "-o", output_path]
    # print(command)
    # command = ['sleep', '5']

    proc = subprocess.Popen(command, stdout=subprocess.PIPE)
    fd = proc.stdout.fileno()
    running_procs[fd] = (task, proc)
    poller.register(proc.stdout, select.EPOLLHUP)

    print("Executing task: {} [PID: {}]".format(task_string(task), proc.pid))


def process_queue():
    while len(work_queue) > 0 and len(running_procs) < NUM_WORKERS:
        task = work_queue.popleft()
        run_task(task)


def add_corr_task(group_key, filename1, filename2):
    work_queue.append((group_key, filename1, filename2))


def get_group_key_mtime(path):
    """Group .corx files based on their modification time."""
    mtime = int(os.path.getmtime(path))  # getmtime returns a float timestamp
    if len(groups) == 0:
        return mtime
    keys = list(groups.keys())
    diffs = [abs(group - mtime) for group in keys]
    nearest = min(range(len(groups)), key=diffs.__getitem__)
    if diffs[nearest] <= GROUP_SPAN:
        return keys[nearest]
    else:
        return mtime


def get_group_key_prefix(path):
    """Group .corx files based on a date_time prefix in their filenames."""
    filename = os.path.basename(path)
    prefixes = filename.split('_', 2)
    if len(prefixes) != 3:
        print('Invalid filename: could not extract date_time prefix from "{}"'
              .format(path))
        return filename
    datestr = prefixes[0] + '_' + prefixes[1]
    date = datetime.strptime(datestr, '%Y%m%d_%H%M%S')
    return int(time.mktime(date.timetuple()))


def _extract_filename_field(filename, idx):
    fields_str = os.path.splitext(filename)[0]
    fields = fields_str.split('_')
    if len(fields) < 4:
        return None
    return fields[idx]


def extract_rxid(filename):
    return _extract_filename_field(filename, 3)


def extract_noise_type(filename):
    return _extract_filename_field(filename, 2)


def add_corx(filename):
    path = os.path.join(CORX_DIR, filename)
    if not os.path.isfile(path):
        print("Skipping {}: not a file or file does not exist".format(filename))
        return

    group_key = get_group_key_prefix(path)
    is_new_group = group_key not in groups
    if is_new_group:
        groups[group_key] = []

    rxid = extract_rxid(filename)
    if rxid is None:
        print("Skipping {}: invalid filename".format(filename))
        return

    print("Add {} to group {}{}".format(rxid, group_key,
                                        " (new group)" if is_new_group else ""))

    for filename2 in groups[group_key]:
        rxid2 = extract_rxid(filename2)
        print("New task: ({}, {}, {})".format(
              group_key, rxid, rxid2))
        add_corr_task(group_key, filename, filename2)

    groups[group_key].append(filename)
    process_queue()
    purge_stale(group_key)


def purge_stale(latest):
    work_queue_groups = [x[0] for x in work_queue]
    process_groups = [x[0][0] for x in running_procs.values()]

    deleted = []
    for group_key, filenames in groups.items():
        if group_key > latest - STALE_TIMEOUT:
            continue
        print("Purge group", group_key)
        if group_key in work_queue_groups:
            print(" ... cannot purge: group is in work queue")
            continue
        if group_key in process_groups:
            print(" ... cannot purge: group is in use by a running process")
            continue
        for filename in filenames:
            print("Delete", filename)
            path = os.path.join(CORX_DIR, filename)
            os.remove(path)
        deleted.append(group_key)
    for group_key in deleted:
        groups.pop(group_key)


def _main():
    # do not block stdin on read
    stdin_fd = sys.stdin.fileno()
    fcntl_flags = fcntl.fcntl(stdin_fd, fcntl.F_GETFL)
    fcntl.fcntl(stdin_fd, fcntl.F_SETFL, fcntl_flags | os.O_NONBLOCK)

    # register stdin
    poller.register(stdin_fd, select.EPOLLIN)

    # wait for stdin and child processes
    while True:
        for fd, _ in poller.poll():
            if fd == stdin_fd:
                print("Received new input")
                try:
                    for line in sys.stdin:
                        add_corx(line.strip('\n'))
                except IOError:
                    # Python 2.x seems to throw IOError for when reaching EOF.
                    # Ignore it. What could possibly go wrong?
                    pass
            else:
                task, proc = running_procs[fd]
                poller.unregister(fd)
                running_procs.pop(fd)

                print("Task done: {} [PID: {}]"
                      .format(task_string(task), proc.pid))

                # write log
                stdout, _ = proc.communicate()
                log_filename = task_filename(task, '.log')
                log_path = os.path.join(LOG_DIR, log_filename)
                with open(log_path, 'wb') as f:
                    f.write(stdout)

                process_queue()
                print("Tasks: {} running; {} in queue".format(len(running_procs),
                                                              len(work_queue)))

        print("\nWaiting for new events...")


if __name__ == '__main__':
    _main()
