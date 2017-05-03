#!/usr/bin/env python3

"""Control multiple correlators on a single host in parallel."""

from __future__ import print_function

import fcntl
import os
import subprocess
import select
import sys
import re
import signal
import socket

from collections import namedtuple

# We only support Python 3
if sys.version_info < (3, 0):
    sys.stdout.write("Only Python 3.x is supported\n")
    sys.exit(1)

# Settings
CORX_ARGS = ['--flagfile=flags.cfg',
             '--input=rtlsdr',
             '--wisdom=/tmp/corx{rxid}.wisdom',
             '--device_index={rxid}',
             '--interactive']
STATE_REGEX_PATTERN = r'\[#\d+\] STATE changed from [A-Z_]+ to ([A-Z_]+)'
STATE_REGEX = re.compile(STATE_REGEX_PATTERN)

# Globals
poller = select.epoll()
subprocs = {}
Subproc = namedtuple('Subproc', ['rxid', 'proc', 'state'])

# Sockets
server = None  # Socket server
clients = {}  # Socket clients

# Args
CORX_CMD = '../build/corx_rx'
SUBPROC_LOG = sys.stdout
INACTIVE_LOG_PATH = None



def create_corx(rxid):
    args = [arg.format(rxid=rxid) for arg in CORX_ARGS]
    cmd = [CORX_CMD] + args
    print("Run", cmd)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stdin=subprocess.PIPE)
    poller.register(proc.stdout, select.EPOLLHUP | select.EPOLLIN)
    fd = proc.stdout.fileno()
    subprocs[fd] = Subproc(rxid=rxid, proc=proc, state=["STOPPED"])
    # TODO: pipe and forward stderr
    
    # Do not block read
    # This doesn't work properly with Python 2.x
    # (see http://stackoverflow.com/a/1810703)
    fcntl_flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fcntl_flags | os.O_NONBLOCK)


def send_command(cmd):
    """Send a command to all receivers."""
    for subproc in subprocs.values():
        # inject RXID
        proc_cmd = cmd.format(rxid=subproc.rxid)
        # send command
        subproc.proc.stdin.write(proc_cmd.encode())
        subproc.proc.stdin.flush()


def read_stdin():
    """Forward commands from stdin directly to receivers."""
    for line in sys.stdin:
        send_command(line)


def check_all_inactive():
    for subproc in subprocs.values():
        if subproc.state[0] not in ("STOPPED", "STANDBY", "DEAD"):
            inactive = False
            return False
    return True


def read_corx_stdout(fd):
    subproc = subprocs[fd]

    active_to_inactive = False

    for line in subproc.proc.stdout:
        # parse state
        line_str = line.decode()
        m = STATE_REGEX.match(line_str)
        if m:
            new_state = m.groups()[0]
            inactive_before = check_all_inactive()
            subproc.state[0] = new_state
            print("RX #{} changed state to {}"
                  .format(subproc.rxid, new_state))
            print("States:", [p.state[0] for p in subprocs.values()])
            if (not inactive_before and check_all_inactive()):
                active_to_inactive = True

        # forward output
        SUBPROC_LOG.write("{}|".format(subproc.rxid))
        SUBPROC_LOG.write(line_str)
        if len(line_str) > 0 and line_str[-1] != '\n':
            SUBPROC_LOG.write('\n')
    SUBPROC_LOG.flush()

    if active_to_inactive:
        print("*** All the receivers are now inactive")
        if INACTIVE_LOG_PATH is not None:
            print("(write to inactive -- may block until read)", end='')
            inactive = open(INACTIVE_LOG_PATH, 'w')
            inactive.write("INACTIVE\n")
            inactive.flush()
            inactive.close()  # send EOF
            print(" ..done")
        for client in clients.values():
            # TODO: only write if requested by client
            print("(write to socket client -- may block until read)", end='')
            client.send(b'INACTIVE\n')
            print(" ..done")


def handle_corx_sighup(fd):
    # process ended
    poller.unregister(fd)
    subproc = subprocs.pop(fd)
    print("RX #{} has stopped".format(subproc.rxid))


def handle_client_sighup(fd):
    close_client(fd)
    print('Client connection closed (HUP)')


def handle_client_input(fd):
    # TODO: buffer input
    data = clients[fd].recv(1024)
    if data:
        start = 0
        while start < len(data):
            stop = data.find(b'\n', start)
            if stop == -1:
                stop = len(data)-1
            line = data[start:stop+1]
            start = stop + 1

            line_str = line.decode().strip() + '\n'
            send_command(line_str)
    else:
        close_client(fd)
        print('Client connection closed (EOF)')


def close_client(fd):
    poller.unregister(fd)
    conn = clients.pop(fd)
    conn.close()


def signal_handler(signum, frame):
    # Forward signal to subprocs
    for subproc in subprocs.values():
        subproc.proc.send_signal(signum)


def _main():
    global CORX_CMD, SUBPROC_LOG, INACTIVE_LOG_PATH
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', type=argparse.FileType('w'),
                        default=sys.stdout,
                        help='File to write output from subprocesses to.')
    parser.add_argument('--corx', type=str, default=CORX_CMD,
                        help='Path to corx program.')
    parser.add_argument('--inactive', type=str, default=None,
                        help='Write a line to this file when all receivers '
                             'are inactive.')
    parser.add_argument('--num', type=int, default=4,
                        help='Number of receivers.')
    parser.add_argument('--socket', action='store_true',
                        help='Start a TCP server.')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                        help='Hostname the socket server should bind to.')
    parser.add_argument('--port', type=int, default=7331,
                        help='Port the socker server should bind to.')
    args = parser.parse_args()
    SUBPROC_LOG = args.log
    CORX_CMD = args.corx
    INACTIVE_LOG_PATH = args.inactive

    if args.socket:
        global server
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setblocking(0)
        server.bind((args.host, args.port))
        server.listen(1)
        poller.register(server.fileno(), select.EPOLLIN)
        print('Started socket on {}:{}'.format(args.host, args.port))

    for rxid in range(args.num):
        create_corx(rxid)

    # do not block stdin on read
    stdin_fd = sys.stdin.fileno()
    fcntl_flags = fcntl.fcntl(stdin_fd, fcntl.F_GETFL)
    fcntl.fcntl(stdin_fd, fcntl.F_SETFL, fcntl_flags | os.O_NONBLOCK)

    # register stdin
    poller.register(stdin_fd, select.EPOLLIN)

    # register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGQUIT, signal_handler)
    signal.signal(signal.SIGPIPE, signal_handler)

    # wait for stdin and child processes
    while True:
        if len(subprocs) == 0:
            # no subprocesses left
            print("No subprocesses left... exiting...")
            break

        for fd, event in poller.poll():
            if fd == stdin_fd:
                read_stdin()
            elif server is not None and fd == server.fileno():
                conn, address = server.accept()
                # conn.setblocking(0)
                fd = conn.fileno()
                poller.register(fd, select.EPOLLIN)
                clients[fd] = conn
                print('New connection from', address)
            elif fd in subprocs:
                if event & select.EPOLLIN:
                    read_corx_stdout(fd)
                if event & select.EPOLLHUP:
                    handle_corx_sighup(fd)
            elif fd in clients:
                if event & select.EPOLLHUP:
                    handle_client_sighup(fd)
                elif event & select.EPOLLIN:
                    handle_client_input(fd)
                else:
                    print("Warning: unknown event from client FD")
            else:
                print("Warning: event from unknown FD")

if __name__ == '__main__':
    _main()
