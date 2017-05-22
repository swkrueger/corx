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
MODE_REGEX_PATTERN = r'\[#\d+\] MODE changed from [A-Z_]+ to ([A-Z_]+)'
MODE_REGEX = re.compile(MODE_REGEX_PATTERN)

# Globals
poller = select.epoll()
subprocs = {}
stderrs = {}

# Sockets
server = None  # Socket server
clients = {}  # Socket clients
notify_later = []

# Args
CORX_CMD = '../build/corx_rx'
SUBPROC_LOG = sys.stdout
INACTIVE_LOG_PATH = None
ALLOW_EXEC = False
HOST_ID = 'A'


class Subproc(object):
    def __init__(self, rxid, proc):
        self.rxid = rxid
        self.proc = proc
        self.state = 'STOPPED'
        self.dirty_state = False
        self.mode = 'STOP'


def create_corx(rxid):
    args = [arg.format(rxid=rxid, hostid=HOST_ID) for arg in CORX_ARGS]
    cmd = [CORX_CMD] + args
    print("Run", cmd)
    proc = subprocess.Popen(cmd,
                            stdout=subprocess.PIPE,
                            stdin=subprocess.PIPE,
                            stderr=subprocess.PIPE)
    poller.register(proc.stdout, select.EPOLLHUP | select.EPOLLIN)
    poller.register(proc.stderr, select.EPOLLIN)
    fd = proc.stdout.fileno()
    subprocs[fd] = Subproc(rxid=rxid, proc=proc)
    errfd = proc.stderr.fileno()
    stderrs[errfd] = subprocs[fd]
    
    # Do not block read
    # This doesn't work properly with Python 2.x
    # (see http://stackoverflow.com/a/1810703)
    fcntl_flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fcntl_flags | os.O_NONBLOCK)
    fcntl_flags = fcntl.fcntl(errfd, fcntl.F_GETFL)
    fcntl.fcntl(errfd, fcntl.F_SETFL, fcntl_flags | os.O_NONBLOCK)


def process_command(line, from_=None):
    """Send a command to all receivers."""
    cmd_args = line.strip().split(' ', 1)
    cmd = cmd_args[0].upper()
    args = cmd_args[1] if len(cmd_args) > 1 else ''

    if cmd == 'NOTIFY' and from_ in clients:
        # (NOTE: we will never write to a client socket except for
        #        inactivity notifications)
        notify_now = False
        if check_all_inactive():
            for subproc in subprocs.values():
                # State is dirty: wait for state transition
                if subproc.dirty_state:
                    break
            else:
                notify_now = True

        if notify_now:
            notify_client(from_)
        else:
            notify_later.append(from_)

    elif cmd == 'EXEC':
        if not ALLOW_EXEC:
            print('Command execution denied. Run multicorx with the '
                  '--allow-exec flag to enable command execution.')
        else:
            print('Execute shell command:', args)
            subprocess.call(args, shell=True)

    else:
        # Command is for subprocs
        if cmd in ['STOP', 'STANDBY', 'LOCK', 'CAPTURE']:
            for subproc in subprocs.values():
                if subproc.mode != cmd:
                    subproc.dirty_state = True

        for subproc in subprocs.values():
            # inject RXID
            proc_cmd = line.format(rxid=subproc.rxid, hostid=HOST_ID)
            print(proc_cmd)
            # send command
            subproc.proc.stdin.write(proc_cmd.encode())
            subproc.proc.stdin.flush()


def read_stdin():
    """Forward commands from stdin directly to receivers."""
    for line in sys.stdin:
        process_command(line)


def check_all_inactive():
    for subproc in subprocs.values():
        if subproc.state not in ("STOPPED", "STANDBY", "DEAD"):
            inactive = False
            return False
    return True


def notify_client(client_fd):
    """Notify a client that all receivers are idling (inactive)."""
    print("(write to socket client -- may block until read)", end='')
    clients[client_fd].send(b'INACTIVE\n')
    print(" ..done")


def read_corx_stdout(fd):
    subproc = subprocs[fd]

    active_to_inactive = False

    for line in subproc.proc.stdout:
        line_str = line.decode()

        # parse state
        m = STATE_REGEX.match(line_str)
        if m:
            new_state = m.groups()[0]
            inactive_before = check_all_inactive()
            subproc.state = new_state
            subproc.dirty_state = False
            print("RX #{} changed state to {}"
                  .format(subproc.rxid, new_state))
            print("States:", [p.state for p in subprocs.values()])
            if not inactive_before and check_all_inactive():
                active_to_inactive = True

        # parse mode
        m = MODE_REGEX.match(line_str)
        if m:
            new_mode = m.groups()[0]
            subproc.mode = new_mode
            print("RX #{} changed mode to {}"
                  .format(subproc.rxid, new_mode))
            print("Modes:", [p.mode for p in subprocs.values()])

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
        for client_fd in notify_later:
            if client_fd in clients:
                notify_client(client_fd)
        notify_later.clear()


def read_corx_stderr(fd):
    subproc = stderrs[fd]
    for line in subproc.proc.stderr:
        # forward output
        line_str = line.decode()
        SUBPROC_LOG.write("{}E|".format(subproc.rxid))
        SUBPROC_LOG.write(line_str)
        if len(line_str) > 0 and line_str[-1] != '\n':
            SUBPROC_LOG.write('\n')
    SUBPROC_LOG.flush()


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
            process_command(line_str, from_=fd)
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
    global CORX_CMD, SUBPROC_LOG, INACTIVE_LOG_PATH, ALLOW_EXEC, HOST_ID
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
    parser.add_argument('--hostid', type=str, default='A',
                        help='Unique identifier that will replace "{hostid}" '
                              'in commands.')
    parser.add_argument('--socket', action='store_true',
                        help='Start a TCP server.')
    parser.add_argument('--host', type=str, default='127.0.0.1',
                        help='Hostname the socket server should bind to.')
    parser.add_argument('--port', type=int, default=7331,
                        help='Port the socker server should bind to.')
    parser.add_argument('--allow-exec', dest='allow_exec', action='store_true',
                        help='Allow arbitrary shell commands to be executed'
                             'via the EXEC command. WARNING: This will provide'
                             'full shell access over an insecure channel.'
                             'Only enable this on a trusted network!')
    args = parser.parse_args()
    SUBPROC_LOG = args.log
    CORX_CMD = args.corx
    INACTIVE_LOG_PATH = args.inactive
    ALLOW_EXEC = args.allow_exec
    HOST_ID = args.hostid

    if args.socket:
        global server
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
            elif fd in stderrs:
                if event & select.EPOLLIN:
                    read_corx_stderr(fd)
                if event & select.EPOLLHUP:
                    poller.unregister(fd)
                    stderrs.pop(fd)
            elif fd in clients:
                if event & select.EPOLLHUP:
                    handle_client_sighup(fd)
                elif event & select.EPOLLIN:
                    handle_client_input(fd)
                else:
                    print("Warning: unknown event from client FD")
            else:
                print("Warning: event from unknown FD")

    if server is not None:
        server.close()

if __name__ == '__main__':
    _main()
