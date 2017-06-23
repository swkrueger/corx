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

exec_procs = {}
exec_when_idle = []
notify_exec_done = []  # list of FDs to notify when all exec and
                       # exec_when_idle jobs are finished

# Sockets
server = None  # Socket server
clients = {}  # Socket clients
notify_later = []  # list of FDs to notify when receivers are inactive

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


def shell_exec(cmd):
    proc = subprocess.Popen(cmd,
                            shell=True,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE)
    poller.register(proc.stdout, select.EPOLLHUP | select.EPOLLIN)
    poller.register(proc.stderr, select.EPOLLHUP | select.EPOLLIN)

    fd = proc.stdout.fileno()
    exec_procs[fd] = proc
    errfd = proc.stderr.fileno()
    exec_procs[errfd] = proc

    # Do not block read
    # This doesn't work properly with Python 2.x
    # (see http://stackoverflow.com/a/1810703)
    fcntl_flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fcntl_flags | os.O_NONBLOCK)
    fcntl_flags = fcntl.fcntl(errfd, fcntl.F_GETFL)
    fcntl.fcntl(errfd, fcntl.F_SETFL, fcntl_flags | os.O_NONBLOCK)

    pid = proc.pid
    print('[S{}] Execute shell command:'.format(pid), cmd)


def process_command(line, from_=None):
    """Send a command to all receivers."""
    cmd_args = line.strip().split(' ', 1)
    cmd = cmd_args[0].upper()
    args = cmd_args[1] if len(cmd_args) > 1 else ''

    if cmd == 'NOTIFY' and from_ in clients:
        # (NOTE: we will never write to a client socket except for
        #        inactivity notifications)
        if check_all_inactive_nondirty():
            # notify now
            notify_client_inactive(from_)
        else:
            # notify later
            notify_later.append(from_)

    elif cmd == 'NOTIFY_EXEC':
        notify_exec_done.append(from_)
        notify_if_exec_done()

    elif cmd == 'EXEC' or cmd == 'EXEC_WHEN_IDLE':
        if not ALLOW_EXEC:
            print('Command execution denied. Run multicorx with the '
                  '--allow-exec flag to enable command execution.')
        else:
            if cmd == 'EXEC' or check_all_inactive_nondirty():
                shell_exec(args)
            else:
                exec_when_idle.append(args)

    else:
        # Command is for subprocs
        if cmd in ['STOP', 'STANDBY', 'LOCK', 'CAPTURE']:
            for subproc in subprocs.values():
                if subproc.mode != cmd:
                    subproc.dirty_state = True

        for subproc in subprocs.values():
            # inject RXID
            proc_cmd = line.format(rxid=subproc.rxid, hostid=HOST_ID)
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
            return False
    return True


def check_all_inactive_nondirty():
    if check_all_inactive():
        for subproc in subprocs.values():
            # State is dirty: wait for state transition
            if subproc.dirty_state:
                break
        else:
            return True
    return False


def notify_client(client_fd, msg):
    print("(write to socket client -- may block until read)", end='')
    clients[client_fd].send(msg)
    print(" ..done")


def notify_client_inactive(client_fd):
    """Notify a client that all receivers are idling (inactive)."""
    notify_client(client_fd, b'INACTIVE\n')


def notify_if_exec_done():
    """Notify clients that all exec and exec_when_idle jobs are done."""
    if len(exec_when_idle) == 0 and len(exec_procs) == 0:
        print("*** All the exec jobs are done.")
        for client_fd in notify_exec_done:
            if client_fd in clients:
                notify_client(client_fd, b'EXEC_DONE\n')
        notify_exec_done.clear()


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
            sorted_subprocs = sorted(subprocs.values(),
                                     key=lambda x: x.rxid)
            state_strs = ["{}={}".format(p.rxid, p.state)
                          for p in sorted_subprocs]
            print("States:", ' '.join(state_strs))
            if not inactive_before and check_all_inactive():
                active_to_inactive = True

        # parse mode
        m = MODE_REGEX.match(line_str)
        if m:
            new_mode = m.groups()[0]
            subproc.mode = new_mode
            print("RX #{} changed mode to {}"
                  .format(subproc.rxid, new_mode))
            sorted_subprocs = sorted(subprocs.values(),
                                     key=lambda x: x.rxid)
            mode_strs = ["{}={}".format(p.rxid, p.mode)
                         for p in sorted_subprocs]
            print("Modes:", ' '.join(mode_strs))

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
                notify_client_inactive(client_fd)
        notify_later.clear()

        if len(exec_when_idle) > 0:
            print("Executing pending exec_when_idle jobs...")
            for cmd in exec_when_idle:
                shell_exec(cmd)
            exec_when_idle.clear()
            # TODO: notify_exec


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


def read_exec_output(fd):
    proc = exec_procs[fd]
    stderr = (proc.stderr.fileno() == fd)
    stream = proc.stderr if stderr else proc.stdout

    for line in stream:
        # forward output
        line_str = line.decode()
        SUBPROC_LOG.write("[S{}]{}|".format(proc.pid,
                                            'E' if stderr else ''))
        SUBPROC_LOG.write(line_str)
        if len(line_str) > 0 and line_str[-1] != '\n':
            SUBPROC_LOG.write('\n')

    SUBPROC_LOG.flush()


def handle_corx_sighup(fd):
    # process ended
    poller.unregister(fd)
    subproc = subprocs.pop(fd)
    print("RX #{} has stopped".format(subproc.rxid))


def handle_exec_sighup(fd):
    poller.unregister(fd)
    proc = exec_procs.pop(fd)
    stderr = (proc.stderr.fileno() == fd)
    if not stderr:
        pid = proc.pid
        proc.poll()
        retcode = proc.returncode
        print('[S{}] Shell command exited with return code {}'
              .format(pid, retcode))
    notify_if_exec_done()


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
            elif fd in exec_procs:
                if event & select.EPOLLIN:
                    read_exec_output(fd)
                if event & select.EPOLLHUP:
                    handle_exec_sighup(fd)
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
