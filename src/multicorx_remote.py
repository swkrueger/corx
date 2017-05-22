#!/usr/bin/env python

"""Control multiple multicorx instances remotely via sockets.

Examples (CLI interface):

 - Connect to multicorx running on localhost, read commands from input
   until SIGHUP:

       $ ./multicorx_remote

 - Connect to multicorx running on localhost, run the given commands and exit:

       $ ./multicorx_remote standby capture

 - Connect to two remote multicorx servers
   (one using the default port, another with a custom port):

       $ ./multicorx_remote -r 192.168.0.100,192.168.0.101:1234

 - Command three multicorx servers to capture data and wait until completion:
   (one using the default port, another with a custom port):

       $ ./multicorx_remote --remote=odroid1,odroid2,odroid3 capture wait
"""

from __future__ import print_function

import select
import socket

class MultiCorxRemote(object):
    def __init__(self, addresses):
        self.poller = select.epoll()
        self.connections = {}
        for address in addresses:
            conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            conn.connect(address)
            self.connections[conn.fileno()] = conn
            self.poller.register(conn.fileno(), select.EPOLLIN)
            print("[{}] Connected to {}:{}"
                  .format(conn.fileno(), address[0], address[1]))
        self.notified = {}
        self._clear_notified()

    def send(self, command):
        # handle any pending reads or SIGHUPs
        self.poll(0)
        # send command
        for conn in self.connections.values():
            conn.send(command + '\n')

    def poll(self, timeout=-1):
        for fd, event in self.poller.poll(timeout=timeout):
            if fd in self.connections:
                if event & select.EPOLLHUP:
                    # Close connection
                    self._close_connection(fd)
                if event & select.EPOLLIN:
                    # Read
                    data = self.connections[fd].recv(1024)
                    if len(data) == 0:  # EOF
                        self._close_connection(fd)
                    else:
                        for line in data.split('\n'):
                            if line == 'INACTIVE':
                                print("[{}] Inactive".format(fd))
                                self.notified[fd] = True

    def wait_idle(self):
        # handle any pending reads or SIGHUPs
        self.poll(0)
        # ask multicorx to notify us
        self._clear_notified()
        for conn in self.connections.values():
            conn.send('NOTIFY\n')

        print("Waiting for all receivers to become inactive (idle)")
        while True:
            self.poll()
            for notified in self.notified.values():
                if not notified:
                    continue
            break

    def num_connections(self):
        return len(self.connections)

    def close(self):
        """Close all connections."""
        fds = self.connections.keys()
        for fd in fds:
            self._close_connection(fd)

    def _close_connection(self, fd):
        self.poller.unregister(fd)
        conn = self.connections.pop(fd)
        self.notified.pop(fd)
        conn.close()
        print("[{}] Disconnect".format(fd))

    def _clear_notified(self):
        for fd in self.connections.keys():
            self.notified[fd] = False


def parse_addresses(string):
    addresses = []
    for pair in string.split(','):
        hostport = pair.strip().split(':', 1)
        host = hostport[0]
        port = int(hostport[1]) if len(hostport) > 1 else 7331
        addresses.append((host, port))
    return addresses


def _main():
    import argparse
    parser = argparse.ArgumentParser(
            description=__doc__,
            formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--remote', '-r', type=str, default='127.0.0.1:7331',
                        help='multicorx servers to connect to as a '
                             'comma-separated list of IP:PORT pairs '
                             '(default: 127.0.0.1:7331)')
    parser.add_argument('commands', metavar='command', nargs='*',
                        help='commands to send to multicorx servers (default:'
                             ' read commands from stdin until SIGHUP)')
    args = parser.parse_args()

    addresses = parse_addresses(args.remote)
    remote = MultiCorxRemote(addresses)

    def run_command(command):
        if command.strip().upper() == 'WAIT':
            remote.wait_idle()
        else:
            remote.send(command)

    if args.commands:
        for command in args.commands:
            print('Command:', command)
            run_command(command)
    else:
        while remote.num_connections() > 0:
            line = raw_input('> ')
            run_command(line.strip())
        print("No active connections left, exiting...")
    remote.close()


if __name__ == '__main__':
    _main()
