#!/usr/bin/env python

import sys
import time

sys.path.insert(0, '../src')
import multicorx_remote

NUM_CAPTURE_NOISE_OFF_REPEATS = 1
NUM_NOISE_GENS = 4

def capture(puppets, noise_type):
    date = time.strftime('%Y%m%d_%H%M%S')
    filename_common = "{corx_path}/{date}_{noise_type}_rx".format(
                       corx_path=args.corx_path,
                       date=date,
                       noise_type=noise_type)
    filename = filename_common + "{hostid}{rxid}.corx"

    puppets.send("output " + filename)
    puppets.send("capture")
    puppets.wait_idle()

    # upload data...
    if args.upload_server != '':
        puppets.send("exec upload_client/upload.sh "
                     "{upload_server} {files}*.corx"
                     .format(upload_server=args.upload_server,
                             files=filename_common))

    # delete old data...
    if not args.keep:
        puppets.send("exec rm {files}*.corx".format(files=filename_common))


def capture_loop(puppets):
    for i in range(NUM_NOISE_GENS):
        print("Noise #{} off".format(i))
        puppets.send("exec ./noise.sh off {}".format(i))
        for j in range(NUM_CAPTURE_NOISE_OFF_REPEATS):
            capture(puppets, 'NN')

        print("Noise #{} on".format(i))
        puppets.send("exec ./noise.sh on {}".format(i))
        capture(puppets, 'N%d' % i)


def run():
    addresses = multicorx_remote.parse_addresses(args.remote)
    puppets = multicorx_remote.MultiCorxRemote(addresses)
    puppets.send("standby")
    puppets.send("exec ./noise.sh init")
    if args.runs < 0:
        while True:
            capture_loop(puppets)
    else:
        for _ in range(args.runs):
            capture_loop(puppets)
    puppets.close()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--remote', '-r', type=str, default='127.0.0.1:7331',
                        help='Multicorx servers to connect to as a '
                             'comma-separated list of IP:PORT pairs '
                             '(default: 127.0.0.1:7331)')
    parser.add_argument('--runs', '-n', type=int, default=1,
                        help='Number of times to repeat capture loop.'
                             'Set to -1 to repeat indefinitely.')
    parser.add_argument('--corx-path', type=str, default='.',
                        help='Corx output path on remote hosts.')
    parser.add_argument('--upload-server', type=str, default='',
                        help='IP address of upload server. Leave blank to'
                             'disable upload.')
    parser.add_argument('--keep', action='store_true',
                        help='Do not delete .corx files on the remote hosts.')
    args = parser.parse_args()
    run()
